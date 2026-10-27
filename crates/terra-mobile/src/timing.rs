//! Sensor timing probe shared by both phone apps.
//!
//! Ports the CSV recorder of the iOS `SensorTimingProbe`. Off until
//! [`SensorTimingRecorder::start`]; while recording, each hop of each sensor
//! stream appends one row and a background thread writes them once a second to
//! `<directory>/<condition>-<utc>.csv`. The file format is unchanged, so
//! `analyze_sensor_timing.py` reads both phones:
//! `stream,seq,t_sensor,t_os,t_queue,t_done,a,b`, empty where a hop does not apply.
//! The app supplies the stamps and the `meta` / `battery` rows.
use crate::ControllerError;
use std::collections::HashMap;
use std::fs::{File, OpenOptions};
use std::io::Write;
use std::path::PathBuf;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Condvar, Mutex};
use std::thread::JoinHandle;
use std::time::Duration;

/// About 20 s of every stream at full rate; rows beyond this are counted as lost.
const MAX_BUFFERED_ROWS: usize = 20_000;

/// One timing stream. See the iOS probe for what each column means per stream.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, uniffi::Enum)]
pub enum TimingStream {
    /// IMU sample.
    Imu,
    /// AR frame.
    Ar,
    /// Object detector result.
    Detector,
    /// GPS fix.
    Gps,
    /// Compass heading.
    Heading,
    /// Control tick.
    Tick,
    /// Long control-queue block.
    Block,
    /// IMU with no Terra code in the handler.
    BareImu,
    /// AR with no Terra code in the handler.
    BareAr,
    /// Thermal state and low-power mode, once a second.
    Meta,
    /// Battery level and state.
    Battery,
}

impl TimingStream {
    fn name(self) -> &'static str {
        match self {
            Self::Imu => "imu",
            Self::Ar => "ar",
            Self::Detector => "detector",
            Self::Gps => "gps",
            Self::Heading => "heading",
            Self::Tick => "tick",
            Self::Block => "block",
            Self::BareImu => "bare_imu",
            Self::BareAr => "bare_ar",
            Self::Meta => "meta",
            Self::Battery => "battery",
        }
    }
}

/// Header facts written at the top of the CSV.
#[derive(Clone, Debug, PartialEq, uniffi::Record)]
pub struct TimingHeader {
    /// Experiment condition code, e.g. `C2-terra-idle`.
    pub condition: String,
    /// OS version string.
    pub os: String,
    /// `debug` or `release`.
    pub build: String,
    /// Device model identifier.
    pub model: String,
    /// Sensor clock at start, seconds.
    pub media_time_at_start: f64,
    /// Unix time at start, seconds.
    pub wall_time_at_start: f64,
}

struct Row {
    stream: TimingStream,
    seq: u64,
    values: [f64; 6],
}

#[derive(Default)]
struct Probe {
    file: Option<File>,
    path: Option<PathBuf>,
    rows: Vec<Row>,
    sequence: HashMap<TimingStream, u64>,
    counts: HashMap<TimingStream, u64>,
    lost: u64,
}

struct Shared {
    probe: Mutex<Probe>,
    wake: Condvar,
    running: AtomicBool,
}

/// CSV recorder for the timing experiment. `record` is cheap: one lock, one push.
#[derive(uniffi::Object)]
pub struct SensorTimingRecorder {
    shared: Arc<Shared>,
    flusher: Mutex<Option<JoinHandle<()>>>,
}

#[uniffi::export]
impl SensorTimingRecorder {
    #[uniffi::constructor]
    /// Not recording.
    pub fn new() -> Arc<Self> {
        Arc::new(Self {
            shared: Arc::new(Shared {
                probe: Mutex::new(Probe::default()),
                wake: Condvar::new(),
                running: AtomicBool::new(false),
            }),
            flusher: Mutex::new(None),
        })
    }

    /// Stop any recording, then start a new CSV in `directory` (created if
    /// missing). Returns the file path. Returns `InvalidInput` if the file
    /// cannot be created.
    pub fn start(
        &self,
        directory: String,
        header: TimingHeader,
    ) -> Result<String, ControllerError> {
        self.stop()?;
        let fail = |e: std::io::Error| ControllerError::InvalidInput {
            message: format!("sensor timing file: {e}"),
        };
        let directory = PathBuf::from(directory);
        std::fs::create_dir_all(&directory).map_err(fail)?;
        let safe: String = header
            .condition
            .chars()
            .filter(|c| c.is_alphanumeric() || *c == '-' || *c == '_')
            .collect();
        let name = format!(
            "{}-{}.csv",
            if safe.is_empty() { "run" } else { &safe },
            utc_stamp(header.wall_time_at_start)
        );
        let path = directory.join(name);
        let mut file = OpenOptions::new()
            .create(true)
            .truncate(true)
            .write(true)
            .open(&path)
            .map_err(fail)?;
        write!(
            file,
            "# terra sensor timing v1\n# condition={}\n# os={}\n# build={}\n# model={}\n# media_time_at_start={}\n# wall_time_at_start={}\nstream,seq,t_sensor,t_os,t_queue,t_done,a,b\n",
            header.condition, header.os, header.build, header.model,
            header.media_time_at_start, header.wall_time_at_start
        )
        .map_err(fail)?;
        {
            let mut probe = self.lock()?;
            *probe = Probe {
                file: Some(file),
                path: Some(path.clone()),
                ..Probe::default()
            };
        }
        self.shared.running.store(true, Ordering::SeqCst);
        let shared = Arc::clone(&self.shared);
        let handle = std::thread::Builder::new()
            .name("terra.sensor-timing".into())
            .spawn(move || {
                while shared.running.load(Ordering::SeqCst) {
                    let Ok(guard) = shared.probe.lock() else {
                        return;
                    };
                    let Ok((mut guard, _)) =
                        shared.wake.wait_timeout(guard, Duration::from_secs(1))
                    else {
                        return;
                    };
                    flush(&mut guard);
                }
            })
            .map_err(fail)?;
        *self.flusher.lock().map_err(|_| ControllerError::Internal)? = Some(handle);
        Ok(path.to_string_lossy().into_owned())
    }

    /// One row. Pass NaN for a hop that does not apply. Ignored when not recording.
    #[allow(clippy::too_many_arguments)]
    pub fn record(
        &self,
        stream: TimingStream,
        sensor: f64,
        os: f64,
        queue: f64,
        done: f64,
        a: f64,
        b: f64,
    ) -> Result<(), ControllerError> {
        let mut probe = self.lock()?;
        if probe.file.is_none() {
            return Ok(());
        }
        // Sequence advances even for dropped rows, so the analysis can count the holes.
        let seq = probe.sequence.entry(stream).or_insert(0);
        let current = *seq;
        *seq += 1;
        if probe.rows.len() >= MAX_BUFFERED_ROWS {
            probe.lost += 1;
            return Ok(());
        }
        *probe.counts.entry(stream).or_insert(0) += 1;
        probe.rows.push(Row {
            stream,
            seq: current,
            values: [sensor, os, queue, done, a, b],
        });
        Ok(())
    }

    /// Flush, close the file and stop the writer thread.
    pub fn stop(&self) -> Result<(), ControllerError> {
        self.shared.running.store(false, Ordering::SeqCst);
        self.shared.wake.notify_all();
        let handle = self
            .flusher
            .lock()
            .map_err(|_| ControllerError::Internal)?
            .take();
        if let Some(handle) = handle {
            let _ = handle.join();
        }
        let mut probe = self.lock()?;
        flush(&mut probe);
        probe.file = None;
        Ok(())
    }

    /// Whether a file is open.
    pub fn is_recording(&self) -> Result<bool, ControllerError> {
        Ok(self.lock()?.file.is_some())
    }

    /// Path of the current or last file.
    pub fn current_file(&self) -> Result<Option<String>, ControllerError> {
        Ok(self
            .lock()?
            .path
            .as_ref()
            .map(|p| p.to_string_lossy().into_owned()))
    }

    /// Rows kept per stream, plus `lost` when the buffer overflowed.
    pub fn row_counts(&self) -> Result<HashMap<String, u64>, ControllerError> {
        let probe = self.lock()?;
        let mut out: HashMap<String, u64> = probe
            .counts
            .iter()
            .map(|(s, n)| (s.name().to_owned(), *n))
            .collect();
        if probe.lost > 0 {
            out.insert("lost".into(), probe.lost);
        }
        Ok(out)
    }
}

impl SensorTimingRecorder {
    fn lock(&self) -> Result<std::sync::MutexGuard<'_, Probe>, ControllerError> {
        self.shared
            .probe
            .lock()
            .map_err(|_| ControllerError::Internal)
    }
}

impl Drop for SensorTimingRecorder {
    fn drop(&mut self) {
        let _ = self.stop();
    }
}

fn flush(probe: &mut Probe) {
    if probe.rows.is_empty() {
        return;
    }
    let rows = std::mem::take(&mut probe.rows);
    let Some(file) = probe.file.as_mut() else {
        return;
    };
    let mut text = String::with_capacity(rows.len() * 96);
    for row in rows {
        text.push_str(row.stream.name());
        text.push(',');
        text.push_str(&row.seq.to_string());
        for value in row.values {
            text.push(',');
            if value.is_finite() {
                text.push_str(&format!("{value:.6}"));
            }
        }
        text.push('\n');
    }
    let _ = file.write_all(text.as_bytes());
}

/// `YYYY-MM-DDTHHMMSSZ` for a Unix time, as the iOS file names use.
fn utc_stamp(unix: f64) -> String {
    let secs = if unix.is_finite() && unix >= 0.0 {
        unix as i64
    } else {
        0
    };
    let (days, rem) = (secs.div_euclid(86_400), secs.rem_euclid(86_400));
    // Civil date from days since 1970-01-01 (Howard Hinnant's algorithm).
    let z = days + 719_468;
    let era = z.div_euclid(146_097);
    let doe = z - era * 146_097;
    let yoe = (doe - doe / 1460 + doe / 36_524 - doe / 146_096) / 365;
    let doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    let mp = (5 * doy + 2) / 153;
    let day = doy - (153 * mp + 2) / 5 + 1;
    let month = if mp < 10 { mp + 3 } else { mp - 9 };
    let year = yoe + era * 400 + i64::from(month <= 2);
    format!(
        "{year:04}-{month:02}-{day:02}T{:02}{:02}{:02}Z",
        rem / 3600,
        rem % 3600 / 60,
        rem % 60
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    /// File names use the iOS stamp format.
    fn stamp() {
        assert_eq!(utc_stamp(0.0), "1970-01-01T000000Z");
        // 2026-10-27T18:37:41Z, a real C2 file name.
        assert_eq!(utc_stamp(1_793_126_261.0), "2026-10-27T183741Z");
    }

    #[test]
    /// Rows land in the iOS CSV format with per-stream sequences and empty NaN cells.
    fn writes_csv() {
        let dir = std::env::temp_dir().join(format!("terra-timing-{}", std::process::id()));
        let recorder = SensorTimingRecorder::new();
        recorder
            .record(TimingStream::Imu, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0)
            .unwrap();
        assert!(!recorder.is_recording().unwrap());
        let header = TimingHeader {
            condition: "C2 terra/idle".into(),
            os: "Android 15".into(),
            build: "debug".into(),
            model: "Pixel".into(),
            media_time_at_start: 12.5,
            wall_time_at_start: 1_793_126_261.0,
        };
        let path = recorder
            .start(dir.to_string_lossy().into_owned(), header)
            .unwrap();
        assert!(path.ends_with("C2terraidle-2026-10-27T183741Z.csv"));
        recorder
            .record(
                TimingStream::Imu,
                1.0,
                1.25,
                f64::NAN,
                2.0,
                f64::NAN,
                f64::NAN,
            )
            .unwrap();
        recorder
            .record(
                TimingStream::Imu,
                1.01,
                1.26,
                f64::NAN,
                2.01,
                f64::NAN,
                f64::NAN,
            )
            .unwrap();
        recorder
            .record(
                TimingStream::BareAr,
                3.0,
                f64::NAN,
                f64::NAN,
                f64::NAN,
                f64::NAN,
                f64::NAN,
            )
            .unwrap();
        assert_eq!(recorder.row_counts().unwrap()["imu"], 2);
        recorder.stop().unwrap();
        let text = std::fs::read_to_string(&path).unwrap();
        assert!(text.starts_with("# terra sensor timing v1\n# condition=C2 terra/idle\n"));
        assert!(text.contains("stream,seq,t_sensor,t_os,t_queue,t_done,a,b\n"));
        assert!(text.contains("imu,0,1.000000,1.250000,,2.000000,,\n"));
        assert!(text.contains("imu,1,1.010000,"));
        assert!(text.contains("bare_ar,0,3.000000,,,,,\n"));
        assert!(!recorder.is_recording().unwrap());
        let _ = std::fs::remove_dir_all(dir);
    }
}
