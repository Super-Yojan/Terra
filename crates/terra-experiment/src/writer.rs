//! Bounded background JSONL writer for experiment evidence.
//! Queue or file failures are exposed to the caller instead of blocking control.

use serde_json::Value;
use std::{
    fs::File,
    io::{self, BufWriter, Write},
    path::Path,
    sync::{
        Arc,
        atomic::{AtomicBool, AtomicU64, Ordering},
        mpsc::{self, SyncSender, TrySendError},
    },
    thread::{self, JoinHandle},
    time::Duration,
};
enum Message {
    Event(Value),
    Close,
}
/// Bounded asynchronous JSONL recorder whose failures mark the run incomplete.
pub struct RunRecorder {
    sender: SyncSender<Message>,
    worker: Option<JoinHandle<io::Result<()>>>,
    failed: Arc<AtomicBool>,
    dropped: Arc<AtomicU64>,
}
impl RunRecorder {
    /// Create or truncate a JSONL file and spawn its bounded recording worker.
    ///
    /// `manifest` must be a JSON object; schema and kind are overwritten. Capacity must be 1–65536.
    /// Returns file-creation or invalid-capacity I/O errors; later worker errors surface through `record`, `failed`, or `close`.
    pub fn create(path: &Path, mut manifest: Value, capacity: usize) -> io::Result<Self> {
        if !(1..=65536).contains(&capacity) {
            return Err(io::Error::new(
                io::ErrorKind::InvalidInput,
                "invalid recording capacity",
            ));
        }
        let file = File::create(path)?;
        manifest["schema_version"] = 1.into();
        manifest["kind"] = "manifest".into();
        let (sender, receiver) = mpsc::sync_channel(capacity);
        let failed = Arc::new(AtomicBool::new(false));
        let state = failed.clone();
        let dropped = Arc::new(AtomicU64::new(0));
        let drops = dropped.clone();
        let worker = thread::spawn(move || {
            let result = (|| {
                let mut w = BufWriter::new(file);
                line(&mut w, &manifest)?;
                w.flush()?;
                loop {
                    match receiver.recv_timeout(Duration::from_secs(1)) {
                        Ok(Message::Event(v)) => line(&mut w, &v)?,
                        Ok(Message::Close) | Err(mpsc::RecvTimeoutError::Disconnected) => {
                            line(
                                &mut w,
                                &serde_json::json!({
                                    "kind": "run_end",
                                    "complete": !state.load(Ordering::Acquire),
                                    "dropped_events": drops.load(Ordering::Acquire),
                                }),
                            )?;
                            w.flush()?;
                            return Ok(());
                        }
                        Err(mpsc::RecvTimeoutError::Timeout) => w.flush()?,
                    }
                }
            })();
            if result.is_err() {
                state.store(true, Ordering::Release);
            }
            result
        });
        Ok(Self {
            sender,
            worker: Some(worker),
            failed,
            dropped,
        })
    }
    /// Try to queue an event without waiting for disk I/O.
    ///
    /// Returns an I/O error if the queue is full, the worker stopped, or recording already failed.
    /// Any rejection marks the run incomplete and increments the dropped-event count.
    pub fn record(&self, event: Value) -> io::Result<()> {
        if self.failed() {
            self.dropped.fetch_add(1, Ordering::Relaxed);
            return Err(io::Error::other("recording incomplete"));
        }
        self.sender.try_send(Message::Event(event)).map_err(|e| {
            self.failed.store(true, Ordering::Release);
            self.dropped.fetch_add(1, Ordering::Relaxed);
            io::Error::other(match e {
                TrySendError::Full(_) => "recording queue full",
                TrySendError::Disconnected(_) => "recording worker stopped",
            })
        })
    }
    /// Return whether a worker or queue failure has made the recording incomplete.
    pub fn failed(&self) -> bool {
        self.failed.load(Ordering::Acquire)
    }
    /// Drain queued events, write the terminal record, flush, and join the worker.
    ///
    /// This may block. Returns worker I/O, panic, or incomplete-recording errors; repeated closes are safe.
    pub fn close(&mut self) -> io::Result<()> {
        if let Some(worker) = self.worker.take() {
            let _ = self.sender.send(Message::Close);
            worker
                .join()
                .map_err(|_| io::Error::other("recording worker panicked"))??;
        }
        if self.failed() {
            Err(io::Error::other("recording incomplete"))
        } else {
            Ok(())
        }
    }
}
impl Drop for RunRecorder {
    fn drop(&mut self) {
        let _ = self.close();
    }
}
fn line(w: &mut impl Write, v: &Value) -> io::Result<()> {
    serde_json::to_writer(&mut *w, v)?;
    w.write_all(b"\n")
}
/// Decode newline-terminated JSON records, ignoring an interrupted final line.
///
/// Returns the first JSON error from a complete line, including an empty line;
/// schema and record kind are not validated here.
///
/// ```
/// let records = terra_experiment::read_records(b"{\"kind\":\"manifest\"}\n{unfinished")?;
/// assert_eq!(records.len(), 1);
/// assert_eq!(records[0]["kind"], "manifest");
/// # Ok::<(), serde_json::Error>(())
/// ```
pub fn read_records(bytes: &[u8]) -> serde_json::Result<Vec<Value>> {
    let mut records = Vec::new();
    for line in bytes.split_inclusive(|b| *b == b'\n') {
        if !line.ends_with(b"\n") {
            break;
        }
        records.push(serde_json::from_slice(line)?);
    }
    Ok(records)
}
