//! Publishes to the fleet's Zenoh router, connecting the same way as the
//! dashboard scripts (client mode, no multicast scouting, optional router
//! login).
//!
//! All network work happens on one background thread. The tick only copies
//! the points into a one-slot channel; if the previous frame has not been
//! sent yet, the new one is dropped, so a slow or absent network never delays
//! the control loop. If the router is unreachable, Zenoh keeps retrying in
//! the background and frames are dropped until it connects.
use std::error::Error;
use std::sync::mpsc::{self, Receiver, SyncSender};
use std::thread;

use ::zenoh::{Config, Wait, qos::CongestionControl};
use serde::Serialize;

use super::{BaseStationLink, BaseStationMsg};
use crate::geometry::Vec3;

/// Where and as whom to connect. See `[comms]` in `config.rs`.
pub struct ZenohSettings {
    /// Zenoh router endpoint, e.g. `tls/host:7448` or `tcp/192.168.1.10:7447`.
    pub endpoint: String,
    /// One key-expression chunk: no `/`, `*`, `$`, `?` or `#`.
    pub rover_id: String,
    /// Router login; `None` for a router without one.
    pub user: Option<String>,
    pub password: Option<String>,
}

pub struct ZenohLink {
    frames: SyncSender<DepthFrame>,
}

struct DepthFrame {
    t: f64,
    points: Vec<[f64; 3]>,
}

#[derive(Serialize)]
struct DepthMsg<'a> {
    rover_id: &'a str,
    t: f64,
    frame: &'static str,
    points: &'a [[f64; 3]],
}

type BoxError = Box<dyn Error + Send + Sync>;

impl ZenohLink {
    /// Start the background thread and return immediately; connecting
    /// happens on that thread.
    pub fn new(settings: ZenohSettings) -> Self {
        let (frames, rx) = mpsc::sync_channel(1);
        thread::Builder::new()
            .name("terra-av-zenoh".into())
            .spawn(move || {
                if let Err(e) = run(&settings, rx) {
                    eprintln!("terra-av: zenoh link stopped: {e}");
                }
            })
            .expect("spawn zenoh thread");
        Self { frames }
    }
}

impl BaseStationLink for ZenohLink {
    fn publish_depth(&mut self, t: f64, points: &[Vec3]) {
        let points = points.iter().map(|p| [p.x, p.y, p.z]).collect();
        // Full: the last frame is still going out, drop this one.
        // Disconnected: the thread has stopped and already logged why.
        let _ = self.frames.try_send(DepthFrame { t, points });
    }

    fn poll(&mut self) -> Option<BaseStationMsg> {
        None
    }
}

/// Body of the background thread. Returns when the link is dropped.
fn run(s: &ZenohSettings, frames: Receiver<DepthFrame>) -> Result<(), BoxError> {
    let mut config = Config::default();
    config.insert_json5("mode", r#""client""#)?;
    config.insert_json5("connect/endpoints", &serde_json::to_string(&[&s.endpoint])?)?;
    // Keep retrying instead of failing when the router is down at startup.
    config.insert_json5("connect/exit_on_failure", "false")?;
    config.insert_json5("scouting/multicast/enabled", "false")?;
    if let Some(user) = &s.user {
        config.insert_json5("transport/auth/usrpwd/user", &serde_json::to_string(user)?)?;
        let password = s.password.as_deref().unwrap_or("");
        config.insert_json5("transport/auth/usrpwd/password", &serde_json::to_string(password)?)?;
    }

    let session = ::zenoh::open(config).wait()?;
    let publisher = session
        .declare_publisher(format!("terra/{}/depth", s.rover_id))
        .congestion_control(CongestionControl::Drop)
        .wait()?;

    for frame in frames {
        let payload = serde_json::to_vec(&DepthMsg {
            rover_id: &s.rover_id,
            t: frame.t,
            frame: "camera",
            points: &frame.points,
        })?;
        if let Err(e) = publisher.put(payload).wait() {
            eprintln!("terra-av: depth publish failed: {e}");
        }
    }
    Ok(())
}
