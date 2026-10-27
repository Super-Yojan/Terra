//! On-demand full-resolution point capture shared by rover and dashboard.
//!
//! The dashboard puts a [`CaptureRequest`] on `pointcloud/request`. The rover
//! answers with one binary [`CaptureFrame`] per sampled depth image on
//! `pointcloud/full` and JSON [`CaptureProgress`] on `pointcloud/capture`.
//! Nothing is sent unless a capture was requested.
//!
//! A `live` request streams instead of recording: frames keep coming at the
//! requested rate, optionally decimated by `stride`, until the dashboard
//! cancels or stops renewing it. `duration_seconds` is then a lease that a
//! repeated request with the same id extends. Live frames carry count 0.
//!
//! # Frame layout (`TPC1`, little endian)
//!
//! | Field | Type |
//! |---|---|
//! | magic `TPC1` | 4 bytes |
//! | rover id | u64 |
//! | request id length, then UTF-8 bytes | u8, ≤ 64 bytes |
//! | frame index, frame count | u16, u16 |
//! | exposure timestamp, seconds | f64 |
//! | image width, height | u32, u32 |
//! | camera position x y z, world metres | 3 × f64 |
//! | optical-to-world quaternion x y z w | 4 × f64 |
//! | fx fy cx cy, pixels | 4 × f64 |
//! | classifier: 0 ground plane, 1 horizon band | u8 |
//! | ground normal x y z and offset, or band half-angle (rad), band range (m), 0, 0 | 4 × f64 |
//! | min obstacle height, max obstacle height, max range, metres | 3 × f64 |
//! | points: width × height, row-major from the top-left pixel | 13 bytes each |
//!
//! Each point is world x y z as f32 metres followed by a [`PointLabel`] byte.
//! A pixel without a usable return has label [`PointLabel::NoReturn`] and NaN coordinates.

use serde::{Deserialize, Serialize};

/// Dashboard-to-rover capture topic suffix.
pub const REQUEST_TOPIC: &str = "pointcloud/request";
/// Rover-to-dashboard binary frame topic suffix.
pub const FRAME_TOPIC: &str = "pointcloud/full";
/// Rover-to-dashboard progress topic suffix.
pub const PROGRESS_TOPIC: &str = "pointcloud/capture";
/// Largest accepted request payload in bytes.
pub const MAX_REQUEST_BYTES: usize = 2048;
/// Largest accepted encoded frame in bytes, enough for a 1024 × 768 depth image.
pub const MAX_FRAME_BYTES: usize = 16 * 1024 * 1024;
/// Longest accepted request id, in bytes.
pub const MAX_REQUEST_ID_BYTES: usize = 64;
/// Highest frame rate a live request may ask for.
pub const MAX_LIVE_RATE_HZ: f64 = 5.0;
/// Largest pixel stride a request may ask for.
pub const MAX_STRIDE: u32 = 8;
const MAGIC: &[u8; 4] = b"TPC1";
const POINT_BYTES: usize = 13;

/// How the mapper classified one depth pixel.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[repr(u8)]
pub enum PointLabel {
    /// Ground-level return: clears its ray and its endpoint cell.
    Free = 0,
    /// Return inside the obstacle height interval, or any return inside the horizon band.
    Obstacle = 1,
    /// Return above the maximum obstacle height; adds no evidence.
    Overhead = 2,
    /// Return beyond the mapper's range limit or the band's range; adds no evidence.
    OutOfRange = 3,
    /// Horizon-band mode only: ray steeper than the band; adds no evidence.
    OutsideBand = 4,
    /// No finite, positive depth for this pixel.
    NoReturn = 5,
    /// Would be an obstacle, but lands on surface ARKit's mesh classifies as
    /// floor: clears its ray and endpoint like [`Self::Free`].
    FloorVeto = 6,
    /// The ray passed a surface ARKit's mesh classifies as wall, window or
    /// door before its return, as through glass: the point is moved to that
    /// surface and marks it occupied.
    Barrier = 7,
}
impl PointLabel {
    /// Every label, in wire order.
    pub const ALL: [Self; 8] = [
        Self::Free,
        Self::Obstacle,
        Self::Overhead,
        Self::OutOfRange,
        Self::OutsideBand,
        Self::NoReturn,
        Self::FloorVeto,
        Self::Barrier,
    ];
    /// Decode a wire byte.
    pub fn from_u8(value: u8) -> Option<Self> {
        Self::ALL.get(value as usize).copied()
    }
    /// Stable lowercase name for files and legends.
    pub fn name(self) -> &'static str {
        match self {
            Self::Free => "free",
            Self::Obstacle => "obstacle",
            Self::Overhead => "overhead",
            Self::OutOfRange => "out_of_range",
            Self::OutsideBand => "outside_band",
            Self::NoReturn => "no_return",
            Self::FloorVeto => "floor_veto",
            Self::Barrier => "barrier",
        }
    }
}

/// Start or cancel a capture. Missing fields take the defaults shown.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct CaptureRequest {
    /// Must be 1.
    pub version: u32,
    /// Operator-chosen id echoed on every frame and progress message:
    /// 1 to 64 ASCII letters, digits, `-` or `_`.
    pub request_id: String,
    /// Capture length in seconds; defaults to 10, within (0, 60].
    #[serde(default = "default_duration")]
    pub duration_seconds: f64,
    /// Frames per second; defaults to 1, within (0, 2].
    #[serde(default = "default_rate")]
    pub rate_hz: f64,
    /// Stop the capture with this id instead of starting one.
    #[serde(default)]
    pub cancel: bool,
    /// Stream until cancelled or the lease lapses; `duration_seconds` is the
    /// lease, renewed by repeating this request with the same id.
    #[serde(default)]
    pub live: bool,
    /// Keep every `stride`-th pixel across and down; defaults to 1, within
    /// [1, 8]. Frame width, height and intrinsics describe the kept grid.
    #[serde(default = "default_stride")]
    pub stride: u32,
}
fn default_stride() -> u32 {
    1
}
fn default_duration() -> f64 {
    10.0
}
fn default_rate() -> f64 {
    1.0
}
impl CaptureRequest {
    /// A start request with the default 10 s at 1 Hz.
    pub fn new(request_id: impl Into<String>) -> Self {
        Self {
            version: 1,
            request_id: request_id.into(),
            duration_seconds: default_duration(),
            rate_hz: default_rate(),
            cancel: false,
            live: false,
            stride: default_stride(),
        }
    }
    /// A live stream request at `rate_hz` keeping every `stride`-th pixel,
    /// leased for `lease_seconds`.
    pub fn live(
        request_id: impl Into<String>,
        rate_hz: f64,
        stride: u32,
        lease_seconds: f64,
    ) -> Self {
        Self {
            duration_seconds: lease_seconds,
            rate_hz,
            live: true,
            stride,
            ..Self::new(request_id)
        }
    }
    /// Parse and validate a request payload.
    pub fn decode(bytes: &[u8]) -> Option<Self> {
        if bytes.len() > MAX_REQUEST_BYTES {
            return None;
        }
        let request: Self = serde_json::from_slice(bytes).ok()?;
        request.valid().then_some(request)
    }
    /// Whether every field is inside its documented bounds.
    pub fn valid(&self) -> bool {
        self.version == 1
            && valid_request_id(&self.request_id)
            && self.duration_seconds.is_finite()
            && self.duration_seconds > 0.0
            && self.duration_seconds <= 60.0
            && self.rate_hz.is_finite()
            && self.rate_hz > 0.0
            && self.rate_hz <= if self.live { MAX_LIVE_RATE_HZ } else { 2.0 }
            && (1..=MAX_STRIDE).contains(&self.stride)
    }
    /// Number of frames the capture aims to send, at least one; 0 for live.
    pub fn frame_count(&self) -> u16 {
        if self.live {
            return 0;
        }
        (self.duration_seconds * self.rate_hz)
            .round()
            .clamp(1.0, 120.0) as u16
    }
}
/// Whether `id` is 1 to 64 ASCII letters, digits, `-` or `_`.
pub fn valid_request_id(id: &str) -> bool {
    !id.is_empty()
        && id.len() <= MAX_REQUEST_ID_BYTES
        && id
            .bytes()
            .all(|b| b.is_ascii_alphanumeric() || b == b'-' || b == b'_')
}

/// Capture lifecycle reported on [`PROGRESS_TOPIC`].
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum CaptureState {
    /// Frames are being collected.
    Running,
    /// Every requested frame was sent, the time ran out, or a live lease lapsed.
    Done,
    /// The operator cancelled it or the dashboard link was replaced.
    Cancelled,
}
/// Progress for one capture.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct CaptureProgress {
    /// Always 1.
    pub version: u32,
    /// Rover that runs the capture.
    pub rover_id: u64,
    /// Id from the request.
    pub request_id: String,
    /// Current state.
    pub state: CaptureState,
    /// Frames handed to the network so far.
    pub frames_sent: u16,
    /// Frames the capture aims to send; 0 for a live stream.
    pub frames_total: u16,
    /// Why frames are missing, such as `no_depth` or `link_busy`; empty when none are.
    #[serde(default)]
    pub detail: String,
}

/// Mapper classification settings that produced a frame's labels.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum CaptureClassifier {
    /// Heights measured above a supporting plane `(normal · p - offset) / |normal|`.
    GroundPlane {
        /// Plane normal, world axes.
        normal: [f64; 3],
        /// Plane offset, scaled with `normal`.
        offset: f64,
    },
    /// Level band used before a floor is found.
    HorizonBand {
        /// Half-height of the band around horizontal, radians.
        half_angle: f64,
        /// Band range limit, metres.
        max_range: f64,
    },
}
/// One labelled world point.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CapturePoint {
    /// World position in metres; NaN for [`PointLabel::NoReturn`].
    pub position: [f32; 3],
    /// Mapper classification.
    pub label: PointLabel,
}
/// One full-resolution depth image, projected to world and labelled.
#[derive(Clone, Debug, PartialEq)]
pub struct CaptureFrame {
    /// Rover that captured it.
    pub rover_id: u64,
    /// Id from the request.
    pub request_id: String,
    /// Zero-based frame number within the capture.
    pub index: u16,
    /// Frames the capture aims to send; 0 for a live stream, whose index wraps.
    pub count: u16,
    /// Depth exposure time, seconds on the rover's monotonic clock.
    pub timestamp: f64,
    /// Image width in pixels.
    pub width: u32,
    /// Image height in pixels.
    pub height: u32,
    /// Camera optical origin, world metres.
    pub camera_position: [f64; 3],
    /// Optical-to-world quaternion `x y z w`.
    pub camera_orientation: [f64; 4],
    /// `fx fy cx cy` in pixels.
    pub intrinsics: [f64; 4],
    /// How heights and ranges were judged.
    pub classifier: CaptureClassifier,
    /// Minimum obstacle height above ground, metres.
    pub min_obstacle_height: f64,
    /// Maximum obstacle height above ground, metres.
    pub max_obstacle_height: f64,
    /// Mapper range limit, metres.
    pub max_range: f64,
    /// `width × height` points, row-major from the top-left pixel.
    pub points: Vec<CapturePoint>,
}
/// Why a frame could not be decoded or encoded.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FrameError {
    /// Missing `TPC1` magic.
    Magic,
    /// The buffer ended early or has trailing bytes.
    Length,
    /// A field is out of range, such as an invalid request id or point count.
    Field,
}
impl std::fmt::Display for FrameError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{self:?}")
    }
}
impl std::error::Error for FrameError {}

impl CaptureFrame {
    /// Serialize to `TPC1`.
    ///
    /// # Errors
    /// [`FrameError::Field`] for an invalid request id, a point count other
    /// than `width × height`, or a frame over [`MAX_FRAME_BYTES`].
    pub fn encode(&self) -> Result<Vec<u8>, FrameError> {
        let pixels = self.width as usize * self.height as usize;
        if !valid_request_id(&self.request_id) || self.points.len() != pixels {
            return Err(FrameError::Field);
        }
        let header = 4 + 8 + 1 + self.request_id.len() + 4 + 8 + 8 + 8 * 18 + 1;
        let size = header + pixels * POINT_BYTES;
        if size > MAX_FRAME_BYTES {
            return Err(FrameError::Field);
        }
        let mut out = Vec::with_capacity(size);
        out.extend_from_slice(MAGIC);
        out.extend_from_slice(&self.rover_id.to_le_bytes());
        out.push(self.request_id.len() as u8);
        out.extend_from_slice(self.request_id.as_bytes());
        out.extend_from_slice(&self.index.to_le_bytes());
        out.extend_from_slice(&self.count.to_le_bytes());
        out.extend_from_slice(&self.timestamp.to_le_bytes());
        out.extend_from_slice(&self.width.to_le_bytes());
        out.extend_from_slice(&self.height.to_le_bytes());
        let (kind, params) = match self.classifier {
            CaptureClassifier::GroundPlane { normal, offset } => {
                (0, [normal[0], normal[1], normal[2], offset])
            }
            CaptureClassifier::HorizonBand {
                half_angle,
                max_range,
            } => (1, [half_angle, max_range, 0.0, 0.0]),
        };
        for value in self
            .camera_position
            .iter()
            .chain(&self.camera_orientation)
            .chain(&self.intrinsics)
        {
            out.extend_from_slice(&value.to_le_bytes());
        }
        out.push(kind);
        for value in params.iter().chain(&[
            self.min_obstacle_height,
            self.max_obstacle_height,
            self.max_range,
        ]) {
            out.extend_from_slice(&value.to_le_bytes());
        }
        for point in &self.points {
            for axis in point.position {
                out.extend_from_slice(&axis.to_le_bytes());
            }
            out.push(point.label as u8);
        }
        debug_assert_eq!(out.len(), size);
        Ok(out)
    }
    /// Parse a `TPC1` frame.
    ///
    /// # Errors
    /// [`FrameError::Magic`] for another format, [`FrameError::Length`] for a
    /// truncated or padded buffer, and [`FrameError::Field`] for an invalid
    /// request id, classifier kind or label.
    pub fn decode(bytes: &[u8]) -> Result<Self, FrameError> {
        if bytes.len() > MAX_FRAME_BYTES {
            return Err(FrameError::Length);
        }
        let mut r = Reader(bytes);
        if r.take(4)? != MAGIC {
            return Err(FrameError::Magic);
        }
        let rover_id = r.u64()?;
        let id_len = r.take(1)?[0] as usize;
        let request_id = std::str::from_utf8(r.take(id_len)?)
            .map_err(|_| FrameError::Field)?
            .to_owned();
        if !valid_request_id(&request_id) {
            return Err(FrameError::Field);
        }
        let index = r.u16()?;
        let count = r.u16()?;
        let timestamp = r.f64()?;
        let width = r.u32()?;
        let height = r.u32()?;
        let mut values = [0.0; 11];
        for value in &mut values {
            *value = r.f64()?;
        }
        let kind = r.take(1)?[0];
        let mut params = [0.0; 4];
        for value in &mut params {
            *value = r.f64()?;
        }
        let classifier = match kind {
            0 => CaptureClassifier::GroundPlane {
                normal: [params[0], params[1], params[2]],
                offset: params[3],
            },
            1 => CaptureClassifier::HorizonBand {
                half_angle: params[0],
                max_range: params[1],
            },
            _ => return Err(FrameError::Field),
        };
        let (min_obstacle_height, max_obstacle_height, max_range) = (r.f64()?, r.f64()?, r.f64()?);
        let pixels = width as usize * height as usize;
        if r.0.len() != pixels * POINT_BYTES {
            return Err(FrameError::Length);
        }
        let points = r
            .0
            .chunks_exact(POINT_BYTES)
            .map(|p| {
                let axis = |i: usize| f32::from_le_bytes(p[i * 4..i * 4 + 4].try_into().unwrap());
                Ok(CapturePoint {
                    position: [axis(0), axis(1), axis(2)],
                    label: PointLabel::from_u8(p[12]).ok_or(FrameError::Field)?,
                })
            })
            .collect::<Result<_, _>>()?;
        Ok(Self {
            rover_id,
            request_id,
            index,
            count,
            timestamp,
            width,
            height,
            camera_position: [values[0], values[1], values[2]],
            camera_orientation: [values[3], values[4], values[5], values[6]],
            intrinsics: [values[7], values[8], values[9], values[10]],
            classifier,
            min_obstacle_height,
            max_obstacle_height,
            max_range,
            points,
        })
    }
}
struct Reader<'a>(&'a [u8]);
impl<'a> Reader<'a> {
    fn take(&mut self, n: usize) -> Result<&'a [u8], FrameError> {
        if self.0.len() < n {
            return Err(FrameError::Length);
        }
        let (head, tail) = self.0.split_at(n);
        self.0 = tail;
        Ok(head)
    }
    fn u16(&mut self) -> Result<u16, FrameError> {
        Ok(u16::from_le_bytes(self.take(2)?.try_into().unwrap()))
    }
    fn u32(&mut self) -> Result<u32, FrameError> {
        Ok(u32::from_le_bytes(self.take(4)?.try_into().unwrap()))
    }
    fn u64(&mut self) -> Result<u64, FrameError> {
        Ok(u64::from_le_bytes(self.take(8)?.try_into().unwrap()))
    }
    fn f64(&mut self) -> Result<f64, FrameError> {
        Ok(f64::from_le_bytes(self.take(8)?.try_into().unwrap()))
    }
}
