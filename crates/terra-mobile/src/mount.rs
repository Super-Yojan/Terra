//! Phone mount calibration and the phone pose pipeline, shared by iOS and Android.
//!
//! The apps hand over raw platform samples: the AR pose in the Y-up AR world
//! (ARKit camera axes, or Android device axes from `Frame.getAndroidSensorPose`)
//! and device-axis IMU vectors. Rust applies the measured mount, removes the
//! camera offset, smooths velocity, orders frames, and collects the turn-in-place
//! offset fit. One convention for both phones: the nominal mount is upright
//! portrait with the rear camera toward rover forward, and the calibration's
//! roll, pitch and yaw rotate away from it (roll −90° is landscape with the top
//! edge toward the rover's left).
use crate::frames::{VelocityFilter, quaternion_from_matrix, yup_point_to_zup, yup_to_zup};
use crate::{ControllerError, ImuReading, VioReading};
use serde::{Deserialize, Serialize};
use std::f64::consts::FRAC_PI_2;
use std::sync::{Arc, Mutex};
use terra_types::{InputError, Quaternion, Vector3};

#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize, uniffi::Record)]
#[serde(rename_all = "camelCase")]
/// Phone rear-camera pose relative to the rover reference point. Angles are
/// degrees, offsets and height metres. The JSON form matches the iOS
/// `terra.mountCalibration.v1` store, so both apps persist the same document.
pub struct PhoneMountCalibration {
    /// Camera ahead of the reference point, metres.
    pub camera_offset_forward: f64,
    /// Camera left of the reference point, metres.
    pub camera_offset_left: f64,
    /// Camera above the floor, metres; 0 means not measured.
    pub camera_height: f64,
    /// Rotation about rover forward away from the upright mount, degrees.
    pub camera_roll_degrees: f64,
    /// Rotation about rover left, degrees.
    pub camera_pitch_degrees: f64,
    /// Rotation about rover up, degrees.
    pub camera_yaw_degrees: f64,
    /// Whether a person has checked the values on the rover.
    pub reviewed: bool,
}

#[uniffi::export]
/// All zeros, unreviewed: upright portrait mount, no offset, height not measured.
pub fn uncalibrated_mount() -> PhoneMountCalibration {
    PhoneMountCalibration {
        camera_offset_forward: 0.0,
        camera_offset_left: 0.0,
        camera_height: 0.0,
        camera_roll_degrees: 0.0,
        camera_pitch_degrees: 0.0,
        camera_yaw_degrees: 0.0,
        reviewed: false,
    }
}

#[uniffi::export]
/// Terra's landscape mount: 0.23 m camera height, rolled ±90°. `top_edge_left`
/// picks −90° (phone top edge toward the rover's left). Offsets stay zero until
/// the turn fit measures them, and the result is unreviewed.
pub fn terra_landscape_mount(top_edge_left: bool) -> PhoneMountCalibration {
    PhoneMountCalibration {
        camera_height: 0.23,
        camera_roll_degrees: if top_edge_left { -90.0 } else { 90.0 },
        ..uncalibrated_mount()
    }
}

#[uniffi::export]
/// Finite, offsets within 2 m, height in (0, 2] m, angles within ±180°.
pub fn mount_is_plausible(calibration: PhoneMountCalibration) -> bool {
    let c = calibration;
    let values = [
        c.camera_offset_forward,
        c.camera_offset_left,
        c.camera_height,
        c.camera_roll_degrees,
        c.camera_pitch_degrees,
        c.camera_yaw_degrees,
    ];
    values.iter().all(|v| v.is_finite())
        && c.camera_offset_forward.abs() <= 2.0
        && c.camera_offset_left.abs() <= 2.0
        && c.camera_height > 0.0
        && c.camera_height <= 2.0
        && c.camera_roll_degrees.abs() <= 180.0
        && c.camera_pitch_degrees.abs() <= 180.0
        && c.camera_yaw_degrees.abs() <= 180.0
}

#[uniffi::export]
/// The stored JSON document for a calibration.
pub fn mount_calibration_to_json(calibration: PhoneMountCalibration) -> String {
    serde_json::to_string(&calibration).unwrap_or_default()
}

#[uniffi::export]
/// Parse a stored document. Returns `InvalidInput` for malformed JSON.
pub fn mount_calibration_from_json(json: String) -> Result<PhoneMountCalibration, ControllerError> {
    serde_json::from_str(&json).map_err(|error| ControllerError::InvalidInput {
        message: format!("mount calibration: {error}"),
    })
}

/// Which phone edge the compass references. Core Location calls the right edge
/// `landscapeLeft` and the top edge `portrait`; Android reads the same edges
/// from the rotation vector.
#[derive(Clone, Copy, Debug, PartialEq, Eq, uniffi::Enum)]
pub enum HeadingEdge {
    /// Phone +X, the right edge.
    RightEdge,
    /// Phone +Y, the top edge.
    TopEdge,
}

impl HeadingEdge {
    fn device_axis(self) -> Vector3 {
        match self {
            Self::RightEdge => Vector3 {
                x: 1.0,
                y: 0.0,
                z: 0.0,
            },
            Self::TopEdge => Vector3 {
                x: 0.0,
                y: 1.0,
                z: 0.0,
            },
        }
    }
}

#[uniffi::export]
/// The phone edge that lies most horizontal on the rover, so a rolled
/// (landscape) mount keeps a usable compass reference. Ties keep the right edge.
pub fn mount_heading_edge(calibration: PhoneMountCalibration) -> HeadingEdge {
    let to_body = device_to_body(&calibration);
    let mut best = HeadingEdge::RightEdge;
    let mut best_horizontal = -1.0;
    for edge in [HeadingEdge::RightEdge, HeadingEdge::TopEdge] {
        let axis = to_body.rotate(edge.device_axis());
        let horizontal = axis.x.hypot(axis.y);
        if horizontal > best_horizontal + 0.001 {
            best = edge;
            best_horizontal = horizontal;
        }
    }
    best
}

#[uniffi::export]
/// Turn a compass heading of the reference edge into the rover's true heading,
/// degrees in [0, 360). `None` for an invalid compass or yaw.
pub fn rover_true_heading(compass_degrees: f64, rover_yaw: f64, reference_yaw: f64) -> Option<f64> {
    if !compass_degrees.is_finite()
        || !(0.0..360.0).contains(&compass_degrees)
        || !rover_yaw.is_finite()
        || !reference_yaw.is_finite()
    {
        return None;
    }
    let corrected = (compass_degrees - (rover_yaw - reference_yaw).to_degrees()) % 360.0;
    Some((corrected + 360.0) % 360.0)
}

/// Upright portrait: phone +X right → rover right (−Y), +Y top → up, +Z out of
/// the screen → rover backward. Same matrix as the Y-up → Z-up world change.
fn nominal_device_to_body() -> Quaternion {
    quaternion_from_matrix([[0.0, 0.0, -1.0], [-1.0, 0.0, 0.0], [0.0, 1.0, 0.0]])
}

/// ARKit camera axes (landscape sensor: +X toward the phone bottom, +Y toward the
/// phone right, +Z out of the screen) into phone device axes.
fn device_from_arkit_camera() -> Quaternion {
    Quaternion::from_rotation_vector(Vector3 {
        x: 0.0,
        y: 0.0,
        z: -FRAC_PI_2,
    })
}

fn correction(c: &PhoneMountCalibration) -> Quaternion {
    let axis = |x: f64, y: f64, z: f64| Quaternion::from_rotation_vector(Vector3 { x, y, z });
    let roll = axis(c.camera_roll_degrees.to_radians(), 0.0, 0.0);
    let pitch = axis(0.0, c.camera_pitch_degrees.to_radians(), 0.0);
    let yaw = axis(0.0, 0.0, c.camera_yaw_degrees.to_radians());
    yaw.compose(pitch).compose(roll)
}

fn device_to_body(c: &PhoneMountCalibration) -> Quaternion {
    let q = correction(c).compose(nominal_device_to_body());
    q.normalized().unwrap_or(q)
}

/// Axes of the AR pose handed to [`PhonePosePipeline::push_pose`].
#[derive(Clone, Copy, Debug, PartialEq, Eq, uniffi::Enum)]
pub enum PoseAxes {
    /// ARKit `camera.transform`: landscape sensor axes.
    ArkitCamera,
    /// Phone device axes: +X right, +Y top edge, +Z out of the screen
    /// (ARCore `Frame.getAndroidSensorPose`).
    Device,
}

#[derive(Clone, Debug, uniffi::Record)]
/// One accepted AR frame after the mount, offset and velocity filter.
/// Positions are Z-up world metres before any floor lift.
pub struct PhonePose {
    /// Ready to push into `MobileController.push_vio`. `tracked` is false until
    /// the velocity filter has two tracked samples.
    pub vio: VioReading,
    /// Whether the AR session reported normal tracking for this frame.
    pub tracked: bool,
    /// Camera (phone) position, world X.
    pub camera_x: f64,
    /// Camera position, world Y.
    pub camera_y: f64,
    /// Camera position, world Z.
    pub camera_z: f64,
    /// Rover heading from world +X toward +Y, radians.
    pub yaw: f64,
    /// World yaw of the compass reference edge, radians; `None` when that edge
    /// points within about 17° of vertical.
    pub heading_reference_yaw: Option<f64>,
}

/// Why an offset-fit collection stopped on its own.
#[derive(Clone, Copy, Debug, PartialEq, Eq, uniffi::Enum)]
pub enum OffsetFitCancel {
    /// AR tracking dropped during the turn.
    TrackingLost,
    /// The turn ran past 12,000 frames.
    TooLong,
}

/// Where the turn-in-place offset fit stands.
#[derive(Clone, Copy, Debug, PartialEq, uniffi::Enum)]
pub enum OffsetFitState {
    /// No collection running.
    Idle,
    /// Collecting frames; `samples` so far.
    Collecting { samples: u32 },
    /// Collection stopped itself; start a new pass.
    Cancelled { reason: OffsetFitCancel },
}

/// Result of finishing one turn pass.
#[derive(Clone, Copy, Debug, PartialEq, uniffi::Enum)]
pub enum OffsetFitOutcome {
    /// Nothing was being collected.
    NotCollecting,
    /// Needs ≥30 samples over ≥2 s, a ≥90° turn and ≤5 cm path RMS.
    Rejected,
    /// First pass accepted; turn the other way next.
    FirstPass { rms: f64 },
    /// The reverse pass disagreed with the first by over 5 cm; both are discarded.
    Disagreed,
    /// Both passes agree. Offsets are their mean, metres.
    Accepted {
        forward: f64,
        left: f64,
        first_rms: f64,
        second_rms: f64,
    },
}

#[derive(Clone, Copy, Debug, PartialEq, uniffi::Record)]
/// One frame of a turn in place: camera position and rover heading in the world.
pub struct TurnOffsetSample {
    /// Seconds, strictly increasing.
    pub timestamp: f64,
    /// Camera world X, metres.
    pub camera_x: f64,
    /// Camera world Y, metres.
    pub camera_y: f64,
    /// Rover heading, radians.
    pub rover_heading: f64,
}

#[derive(Clone, Copy, Debug, PartialEq, uniffi::Record)]
/// Camera offset from the turning pivot, in the rover frame.
pub struct TurnOffsetFit {
    /// Camera ahead of the pivot, metres.
    pub forward: f64,
    /// Camera left of the pivot, metres.
    pub left: f64,
    /// Path residual, metres.
    pub rms_error: f64,
    /// Unwrapped turn, radians.
    pub heading_span: f64,
    /// Samples used.
    pub sample_count: u32,
    /// +1 for a left (counterclockwise) turn, −1 for right.
    pub turn_direction: i32,
}

#[uniffi::export]
/// Least-squares camera offset from a turn about a fixed pivot. `None` unless
/// there are ≥30 ordered finite samples over ≥2 s, the turn reaches 90°, the
/// offset is within 2 m and the path RMS is ≤5 cm.
pub fn fit_turn_offset(samples: Vec<TurnOffsetSample>) -> Option<TurnOffsetFit> {
    if samples.len() < 30 {
        return None;
    }
    let first = samples[0];
    let last = samples[samples.len() - 1];
    let finite = samples.iter().all(|s| {
        s.timestamp.is_finite()
            && s.camera_x.is_finite()
            && s.camera_y.is_finite()
            && s.rover_heading.is_finite()
    });
    let ordered = samples.windows(2).all(|w| w[0].timestamp < w[1].timestamp);
    if !finite || !ordered || last.timestamp - first.timestamp < 2.0 {
        return None;
    }
    let mut angles = vec![0.0; samples.len()];
    let mut accumulated = 0.0;
    for i in 1..samples.len() {
        let delta = samples[i].rover_heading - samples[i - 1].rover_heading;
        accumulated += delta.sin().atan2(delta.cos());
        angles[i] = accumulated;
    }
    let span = accumulated.abs();
    if span < FRAC_PI_2 {
        return None;
    }
    // dPosition = (R(theta) − I) · offset, in the rover frame at the start.
    let (mut aa, mut ab, mut bb, mut ax, mut bx) = (0.0, 0.0, 0.0, 0.0, 0.0);
    let mut accumulate = |a: f64, b: f64, value: f64| {
        aa += a * a;
        ab += a * b;
        bb += b * b;
        ax += a * value;
        bx += b * value;
    };
    let (c0, s0) = (first.rover_heading.cos(), first.rover_heading.sin());
    for i in 1..samples.len() {
        let (c, s) = (angles[i].cos(), angles[i].sin());
        let wx = samples[i].camera_x - first.camera_x;
        let wy = samples[i].camera_y - first.camera_y;
        let dx = c0 * wx + s0 * wy;
        let dy = -s0 * wx + c0 * wy;
        accumulate(c - 1.0, -s, dx);
        accumulate(s, c - 1.0, dy);
    }
    let determinant = aa * bb - ab * ab;
    if !determinant.is_finite() || determinant <= 1e-6 {
        return None;
    }
    let forward = (ax * bb - bx * ab) / determinant;
    let left = (bx * aa - ax * ab) / determinant;
    if !forward.is_finite() || !left.is_finite() || forward.hypot(left) > 2.0 {
        return None;
    }
    // Fixed pivot from all samples, then the trajectory residual.
    let lever = |heading: f64| {
        let (c, s) = (heading.cos(), heading.sin());
        (c * forward - s * left, s * forward + c * left)
    };
    let n = samples.len() as f64;
    let (mut cx, mut cy) = (0.0, 0.0);
    for s in &samples {
        let (lx, ly) = lever(s.rover_heading);
        cx += s.camera_x - lx;
        cy += s.camera_y - ly;
    }
    cx /= n;
    cy /= n;
    let squared: f64 = samples
        .iter()
        .map(|s| {
            let (lx, ly) = lever(s.rover_heading);
            let (ex, ey) = (s.camera_x - cx - lx, s.camera_y - cy - ly);
            ex * ex + ey * ey
        })
        .sum();
    let rms = (squared / n).sqrt();
    if !rms.is_finite() || rms > 0.05 {
        return None;
    }
    Some(TurnOffsetFit {
        forward,
        left,
        rms_error: rms,
        heading_span: span,
        sample_count: samples.len() as u32,
        turn_direction: if accumulated >= 0.0 { 1 } else { -1 },
    })
}

const MAX_FIT_SAMPLES: usize = 12_000;

struct Pipeline {
    calibration: PhoneMountCalibration,
    to_body: Quaternion,
    edge: HeadingEdge,
    axes: PoseAxes,
    session_start: f64,
    last_timestamp: Option<f64>,
    velocity: VelocityFilter,
    fit: Option<Vec<TurnOffsetSample>>,
    fit_cancelled: Option<OffsetFitCancel>,
    first_pass: Option<TurnOffsetFit>,
}

impl Pipeline {
    fn apply(&mut self, calibration: PhoneMountCalibration) {
        self.calibration = calibration;
        self.to_body = device_to_body(&calibration);
        self.edge = mount_heading_edge(calibration);
    }
}

/// Mount, offset, velocity and frame ordering for one AR session.
///
/// Feed every AR frame to [`Self::push_pose`] and every IMU sample to
/// [`Self::imu`] from the control thread. Calls are serialized by a mutex.
#[derive(uniffi::Object)]
pub struct PhonePosePipeline {
    state: Mutex<Pipeline>,
}

#[uniffi::export]
impl PhonePosePipeline {
    #[uniffi::constructor]
    /// `axes` says what [`Self::push_pose`] receives. Frames before
    /// `session_start` seconds are ignored. Any calibration is accepted here,
    /// including [`uncalibrated_mount`]; an implausible one applies no offset.
    pub fn new(
        calibration: PhoneMountCalibration,
        axes: PoseAxes,
        session_start: f64,
    ) -> Arc<Self> {
        let mut state = Pipeline {
            calibration,
            to_body: nominal_device_to_body(),
            edge: HeadingEdge::RightEdge,
            axes,
            session_start,
            last_timestamp: None,
            velocity: VelocityFilter::new(),
            fit: None,
            fit_cancelled: None,
            first_pass: None,
        };
        state.apply(calibration);
        Arc::new(Self {
            state: Mutex::new(state),
        })
    }

    /// The calibration in use.
    pub fn calibration(&self) -> Result<PhoneMountCalibration, ControllerError> {
        Ok(self.lock()?.calibration)
    }

    /// The compass edge for this calibration.
    pub fn heading_edge(&self) -> Result<HeadingEdge, ControllerError> {
        Ok(self.lock()?.edge)
    }

    /// Replace the calibration. Rejects implausible values and any change while
    /// a turn fit is collecting. Clears velocity history.
    pub fn set_calibration(
        &self,
        calibration: PhoneMountCalibration,
    ) -> Result<(), ControllerError> {
        if !mount_is_plausible(calibration) {
            return Err(ControllerError::InvalidInput {
                message: "Mount calibration values are out of range".into(),
            });
        }
        let mut state = self.lock()?;
        if state.fit.is_some() {
            return Err(ControllerError::InvalidInput {
                message: "Finish the turn pass before saving the calibration".into(),
            });
        }
        state.apply(calibration);
        state.velocity.clear();
        Ok(())
    }

    /// A new AR session: forget frame order and velocity, ignore frames before `session_start`.
    pub fn restart(&self, session_start: f64) -> Result<(), ControllerError> {
        let mut state = self.lock()?;
        state.session_start = session_start;
        state.last_timestamp = None;
        state.velocity.clear();
        Ok(())
    }

    /// One AR frame: position and orientation in the Y-up AR world, axes per
    /// [`PoseAxes`]. Returns `None` for a frame before the session start, a
    /// repeated or older timestamp, or a nonfinite time. Returns `InvalidInput`
    /// for a nonfinite position or an invalid quaternion on a tracked frame.
    #[allow(clippy::too_many_arguments)]
    pub fn push_pose(
        &self,
        timestamp: f64,
        tracked: bool,
        x: f64,
        y: f64,
        z: f64,
        quaternion_x: f64,
        quaternion_y: f64,
        quaternion_z: f64,
        quaternion_w: f64,
    ) -> Result<Option<PhonePose>, ControllerError> {
        let mut state = self.lock()?;
        let start = state.session_start;
        if !start.is_finite() || !timestamp.is_finite() || timestamp < start {
            return Ok(None);
        }
        if state.last_timestamp.is_some_and(|last| timestamp <= last) {
            return Ok(None);
        }
        let input = Quaternion {
            x: quaternion_x,
            y: quaternion_y,
            z: quaternion_z,
            w: quaternion_w,
        };
        let finite = [x, y, z].iter().all(|v| v.is_finite());
        let input = match input.normalized() {
            Some(q) if finite => q,
            _ if tracked => return Err(InputError::InvalidQuaternion.into()),
            _ => {
                state.last_timestamp = Some(timestamp);
                state.velocity.clear();
                state.cancel_fit(OffsetFitCancel::TrackingLost);
                return Ok(None);
            }
        };
        state.last_timestamp = Some(timestamp);
        let device_in_yup = match state.axes {
            PoseAxes::ArkitCamera => input.compose(device_from_arkit_camera().conjugate()),
            PoseAxes::Device => input,
        };
        let world_from_device = yup_to_zup().compose(device_in_yup);
        let body = world_from_device
            .compose(state.to_body.conjugate())
            .normalized()
            .ok_or(InputError::InvalidQuaternion)?;
        let camera = yup_point_to_zup(Vector3 { x, y, z });
        let c = state.calibration;
        let offset = if mount_is_plausible(c) {
            Vector3 {
                x: c.camera_offset_forward,
                y: c.camera_offset_left,
                z: c.camera_height,
            }
        } else {
            Vector3::ZERO
        };
        let rover = camera.plus(body.rotate(offset).scaled(-1.0));
        let forward = body.rotate(Vector3 {
            x: 1.0,
            y: 0.0,
            z: 0.0,
        });
        let yaw = forward.y.atan2(forward.x);
        if let Some(samples) = state.fit.as_mut() {
            if !tracked {
                state.cancel_fit(OffsetFitCancel::TrackingLost);
            } else {
                samples.push(TurnOffsetSample {
                    timestamp,
                    camera_x: camera.x,
                    camera_y: camera.y,
                    rover_heading: yaw,
                });
                if samples.len() > MAX_FIT_SAMPLES {
                    state.cancel_fit(OffsetFitCancel::TooLong);
                }
            }
        }
        let velocity = if tracked {
            state.velocity.push(rover, timestamp)
        } else {
            state.velocity.clear();
            None
        };
        let reference = world_from_device.rotate(state.edge.device_axis());
        let heading_reference_yaw =
            (reference.x.hypot(reference.y) >= 0.3).then(|| reference.y.atan2(reference.x));
        let v = velocity.unwrap_or(Vector3::ZERO);
        Ok(Some(PhonePose {
            vio: VioReading {
                timestamp,
                position_x: rover.x,
                position_y: rover.y,
                position_z: rover.z,
                quaternion_x: body.x,
                quaternion_y: body.y,
                quaternion_z: body.z,
                quaternion_w: body.w,
                velocity_x: v.x,
                velocity_y: v.y,
                velocity_z: v.z,
                tracked: tracked && velocity.is_some(),
            },
            tracked,
            camera_x: camera.x,
            camera_y: camera.y,
            camera_z: camera.z,
            yaw,
            heading_reference_yaw,
        }))
    }

    /// Device-axis IMU (+X right, +Y top edge, +Z out of the screen) into the
    /// rover body. Acceleration in m/s² (Core Motion callers multiply g by
    /// 9.80665), rates in rad/s. Returns `InvalidInput` for a nonfinite value.
    #[allow(clippy::too_many_arguments)]
    pub fn imu(
        &self,
        timestamp: f64,
        acceleration_x: f64,
        acceleration_y: f64,
        acceleration_z: f64,
        rate_x: f64,
        rate_y: f64,
        rate_z: f64,
    ) -> Result<ImuReading, ControllerError> {
        let values = [
            timestamp,
            acceleration_x,
            acceleration_y,
            acceleration_z,
            rate_x,
            rate_y,
            rate_z,
        ];
        if !values.iter().all(|v| v.is_finite()) {
            return Err(InputError::NonFinite.into());
        }
        let to_body = self.lock()?.to_body;
        let a = to_body.rotate(Vector3 {
            x: acceleration_x,
            y: acceleration_y,
            z: acceleration_z,
        });
        let g = to_body.rotate(Vector3 {
            x: rate_x,
            y: rate_y,
            z: rate_z,
        });
        Ok(ImuReading {
            timestamp,
            acceleration_forward: a.x,
            acceleration_left: a.y,
            acceleration_up: a.z,
            gyro_roll: g.x,
            gyro_pitch: g.y,
            gyro_yaw: g.z,
        })
    }

    /// Start collecting a turn pass. Needs a measured camera height.
    pub fn begin_offset_fit(&self) -> Result<(), ControllerError> {
        let mut state = self.lock()?;
        if state.calibration.camera_height <= 0.0 || !state.calibration.camera_height.is_finite() {
            return Err(ControllerError::InvalidInput {
                message: "Enter and save camera height before fitting".into(),
            });
        }
        if state.fit.is_none() {
            state.fit = Some(Vec::new());
            state.fit_cancelled = None;
        }
        Ok(())
    }

    /// Whether a pass is collecting, idle, or cancelled itself.
    pub fn offset_fit_state(&self) -> Result<OffsetFitState, ControllerError> {
        let state = self.lock()?;
        Ok(match (&state.fit, state.fit_cancelled) {
            (Some(samples), _) => OffsetFitState::Collecting {
                samples: samples.len() as u32,
            },
            (None, Some(reason)) => OffsetFitState::Cancelled { reason },
            (None, None) => OffsetFitState::Idle,
        })
    }

    /// Stop collecting and fit. Two passes in opposite directions within 5 cm
    /// of each other are needed for [`OffsetFitOutcome::Accepted`].
    pub fn finish_offset_fit(&self) -> Result<OffsetFitOutcome, ControllerError> {
        let mut state = self.lock()?;
        let Some(samples) = state.fit.take() else {
            return Ok(OffsetFitOutcome::NotCollecting);
        };
        let Some(fit) = fit_turn_offset(samples) else {
            return Ok(OffsetFitOutcome::Rejected);
        };
        let Some(first) = state.first_pass.take() else {
            state.first_pass = Some(fit);
            return Ok(OffsetFitOutcome::FirstPass { rms: fit.rms_error });
        };
        if first.turn_direction == fit.turn_direction
            || (first.forward - fit.forward).hypot(first.left - fit.left) > 0.05
        {
            return Ok(OffsetFitOutcome::Disagreed);
        }
        Ok(OffsetFitOutcome::Accepted {
            forward: (first.forward + fit.forward) * 0.5,
            left: (first.left + fit.left) * 0.5,
            first_rms: first.rms_error,
            second_rms: fit.rms_error,
        })
    }
}

impl Pipeline {
    fn cancel_fit(&mut self, reason: OffsetFitCancel) {
        if self.fit.take().is_some() {
            self.fit_cancelled = Some(reason);
        }
    }
}

impl PhonePosePipeline {
    fn lock(&self) -> Result<std::sync::MutexGuard<'_, Pipeline>, ControllerError> {
        self.state.lock().map_err(|_| ControllerError::Internal)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;

    fn close(a: f64, b: f64, what: &str) {
        assert!((a - b).abs() < 1e-9, "{what}: {a} != {b}");
    }
    fn close3(a: Vector3, b: Vector3, what: &str) {
        close(a.x, b.x, what);
        close(a.y, b.y, what);
        close(a.z, b.z, what);
    }
    fn v(x: f64, y: f64, z: f64) -> Vector3 {
        Vector3 { x, y, z }
    }
    fn about(axis: Vector3, angle: f64) -> Quaternion {
        Quaternion::from_rotation_vector(axis.scaled(angle))
    }
    /// Rotation from matrix columns (images of the basis), as simd builds them.
    fn from_columns(c0: Vector3, c1: Vector3, c2: Vector3) -> Quaternion {
        quaternion_from_matrix([[c0.x, c1.x, c2.x], [c0.y, c1.y, c2.y], [c0.z, c1.z, c2.z]])
    }
    /// The Swift `PhoneMountGeometry.cameraToBody` literal.
    fn swift_camera_to_body() -> Quaternion {
        from_columns(v(0.0, 0.0, -1.0), v(0.0, -1.0, 0.0), v(-1.0, 0.0, 0.0))
    }
    fn world_yup_from_zup(q: Quaternion) -> Quaternion {
        yup_to_zup().conjugate().compose(q)
    }

    #[test]
    /// Ports PhoneMountGeometryTests: upright axes, AR camera axes, and the Swift camera literal.
    fn upright_mount_axes_match_the_ios_constants() {
        let mount = nominal_device_to_body();
        close3(
            mount.rotate(v(0.0, 0.0, -1.0)),
            v(1.0, 0.0, 0.0),
            "rear camera is forward",
        );
        close3(
            mount.rotate(v(0.0, 1.0, 0.0)),
            v(0.0, 0.0, 1.0),
            "top is up",
        );
        close3(
            mount.rotate(v(1.0, 0.0, 0.0)),
            v(0.0, -1.0, 0.0),
            "right is right",
        );
        let camera = mount.compose(device_from_arkit_camera());
        let swift = swift_camera_to_body();
        for axis in [v(1.0, 0.0, 0.0), v(0.0, 1.0, 0.0), v(0.0, 0.0, 1.0)] {
            close3(camera.rotate(axis), swift.rotate(axis), "camera to body");
        }
    }

    #[test]
    /// ARKit camera input and device-axis input of one physical pose give one body pose.
    fn arkit_and_device_axes_agree() {
        let cal = PhoneMountCalibration {
            camera_roll_degrees: -90.0,
            camera_pitch_degrees: 7.0,
            camera_height: 0.23,
            camera_offset_forward: 0.1,
            ..uncalibrated_mount()
        };
        let world_body = about(v(0.0, 0.0, 1.0), 0.7).compose(about(v(0.0, 1.0, 0.0), 0.1));
        let device_yup = world_yup_from_zup(world_body.compose(device_to_body(&cal)));
        let camera_yup = device_yup.compose(device_from_arkit_camera());
        let (x, y, z) = (0.3, 0.5, -1.2);
        let a = PhonePosePipeline::new(cal, PoseAxes::Device, 0.0);
        let b = PhonePosePipeline::new(cal, PoseAxes::ArkitCamera, 0.0);
        let d = device_yup;
        let c = camera_yup;
        let pa = a
            .push_pose(1.0, true, x, y, z, d.x, d.y, d.z, d.w)
            .unwrap()
            .unwrap();
        let pb = b
            .push_pose(1.0, true, x, y, z, c.x, c.y, c.z, c.w)
            .unwrap()
            .unwrap();
        close(pa.yaw, 0.7, "yaw recovered");
        close(pa.yaw, pb.yaw, "yaw");
        close(pa.vio.position_x, pb.vio.position_x, "x");
        close(pa.vio.position_z, pb.vio.position_z, "z");
        let qa = Quaternion {
            x: pa.vio.quaternion_x,
            y: pa.vio.quaternion_y,
            z: pa.vio.quaternion_z,
            w: pa.vio.quaternion_w,
        };
        close3(
            qa.rotate(v(0.0, 0.0, 1.0)),
            world_body.rotate(v(0.0, 0.0, 1.0)),
            "body up",
        );
    }

    #[test]
    /// The iOS formula (worldFromAR · transform · arCameraToBody⁻¹, offset subtraction) matches.
    fn ios_pose_formula_parity() {
        let cal = PhoneMountCalibration {
            camera_offset_forward: 0.12,
            camera_offset_left: -0.04,
            camera_height: 0.23,
            camera_roll_degrees: 90.0,
            camera_pitch_degrees: -5.0,
            camera_yaw_degrees: 3.0,
            reviewed: true,
        };
        let transform = about(v(0.3, 1.0, 0.2).scaled(1.0 / 1.0618), 1.1)
            .normalized()
            .unwrap();
        let t = v(1.0, 0.4, -2.0);
        let ar_camera_to_body = correction(&cal).compose(swift_camera_to_body());
        let ios_body = yup_to_zup()
            .compose(transform)
            .compose(ar_camera_to_body.conjugate());
        let ios_camera = yup_point_to_zup(t);
        let ios_rover = ios_camera.plus(ios_body.rotate(v(0.12, -0.04, 0.23)).scaled(-1.0));
        let p = PhonePosePipeline::new(cal, PoseAxes::ArkitCamera, 0.0);
        let pose = p
            .push_pose(
                2.0,
                true,
                t.x,
                t.y,
                t.z,
                transform.x,
                transform.y,
                transform.z,
                transform.w,
            )
            .unwrap()
            .unwrap();
        close3(
            v(
                pose.vio.position_x,
                pose.vio.position_y,
                pose.vio.position_z,
            ),
            ios_rover,
            "rover",
        );
        let q = Quaternion {
            x: pose.vio.quaternion_x,
            y: pose.vio.quaternion_y,
            z: pose.vio.quaternion_z,
            w: pose.vio.quaternion_w,
        };
        for axis in [v(1.0, 0.0, 0.0), v(0.0, 0.0, 1.0)] {
            close3(q.rotate(axis), ios_body.rotate(axis), "body axes");
        }
    }

    #[test]
    /// Upright keeps the right edge; landscape rolls pick the horizontal top edge and read the right yaw.
    fn compass_edge_and_reference_yaw() {
        assert_eq!(
            mount_heading_edge(uncalibrated_mount()),
            HeadingEdge::RightEdge
        );
        for (top_left, top_edge_yaw) in [(true, FRAC_PI_2), (false, -FRAC_PI_2)] {
            let cal = terra_landscape_mount(top_left);
            assert_eq!(mount_heading_edge(cal), HeadingEdge::TopEdge);
            for yaw in [0.0, FRAC_PI_2, -FRAC_PI_2, PI] {
                for pitch in [0.0, 0.2, -0.2] {
                    let world_body =
                        about(v(0.0, 0.0, 1.0), yaw).compose(about(v(0.0, 1.0, 0.0), pitch));
                    let d = world_yup_from_zup(world_body.compose(device_to_body(&cal)));
                    let p = PhonePosePipeline::new(cal, PoseAxes::Device, 0.0);
                    let pose = p
                        .push_pose(1.0, true, 0.0, 0.0, 0.0, d.x, d.y, d.z, d.w)
                        .unwrap()
                        .unwrap();
                    let actual = pose.heading_reference_yaw.expect("landscape reference");
                    let expected = yaw + top_edge_yaw;
                    assert!(
                        (actual - expected)
                            .sin()
                            .atan2((actual - expected).cos())
                            .abs()
                            < 1e-9
                    );
                    let compass = (200.0 - top_edge_yaw.to_degrees() + 360.0) % 360.0;
                    let rover = rover_true_heading(compass, yaw, actual).unwrap();
                    let e = (rover - 200.0).to_radians();
                    assert!(e.sin().atan2(e.cos()).abs() < 1e-6, "rover true heading");
                }
            }
        }
        // Upright phone: the right edge is the reference, rover yaw − 90°.
        let p = PhonePosePipeline::new(uncalibrated_mount(), PoseAxes::Device, 0.0);
        let d = world_yup_from_zup(nominal_device_to_body());
        let pose = p
            .push_pose(1.0, true, 0.0, 0.0, 0.0, d.x, d.y, d.z, d.w)
            .unwrap()
            .unwrap();
        close(
            pose.heading_reference_yaw.unwrap(),
            -FRAC_PI_2,
            "upright right edge",
        );
        assert!(rover_true_heading(-1.0, 0.0, 0.0).is_none());
        for offset in [0.0, 0.4, -FRAC_PI_2, PI] {
            let actual = rover_true_heading(100.0, 1.2 + offset, 1.2).unwrap();
            close(
                actual,
                (100.0 - offset.to_degrees() + 720.0) % 360.0,
                "offset",
            );
        }
    }

    #[test]
    /// Landscape roll −90: phone right edge is up, top edge is rover left, for both IMU and pose.
    fn landscape_imu_axes() {
        let p = PhonePosePipeline::new(terra_landscape_mount(true), PoseAxes::Device, 0.0);
        let imu = p.imu(0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0).unwrap();
        close3(
            v(
                imu.acceleration_forward,
                imu.acceleration_left,
                imu.acceleration_up,
            ),
            v(0.0, 0.0, 1.0),
            "right edge up",
        );
        close3(
            v(imu.gyro_roll, imu.gyro_pitch, imu.gyro_yaw),
            v(0.0, 1.0, 0.0),
            "top edge left",
        );
        let rear = p.imu(0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0).unwrap();
        close(rear.acceleration_forward, 1.0, "rear camera forward");
        assert!(p.imu(0.0, f64::NAN, 0.0, 0.0, 0.0, 0.0, 0.0).is_err());
    }

    #[test]
    /// Ports PhoneFrameOrderTests: duplicates, late frames, invalid time and session reset.
    fn frame_order() {
        let p = PhonePosePipeline::new(uncalibrated_mount(), PoseAxes::Device, 90.0);
        let push = |t: f64| {
            p.push_pose(t, true, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0)
                .unwrap()
                .is_some()
        };
        assert!(push(100.0));
        assert!(!push(100.0));
        assert!(!push(99.0));
        assert!(push(101.0));
        assert!(!push(f64::NAN));
        assert!(!push(f64::INFINITY));
        p.restart(200.0).unwrap();
        assert!(!push(199.0));
        assert!(push(201.0));
        p.restart(40.0).unwrap();
        assert!(push(50.0));
    }

    #[test]
    /// Offset is subtracted in the body frame; velocity needs two tracked frames and drops on loss.
    fn offset_and_velocity() {
        let cal = PhoneMountCalibration {
            camera_offset_forward: 0.1,
            camera_height: 0.2,
            ..uncalibrated_mount()
        };
        let p = PhonePosePipeline::new(cal, PoseAxes::Device, 0.0);
        let body = about(v(0.0, 0.0, 1.0), FRAC_PI_2);
        let d = world_yup_from_zup(body.compose(device_to_body(&cal)));
        // Z-up camera at (1, 2, 0.5) is Y-up (-2, 0.5, -1).
        let push = |t: f64, tracked: bool, dx: f64| {
            p.push_pose(t, tracked, -2.0 - dx, 0.5, -1.0, d.x, d.y, d.z, d.w)
                .unwrap()
                .unwrap()
        };
        let first = push(0.0, true, 0.0);
        close3(
            v(
                first.vio.position_x,
                first.vio.position_y,
                first.vio.position_z,
            ),
            v(1.0, 1.9, 0.3),
            "rover",
        );
        assert!(!first.vio.tracked);
        let second = push(0.1, true, 0.1);
        assert!(second.vio.tracked);
        close(second.vio.velocity_y, 1.0 * 0.1 / 0.13, "filtered velocity");
        let lost = push(0.2, false, 0.2);
        assert!(!lost.vio.tracked);
        assert!(!push(0.3, true, 0.3).vio.tracked);
    }

    fn turn(
        pivot: (f64, f64),
        offset: (f64, f64),
        start: f64,
        span: f64,
        t0: f64,
    ) -> Vec<TurnOffsetSample> {
        (0..300)
            .map(|i| {
                let f = i as f64 / 299.0;
                let h = start + span * f;
                TurnOffsetSample {
                    timestamp: t0 + 3.0 * f,
                    camera_x: pivot.0 + h.cos() * offset.0 - h.sin() * offset.1,
                    camera_y: pivot.1 + h.sin() * offset.0 + h.cos() * offset.1,
                    rover_heading: h,
                }
            })
            .collect()
    }

    #[test]
    /// Recovers a known offset from a clean turn; rejects short, small and noisy passes.
    fn rotation_fit() {
        let fit = fit_turn_offset(turn((1.0, -2.0), (0.12, 0.03), 0.4, 2.0, 0.0)).unwrap();
        close(fit.forward, 0.12, "forward");
        close(fit.left, 0.03, "left");
        assert_eq!(fit.turn_direction, 1);
        assert!(fit_turn_offset(turn((0.0, 0.0), (0.1, 0.0), 0.0, 1.0, 0.0)).is_none());
        assert!(
            fit_turn_offset(turn((0.0, 0.0), (0.1, 0.0), 0.0, 2.0, 0.0)[..20].to_vec()).is_none()
        );
        let mut noisy = turn((0.0, 0.0), (0.1, 0.0), 0.0, 2.0, 0.0);
        for (i, s) in noisy.iter_mut().enumerate() {
            s.camera_x += if i % 2 == 0 { 0.2 } else { -0.2 };
        }
        assert!(fit_turn_offset(noisy).is_none());
    }

    fn feed(p: &PhonePosePipeline, samples: &[TurnOffsetSample], cal: &PhoneMountCalibration) {
        for s in samples {
            let body = about(v(0.0, 0.0, 1.0), s.rover_heading);
            let d = world_yup_from_zup(body.compose(device_to_body(cal)));
            // Z-up (x, y, 0) is Y-up (-y, 0, -x).
            p.push_pose(
                s.timestamp,
                true,
                -s.camera_y,
                0.0,
                -s.camera_x,
                d.x,
                d.y,
                d.z,
                d.w,
            )
            .unwrap();
        }
    }

    #[test]
    /// Two opposite passes are averaged; same-direction passes disagree; tracking loss cancels.
    fn two_way_offset_fit() {
        let cal = PhoneMountCalibration {
            camera_height: 0.23,
            ..uncalibrated_mount()
        };
        let p = PhonePosePipeline::new(cal, PoseAxes::Device, 0.0);
        assert_eq!(
            p.finish_offset_fit().unwrap(),
            OffsetFitOutcome::NotCollecting
        );
        p.begin_offset_fit().unwrap();
        feed(&p, &turn((0.5, 0.5), (0.1, 0.02), 0.0, 2.0, 1.0), &cal);
        assert!(matches!(
            p.offset_fit_state().unwrap(),
            OffsetFitState::Collecting { samples: 300 }
        ));
        assert!(matches!(
            p.finish_offset_fit().unwrap(),
            OffsetFitOutcome::FirstPass { .. }
        ));
        p.begin_offset_fit().unwrap();
        feed(&p, &turn((0.5, 0.5), (0.1, 0.02), 2.0, -2.0, 10.0), &cal);
        match p.finish_offset_fit().unwrap() {
            OffsetFitOutcome::Accepted { forward, left, .. } => {
                assert!((forward - 0.1).abs() < 1e-6 && (left - 0.02).abs() < 1e-6);
            }
            other => panic!("expected Accepted, got {other:?}"),
        }
        p.begin_offset_fit().unwrap();
        feed(&p, &turn((0.0, 0.0), (0.1, 0.0), 0.0, 2.0, 20.0), &cal);
        p.finish_offset_fit().unwrap();
        p.begin_offset_fit().unwrap();
        feed(&p, &turn((0.0, 0.0), (0.1, 0.0), 0.0, 2.0, 30.0), &cal);
        assert_eq!(p.finish_offset_fit().unwrap(), OffsetFitOutcome::Disagreed);
        p.begin_offset_fit().unwrap();
        p.push_pose(40.0, false, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0)
            .unwrap();
        assert_eq!(
            p.offset_fit_state().unwrap(),
            OffsetFitState::Cancelled {
                reason: OffsetFitCancel::TrackingLost
            }
        );
        let flat = PhonePosePipeline::new(uncalibrated_mount(), PoseAxes::Device, 0.0);
        assert!(flat.begin_offset_fit().is_err());
        assert!(p.set_calibration(uncalibrated_mount()).is_err());
    }

    #[test]
    /// The stored JSON uses the iOS Codable keys and round-trips.
    fn calibration_json_matches_ios_keys() {
        let cal = terra_landscape_mount(true);
        let json = mount_calibration_to_json(cal);
        assert!(
            json.contains("\"cameraOffsetForward\"") && json.contains("\"cameraRollDegrees\":-90")
        );
        assert_eq!(mount_calibration_from_json(json).unwrap(), cal);
        let ios = r#"{"cameraOffsetForward":0.1,"cameraOffsetLeft":0,"cameraHeight":0.23,"cameraRollDegrees":90,"cameraPitchDegrees":0,"cameraYawDegrees":0,"reviewed":true}"#;
        assert!(mount_is_plausible(
            mount_calibration_from_json(ios.into()).unwrap()
        ));
        assert!(!mount_is_plausible(uncalibrated_mount()));
    }
}
