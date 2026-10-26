//! Shared map ownership for runtime and platform frame-ingestion adapters.
use std::sync::{Arc, Mutex};
use terra_mapping::*;
/// Latest accepted depth frame, kept only while a point capture is armed.
#[derive(Default)]
struct CaptureSlot {
    armed: bool,
    latest: Option<DepthCapture>,
}
#[derive(Clone)]
pub struct MappingState(
    Arc<Mutex<LocalOccupancyMap>>,
    MapConfig,
    Arc<Mutex<CaptureSlot>>,
    /// Second, independent copy slot for the phone's on-device classification
    /// view, so it never steals frames from a dashboard capture.
    Arc<Mutex<CaptureSlot>>,
);
impl MappingState {
    pub fn new(config: MapConfig) -> Result<Self, MapError> {
        Ok(Self(
            Arc::new(Mutex::new(LocalOccupancyMap::new(config)?)),
            config,
            Default::default(),
            Default::default(),
        ))
    }
    pub fn config(&self) -> MapConfig {
        self.1
    }
    pub fn clear(&self) {
        self.0.lock().unwrap().clear();
    }
    pub fn recenter(&self, x: f64, y: f64) -> Result<(), MapError> {
        self.0
            .lock()
            .map_err(|_| MapError::InvalidFrame)?
            .recenter(x, y)
    }
    pub fn snapshot(&self) -> MapSnapshot {
        self.0.lock().unwrap().snapshot()
    }
    /// Install the mesh floor that vetoes obstacle returns; see
    /// [`LocalOccupancyMap::set_floor_surface`].
    pub fn set_floor_surface(&self, floor: Option<std::sync::Arc<FloorSurface>>) {
        if let Ok(mut map) = self.0.lock() {
            map.set_floor_surface(floor);
        }
    }
    pub fn floor_surface(&self) -> Option<std::sync::Arc<FloorSurface>> {
        self.0.lock().ok()?.floor_surface()
    }
    /// Start keeping a copy of each accepted depth frame. Copies cost a full
    /// depth image per frame, so they are made only while armed.
    pub fn arm_capture(&self) {
        if let Ok(mut slot) = self.2.lock() {
            slot.armed = true;
        }
    }
    /// Stop copying frames and drop any copy not yet taken.
    pub fn disarm_capture(&self) {
        if let Ok(mut slot) = self.2.lock() {
            *slot = CaptureSlot::default();
        }
    }
    pub fn capture_armed(&self) -> bool {
        self.2.lock().is_ok_and(|slot| slot.armed)
    }
    /// Take the newest accepted frame since the previous take, if armed.
    pub fn take_capture(&self) -> Option<DepthCapture> {
        self.2.lock().ok()?.latest.take()
    }
    /// Arm or disarm the on-device debug copy; see [`Self::arm_capture`].
    pub fn set_debug_capture(&self, armed: bool) {
        if let Ok(mut slot) = self.3.lock() {
            *slot = CaptureSlot {
                armed,
                latest: None,
            };
        }
    }
    /// Take the newest accepted frame for the debug view, if armed.
    pub fn take_debug_capture(&self) -> Option<DepthCapture> {
        self.3.lock().ok()?.latest.take()
    }
    pub fn integrate_depth(
        &self,
        t: f64,
        k: CameraIntrinsics,
        c: CameraPose,
        pixels: &[f32],
    ) -> Result<(), MapError> {
        self.integrate_depth_with_ground_plane(t, k, c, pixels, GroundPlane::default())
    }
    pub fn integrate_depth_with_ground_plane(
        &self,
        t: f64,
        k: CameraIntrinsics,
        c: CameraPose,
        pixels: &[f32],
        ground: GroundPlane,
    ) -> Result<(), MapError> {
        self.integrate(t, k, c, pixels, DepthClassifier::GroundPlane(ground))
    }
    pub fn integrate_depth_in_horizon_band(
        &self,
        t: f64,
        k: CameraIntrinsics,
        c: CameraPose,
        pixels: &[f32],
        band: HorizonBand,
    ) -> Result<(), MapError> {
        self.integrate(t, k, c, pixels, DepthClassifier::HorizonBand(band))
    }
    fn integrate(
        &self,
        t: f64,
        k: CameraIntrinsics,
        c: CameraPose,
        pixels: &[f32],
        classifier: DepthClassifier,
    ) -> Result<(), MapError> {
        let floor = {
            let mut map = self.0.lock().map_err(|_| MapError::InvalidFrame)?;
            map.integrate_classified(t, k, c, pixels, classifier)?;
            map.floor_surface()
        };
        let copy = || DepthCapture {
            timestamp: t,
            intrinsics: k,
            pose: c,
            classifier,
            config: self.1,
            depth: pixels.to_vec(),
            floor: floor.clone(),
        };
        for slot in [&self.2, &self.3] {
            if let Ok(mut slot) = slot.lock()
                && slot.armed
            {
                slot.latest = Some(copy());
            }
        }
        Ok(())
    }
}
#[cfg(test)]
mod tests {
    use super::*;
    use terra_types::{Quaternion, Vector3};

    fn frame() -> (CameraIntrinsics, CameraPose) {
        (
            CameraIntrinsics {
                width: 1,
                height: 1,
                fx: 1.0,
                fy: 1.0,
                cx: 0.0,
                cy: 0.0,
            },
            CameraPose {
                position: Vector3 {
                    z: 0.5,
                    ..Vector3::ZERO
                },
                orientation: Quaternion::from_rotation_vector(Vector3 {
                    y: std::f64::consts::FRAC_PI_2,
                    ..Vector3::ZERO
                }),
            },
        )
    }

    /// Frames are copied only while armed, only when accepted, and only the
    /// newest one is kept; disarming drops it.
    #[test]
    fn capture_keeps_latest_accepted_frame_only_while_armed() {
        let state = MappingState::new(MapConfig::default()).unwrap();
        let (k, pose) = frame();
        state.integrate_depth(0.0, k, pose, &[2.0]).unwrap();
        assert!(state.take_capture().is_none());
        state.arm_capture();
        state.integrate_depth(1.0, k, pose, &[2.0]).unwrap();
        state.integrate_depth(2.0, k, pose, &[3.0]).unwrap();
        // Rejected (stale) frames are not captured.
        assert!(state.integrate_depth(2.0, k, pose, &[4.0]).is_err());
        let capture = state.take_capture().unwrap();
        assert_eq!(capture.timestamp, 2.0);
        assert_eq!(capture.depth, vec![3.0]);
        assert_eq!(capture.label().unwrap()[0].label, PointLabel::Obstacle);
        assert!(state.take_capture().is_none());
        state.integrate_depth(3.0, k, pose, &[2.0]).unwrap();
        state.disarm_capture();
        assert!(!state.capture_armed());
        assert!(state.take_capture().is_none());
    }
}
