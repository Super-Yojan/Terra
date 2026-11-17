//! The phone's measured-floor calibration, shared by native and simulated sensors.
use terra_mapping::GroundPlane;
use terra_types::Vector3;
#[derive(Default)]
pub struct GroundPolicy {
    pub offset: Option<f64>,
    pub clearance: Option<f64>,
    candidate: Option<f64>,
    since: Option<f64>,
}
impl GroundPolicy {
    /// A simulator may supply its configured mount calibration as a synthetic measurement.
    pub fn calibrated(clearance: f64) -> Option<Self> {
        (clearance.is_finite() && (0.1..=3.0).contains(&clearance)).then_some(Self {
            offset: Some(0.0),
            clearance: Some(clearance),
            candidate: None,
            since: None,
        })
    }
    /// Take the floor from the measured mount: the camera sits `clearance` above
    /// the rover's ground point, the way a robot model's fixed camera transform
    /// does. Unlike a floor anchor seen at startup, this stays right when the
    /// phone is mounted after the app starts. `camera_height` is the camera's
    /// current world z, so the floor maps to world z = 0.
    pub fn mount(&mut self, clearance: f64, camera_height: f64) -> Option<f64> {
        if self.offset.is_some() {
            return self.offset;
        }
        if !camera_height.is_finite() {
            return None;
        }
        *self = Self::calibrated(clearance)?;
        self.offset = Some(clearance - camera_height);
        self.offset
    }
    pub fn reset(&mut self) {
        *self = Self::default();
    }
    pub fn observe(
        &mut self,
        height: Option<f64>,
        camera_height: f64,
        area: f64,
        now: f64,
        up_z: f64,
    ) -> Option<f64> {
        if self.offset.is_some() {
            return self.offset;
        }
        let Some(height) = height else {
            self.candidate = None;
            self.since = None;
            return None;
        };
        if !height.is_finite()
            || !camera_height.is_finite()
            || !now.is_finite()
            || !up_z.is_finite()
            || !(0.5..=1.000001).contains(&up_z)
            || !area.is_finite()
            || area < 0.4
            || !(0.1..=3.).contains(&(camera_height - height))
        {
            self.candidate = None;
            self.since = None;
            return None;
        }
        if self.candidate.is_none_or(|old| (old - height).abs() > 0.03) {
            self.candidate = Some(height);
            self.since = Some(now);
        }
        if self.since.is_some_and(|since| now - since >= 1.) {
            self.offset = Some(-height);
            // Vertical drop from camera to floor; `plane` measures along world up.
            self.clearance = Some(camera_height - height);
        }
        self.offset
    }
    /// Level floor `clearance` below the camera, so it follows terrain elevation.
    /// `up` is the body's up axis in world; it only rejects a tipped-over body.
    /// The normal is world up, not `up`: a mount pitch/roll error tilts body up
    /// even on flat floor, and a tilted plane turns floor a few metres out into
    /// obstacles on one side and walls into free space on the other.
    pub fn plane(&self, camera: Vector3, up: Vector3) -> Option<GroundPlane> {
        let offset = self.offset?;
        let clearance = self.clearance?;
        if !camera.finite() || !up.finite() {
            return None;
        }
        let length = up.x.hypot(up.y).hypot(up.z);
        if length < 1e-12 || !length.is_finite() || up.z / length < 0.5 {
            return None;
        }
        Some(GroundPlane {
            normal: Vector3 {
                x: 0.0,
                y: 0.0,
                z: 1.0,
            },
            offset: camera.z + offset - clearance,
        })
    }
}
