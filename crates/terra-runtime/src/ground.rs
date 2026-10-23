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
            self.clearance = Some((camera_height - height) * up_z);
        }
        self.offset
    }
    pub fn plane(&self, camera: Vector3, up: Vector3) -> Option<GroundPlane> {
        let offset = self.offset?;
        let clearance = self.clearance?;
        if !camera.finite() || !up.finite() {
            return None;
        }
        let length = up.x.hypot(up.y).hypot(up.z);
        if length < 1e-12 || !length.is_finite() {
            return None;
        }
        let normal = Vector3 {
            x: up.x / length,
            y: up.y / length,
            z: up.z / length,
        };
        if normal.z < 0.5 {
            return None;
        }
        Some(GroundPlane {
            normal,
            offset: normal.x * camera.x + normal.y * camera.y + normal.z * (camera.z + offset)
                - clearance,
        })
    }
}
