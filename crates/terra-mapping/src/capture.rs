//! Copies of accepted depth frames, for the phone's classification view and
//! the dashboard point capture. Both relabel a frame with the rules the grid used.
use crate::{
    BarrierSurface, CameraIntrinsics, CameraPose, DepthClassifier, FloorSurface, LabeledPoint,
    MapConfig, MapError, label_depth_with_mesh,
};
use std::sync::Arc;
use terra_types::point_capture::{CaptureClassifier, CaptureFrame, CapturePoint, PointLabel};

/// Copy of one accepted depth frame and the rules it was integrated with.
#[derive(Clone, Debug)]
pub struct DepthCapture {
    /// Exposure time, seconds.
    pub timestamp: f64,
    /// Calibration of `depth`.
    pub intrinsics: CameraIntrinsics,
    /// Camera pose at exposure.
    pub pose: CameraPose,
    /// Classifier used for this frame.
    pub classifier: DepthClassifier,
    /// Mapper limits in force.
    pub config: MapConfig,
    /// Row-major axial depth in metres.
    pub depth: Vec<f32>,
    /// Mesh floor the map consulted for this frame, if any.
    pub floor: Option<Arc<FloorSurface>>,
    /// Mesh barrier the map consulted for this frame, if any.
    pub barrier: Option<Arc<BarrierSurface>>,
}
impl DepthCapture {
    /// Label every pixel; see [`label_depth_with_mesh`].
    ///
    /// # Errors
    /// As [`label_depth_with_mesh`]; a frame the map accepted does not fail.
    pub fn label(&self) -> Result<Vec<LabeledPoint>, MapError> {
        label_depth_with_mesh(
            &self.config,
            self.intrinsics,
            self.pose,
            &self.depth,
            self.classifier,
            self.floor.as_deref(),
            self.barrier.as_deref(),
        )
    }
    /// Keep every `stride`-th pixel across and down. Pixel `(u, v)` of the result
    /// is pixel `(stride·u, stride·v)` of the source, so dividing the focal lengths
    /// and principal point by `stride` keeps every ray exactly where it was.
    /// A stride of 0 or 1 returns an unchanged copy.
    pub fn decimated(&self, stride: u32) -> Self {
        if stride <= 1 {
            return self.clone();
        }
        let k = self.intrinsics;
        let width = k.width.div_ceil(stride);
        let height = k.height.div_ceil(stride);
        let scale = f64::from(stride);
        Self {
            timestamp: self.timestamp,
            intrinsics: CameraIntrinsics {
                width,
                height,
                fx: k.fx / scale,
                fy: k.fy / scale,
                cx: k.cx / scale,
                cy: k.cy / scale,
            },
            pose: self.pose,
            classifier: self.classifier,
            config: self.config,
            depth: (0..height)
                .flat_map(|v| (0..width).map(move |u| (u, v)))
                .map(|(u, v)| self.depth[(v * stride * k.width + u * stride) as usize])
                .collect(),
            floor: self.floor.clone(),
            barrier: self.barrier.clone(),
        }
    }
    /// World height of the classifier's ground plane under the camera, metres;
    /// `None` in horizon-band mode or for a vertical plane.
    pub fn ground_z(&self) -> Option<f64> {
        let c = self.pose.position;
        match self.classifier {
            DepthClassifier::GroundPlane(g) if g.normal.z.abs() > 1e-9 => {
                Some((g.offset - g.normal.x * c.x - g.normal.y * c.y) / g.normal.z)
            }
            _ => None,
        }
    }
    /// This frame as one `pointcloud/full` record: every pixel, labelled.
    ///
    /// # Errors
    /// As [`Self::label`].
    pub fn capture_frame(
        &self,
        rover_id: u64,
        request_id: String,
        index: u16,
        count: u16,
    ) -> Result<CaptureFrame, MapError> {
        let points = self.label()?;
        let q = self.pose.orientation;
        let p = self.pose.position;
        let k = self.intrinsics;
        Ok(CaptureFrame {
            rover_id,
            request_id,
            index,
            count,
            timestamp: self.timestamp,
            width: k.width,
            height: k.height,
            camera_position: [p.x, p.y, p.z],
            camera_orientation: [q.x, q.y, q.z, q.w],
            intrinsics: [k.fx, k.fy, k.cx, k.cy],
            classifier: self.classifier.into(),
            min_obstacle_height: self.config.min_obstacle_height,
            max_obstacle_height: self.config.max_obstacle_height,
            max_range: self.config.max_range,
            points: points
                .into_iter()
                .map(|point| CapturePoint {
                    position: [
                        point.position.x as f32,
                        point.position.y as f32,
                        point.position.z as f32,
                    ],
                    label: point.label,
                })
                .collect(),
        })
    }
}

impl DepthClassifier {
    /// Wire name: `ground_plane` or `horizon_band`.
    pub fn name(&self) -> &'static str {
        match self {
            Self::GroundPlane(_) => "ground_plane",
            Self::HorizonBand(_) => "horizon_band",
        }
    }
}

impl From<DepthClassifier> for CaptureClassifier {
    fn from(classifier: DepthClassifier) -> Self {
        match classifier {
            DepthClassifier::GroundPlane(g) => Self::GroundPlane {
                normal: [g.normal.x, g.normal.y, g.normal.z],
                offset: g.offset,
            },
            DepthClassifier::HorizonBand(b) => Self::HorizonBand {
                half_angle: b.half_angle,
                max_range: b.max_range,
            },
        }
    }
}

/// Labelled returns of one frame, flattened for display.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct LabelledReturns {
    /// World x y z triples, metres, for every pixel with a return.
    pub points: Vec<f32>,
    /// One [`PointLabel`] wire byte per point.
    pub labels: Vec<u8>,
    /// Pixels per label, indexed by wire byte, including no-return pixels.
    pub counts: Vec<u32>,
    /// 10th-percentile height of the returns, metres: the floor when the
    /// camera sees it.
    pub low_return_z: Option<f64>,
}
impl LabelledReturns {
    /// Flatten labelled pixels, dropping the no-return ones from `points`.
    pub fn new(labelled: impl IntoIterator<Item = LabeledPoint>) -> Self {
        let mut out = Self {
            counts: vec![0; PointLabel::ALL.len()],
            ..Self::default()
        };
        for p in labelled {
            out.counts[p.label as usize] += 1;
            if p.label != PointLabel::NoReturn {
                out.points.extend([
                    p.position.x as f32,
                    p.position.y as f32,
                    p.position.z as f32,
                ]);
                out.labels.push(p.label as u8);
            }
        }
        let mut heights: Vec<f32> = out.points.chunks_exact(3).map(|p| p[2]).collect();
        out.low_return_z = (!heights.is_empty()).then(|| {
            let at = heights.len() / 10;
            *heights.select_nth_unstable_by(at, f32::total_cmp).1 as f64
        });
        out
    }
}
