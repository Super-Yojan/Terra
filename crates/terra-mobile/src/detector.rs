//! Rover YOLO post-processing shared by both phone apps.
//!
//! The app runs the model on its own accelerator (Core ML on iOS, LiteRT on
//! Android) and hands Rust the raw output rows, the camera intrinsics and pose,
//! and the depth image. Rust picks the image orientation, maps letterboxed
//! boxes back to the camera image, measures depth inside each box, and places
//! the detection in the Z-up world. This ports `ObjectSearchDetector`'s
//! decode and `PersonDepthGeometry` from the iOS app.
//!
//! The model is the end-to-end YOLO26 export: up to 300 rows of
//! `[x1, y1, x2, y2, score, class]` in model-input pixels, already free of
//! duplicates, so there is no NMS here.
use crate::ControllerError;
use crate::frames::yup_point_to_zup;
use std::sync::{Arc, Mutex};
use terra_types::{Quaternion, Vector3};

/// Detector identity reported to search, as on iOS.
pub const DETECTOR_VERSION: &str = "yolo26n-rover-depth-v1";
/// Boxes below this confidence never reach the object map.
pub const DETECTOR_MIN_CONFIDENCE: f32 = 0.5;
/// The debug page shows boxes down to this confidence.
pub const DETECTOR_DEBUG_MIN_CONFIDENCE: f32 = 0.1;
/// Depth samples beyond this are ignored, metres.
const MAX_DEPTH: f32 = 20.0;
/// At most one inference every this many seconds (5 Hz).
const FRAME_PERIOD: f64 = 0.2;

#[uniffi::export]
/// Detector version string for `register_search_detector`.
pub fn detector_version() -> String {
    DETECTOR_VERSION.into()
}

#[uniffi::export]
/// Class tokens the model reports, for `register_search_detector`.
pub fn detector_classes() -> Vec<String> {
    vec!["bucket".into(), "ball".into()]
}

/// One model class: the last word is the class token, earlier words the colour.
#[derive(Clone, Debug, PartialEq, Eq, uniffi::Record)]
pub struct DetectorLabel {
    /// Model class index.
    pub index: u32,
    /// `bucket` or `ball`.
    pub class_name: String,
    /// `blue`, `orange`, or `None`.
    pub color: Option<String>,
}

#[uniffi::export]
/// Parse Ultralytics `names` metadata, e.g. `{0: 'ball', 1: 'blue bucket'}`.
/// `None` uses the rover model's three classes.
pub fn detector_labels(names: Option<String>) -> Vec<DetectorLabel> {
    let names = names.unwrap_or_else(|| "{0: 'ball', 1: 'blue bucket', 2: 'orange bucket'}".into());
    let mut out = Vec::new();
    let mut rest = names.as_str();
    while let Some(quote) = rest.find('\'') {
        let head = &rest[..quote];
        let after = &rest[quote + 1..];
        let Some(end) = after.find('\'') else { break };
        let label = &after[..end];
        rest = &after[end + 1..];
        let index = head
            .trim_end()
            .strip_suffix(':')
            .map(str::trim_end)
            .and_then(|h| {
                let digits = h.len() - h.chars().rev().take_while(char::is_ascii_digit).count();
                h[digits..].parse::<u32>().ok()
            });
        let words: Vec<String> = label
            .to_lowercase()
            .split_whitespace()
            .map(str::to_owned)
            .collect();
        if let (Some(index), Some(class)) = (index, words.last()) {
            out.push(DetectorLabel {
                index,
                class_name: class.clone(),
                color: (words.len() > 1).then(|| words[..words.len() - 1].join("-")),
            });
        }
    }
    out
}

/// Rotation that makes the raw (landscape sensor) camera image upright.
#[derive(Clone, Copy, Debug, PartialEq, Eq, uniffi::Enum)]
pub enum ImageOrientation {
    /// Already upright.
    Up,
    /// Rotate right.
    Right,
    /// Upside down.
    Down,
    /// Rotate left.
    Left,
}

#[uniffi::export]
/// Orientation from world up expressed in camera axes (ARKit
/// `camera.transform.columns.0.y` and `.columns.1.y`; for ARCore, the Y
/// components of the camera pose's X and Y axes). Follows the physical mount,
/// not the UI. `None` when the camera looks almost straight up or down.
pub fn detector_image_orientation(up_x: f32, up_y: f32) -> Option<ImageOrientation> {
    if !up_x.is_finite() || !up_y.is_finite() || up_x.abs().max(up_y.abs()) <= 0.3 {
        return None;
    }
    Some(if up_y.abs() >= up_x.abs() {
        if up_y > 0.0 {
            ImageOrientation::Up
        } else {
            ImageOrientation::Down
        }
    } else if up_x < 0.0 {
        ImageOrientation::Right
    } else {
        ImageOrientation::Left
    })
}

/// Normalized upright point (bottom-left origin) to normalized raw-image point (top-left origin).
fn raw_point(x: f32, y: f32, orientation: ImageOrientation) -> (f32, f32) {
    match orientation {
        ImageOrientation::Up => (x, 1.0 - y),
        ImageOrientation::Right => (1.0 - y, 1.0 - x),
        ImageOrientation::Down => (1.0 - x, y),
        ImageOrientation::Left => (y, x),
    }
}

/// Model-input pixel of a centred letterbox to normalized, bottom-left-origin
/// coordinates of the upright image.
fn vision_point(model_x: f32, model_y: f32, input: f32, width: f32, height: f32) -> (f32, f32) {
    let scale = input / width.max(height);
    let (w, h) = (width * scale, height * scale);
    let u = (model_x - (input - w) / 2.0) / w;
    let v = (model_y - (input - h) / 2.0) / h;
    (u.clamp(0.0, 1.0), (1.0 - v).clamp(0.0, 1.0))
}

/// Median of at least five valid samples, if most samples agree with it.
fn measured_depth(values: &[f32]) -> Option<f32> {
    let mut valid: Vec<f32> = values
        .iter()
        .copied()
        .filter(|v| v.is_finite() && *v > 0.0 && *v <= MAX_DEPTH)
        .collect();
    valid.sort_by(f32::total_cmp);
    if valid.len() < 5 {
        return None;
    }
    let median = valid[valid.len() / 2];
    let agreeing: Vec<f32> = valid
        .iter()
        .copied()
        .filter(|v| (v - median).abs() <= 0.15f32.max(median * 0.1))
        .collect();
    if agreeing.len() < 5 || agreeing.len() * 2 < valid.len() {
        return None;
    }
    Some(agreeing[agreeing.len() / 2])
}

/// Camera intrinsics and pose for one frame.
#[derive(Clone, Copy, Debug, PartialEq, uniffi::Record)]
pub struct DetectorCamera {
    /// Raw camera image width, pixels.
    pub image_width: u32,
    /// Raw camera image height, pixels.
    pub image_height: u32,
    /// Focal length x, pixels of the raw image.
    pub fx: f32,
    /// Focal length y, pixels.
    pub fy: f32,
    /// Principal point x, pixels.
    pub cx: f32,
    /// Principal point y, pixels.
    pub cy: f32,
    /// Camera position in the Y-up AR world, metres.
    pub x: f32,
    /// Camera position Y.
    pub y: f32,
    /// Camera position Z.
    pub z: f32,
    /// Camera-to-world quaternion (camera X right, Y up, Z back) X.
    pub quaternion_x: f32,
    /// Quaternion Y.
    pub quaternion_y: f32,
    /// Quaternion Z.
    pub quaternion_z: f32,
    /// Quaternion W.
    pub quaternion_w: f32,
}

/// Depth image aligned with the camera image.
#[derive(Clone, Debug, PartialEq, uniffi::Record)]
pub struct DetectorDepth {
    /// Width, pixels.
    pub width: u32,
    /// Height, pixels.
    pub height: u32,
    /// Row-major metres; NaN or 0 for no return.
    pub metres: Vec<f32>,
    /// Per-pixel confidence, same size, or `None` to trust every sample.
    pub confidence: Option<Vec<u8>>,
    /// Lowest accepted confidence value (ARKit high = 2; ARCore raw 0–255).
    pub min_confidence: u8,
}

/// One placed detection.
#[derive(Clone, Debug, PartialEq, uniffi::Record)]
pub struct PlacedDetection {
    /// `bucket` or `ball`.
    pub class_name: String,
    /// Colour word, if any.
    pub color: Option<String>,
    /// Model score.
    pub confidence: f64,
    /// Z-up world X, metres.
    pub world_x: f64,
    /// Z-up world Y.
    pub world_y: f64,
    /// Z-up world Z.
    pub world_z: f64,
}

/// One raw box for the debug page.
#[derive(Clone, Debug, PartialEq, uniffi::Record)]
pub struct DetectorBox {
    /// Output row.
    pub row: u32,
    /// Left edge, normalized upright image, bottom-left origin.
    pub x: f32,
    /// Bottom edge.
    pub y: f32,
    /// Width.
    pub width: f32,
    /// Height.
    pub height: f32,
    /// Class token.
    pub class_name: String,
    /// Colour word.
    pub color: Option<String>,
    /// Model score.
    pub confidence: f32,
    /// Measured depth of the box centre, metres.
    pub depth: Option<f32>,
}

/// Result of one inference.
#[derive(Clone, Debug, PartialEq, uniffi::Record)]
pub struct DetectorResult {
    /// Detections at or above [`DETECTOR_MIN_CONFIDENCE`] with measured depth.
    pub detections: Vec<PlacedDetection>,
    /// Debug boxes, empty unless asked for.
    pub debug_boxes: Vec<DetectorBox>,
    /// Boxes dropped for an unknown class or no measured depth.
    pub rejected: u32,
}

#[uniffi::export]
/// Decode one inference. `rows` is the flattened `[N, 6]` output; `input_size`
/// the square model input in pixels. Returns `InvalidInput` for mismatched sizes.
pub fn decode_detections(
    rows: Vec<f32>,
    input_size: f32,
    labels: Vec<DetectorLabel>,
    orientation: ImageOrientation,
    camera: DetectorCamera,
    depth: DetectorDepth,
    debug: bool,
) -> Result<DetectorResult, ControllerError> {
    let invalid = |message: &str| ControllerError::InvalidInput {
        message: message.into(),
    };
    if !rows.len().is_multiple_of(6) || !input_size.is_finite() || input_size <= 0.0 {
        return Err(invalid(
            "detector output must be N×6 with a positive input size",
        ));
    }
    let (dw, dh) = (depth.width as usize, depth.height as usize);
    if dw == 0 || dh == 0 || depth.metres.len() != dw * dh {
        return Err(invalid("depth image size mismatch"));
    }
    if depth
        .confidence
        .as_ref()
        .is_some_and(|c| c.len() != dw * dh)
    {
        return Err(invalid("depth confidence size mismatch"));
    }
    let (raw_w, raw_h) = (camera.image_width as f32, camera.image_height as f32);
    let (up_w, up_h) = match orientation {
        ImageOrientation::Up | ImageOrientation::Down => (raw_w, raw_h),
        _ => (raw_h, raw_w),
    };
    let world_from_camera = Quaternion {
        x: camera.quaternion_x.into(),
        y: camera.quaternion_y.into(),
        z: camera.quaternion_z.into(),
        w: camera.quaternion_w.into(),
    }
    .normalized();
    let threshold = if debug {
        DETECTOR_DEBUG_MIN_CONFIDENCE
    } else {
        DETECTOR_MIN_CONFIDENCE
    };
    let mut result = DetectorResult {
        detections: Vec::new(),
        debug_boxes: Vec::new(),
        rejected: 0,
    };
    for (row, v) in rows.chunks_exact(6).enumerate() {
        let score = v[4];
        if score.is_nan() || score < threshold || !v[5].is_finite() {
            continue;
        }
        let Some(label) = labels.iter().find(|l| l.index as f32 == v[5].round()) else {
            result.rejected += 1;
            continue;
        };
        let a = vision_point(v[0], v[1], input_size, up_w, up_h);
        let b = vision_point(v[2], v[3], input_size, up_w, up_h);
        let (bx, by) = (a.0.min(b.0), a.1.min(b.1));
        let (bw, bh) = ((b.0 - a.0).abs(), (b.1 - a.1).abs());
        // Central region of the box, away from the background at its edges.
        let mut samples = Vec::with_capacity(25);
        for j in 0..5 {
            for i in 0..5 {
                let vx = bx + bw * (0.3 + i as f32 * 0.1);
                let vy = by + bh * (0.3 + j as f32 * 0.1);
                let (rx, ry) = raw_point(vx, vy, orientation);
                let px = ((rx * dw as f32) as isize).clamp(0, dw as isize - 1) as usize;
                let py = ((ry * dh as f32) as isize).clamp(0, dh as isize - 1) as usize;
                let index = py * dw + px;
                let trusted = depth
                    .confidence
                    .as_ref()
                    .is_none_or(|c| c[index] >= depth.min_confidence);
                if trusted {
                    samples.push(depth.metres[index]);
                }
            }
        }
        let measured = measured_depth(&samples);
        if debug {
            result.debug_boxes.push(DetectorBox {
                row: row as u32,
                x: bx,
                y: by,
                width: bw,
                height: bh,
                class_name: label.class_name.clone(),
                color: label.color.clone(),
                confidence: score,
                depth: measured,
            });
        }
        if score < DETECTOR_MIN_CONFIDENCE {
            continue;
        }
        let (cx, cy) = raw_point(bx + bw / 2.0, by + bh / 2.0, orientation);
        let world = match (measured, world_from_camera) {
            (Some(d), Some(q)) => world_point(cx * raw_w, cy * raw_h, d, &camera, q),
            _ => None,
        };
        let Some(world) = world else {
            result.rejected += 1;
            continue;
        };
        result.detections.push(PlacedDetection {
            class_name: label.class_name.clone(),
            color: label.color.clone(),
            confidence: score.into(),
            world_x: world.x,
            world_y: world.y,
            world_z: world.z,
        });
    }
    Ok(result)
}

/// Back-project a raw-image pixel at `depth` into the Z-up world.
fn world_point(
    px: f32,
    py: f32,
    depth: f32,
    camera: &DetectorCamera,
    world_from_camera: Quaternion,
) -> Option<Vector3> {
    if !depth.is_finite()
        || depth <= 0.0
        || depth > MAX_DEPTH
        || !px.is_finite()
        || !py.is_finite()
        || camera.fx.is_nan()
        || camera.fx <= 0.0
        || camera.fy.is_nan()
        || camera.fy <= 0.0
    {
        return None;
    }
    let local = Vector3 {
        x: f64::from((px - camera.cx) * depth / camera.fx),
        y: f64::from(-(py - camera.cy) * depth / camera.fy),
        z: f64::from(-depth),
    };
    let ar = world_from_camera.rotate(local).plus(Vector3 {
        x: camera.x.into(),
        y: camera.y.into(),
        z: camera.z.into(),
    });
    let world = yup_point_to_zup(ar);
    (world.x.is_finite() && world.y.is_finite() && world.z.is_finite()).then_some(world)
}

/// At most one inference in flight and at most 5 Hz; returns the frame number.
#[derive(uniffi::Object)]
pub struct DetectorFrameGate {
    state: Mutex<(bool, f64, u64)>,
}

#[uniffi::export]
impl DetectorFrameGate {
    #[uniffi::constructor]
    /// Idle.
    pub fn new() -> Arc<Self> {
        Arc::new(Self {
            state: Mutex::new((false, f64::NEG_INFINITY, 0)),
        })
    }

    /// Frame number to run, or `None` while one runs or within 0.2 s of the last.
    pub fn begin(&self, timestamp: f64) -> Result<Option<u64>, ControllerError> {
        let mut s = self.state.lock().map_err(|_| ControllerError::Internal)?;
        if !timestamp.is_finite() || timestamp < 0.0 || s.0 || timestamp - s.1 < FRAME_PERIOD {
            return Ok(None);
        }
        s.0 = true;
        s.1 = timestamp;
        s.2 = s.2.wrapping_add(1);
        Ok(Some(s.2))
    }

    /// The inference finished.
    pub fn finish(&self) -> Result<(), ControllerError> {
        self.state.lock().map_err(|_| ControllerError::Internal)?.0 = false;
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn close(a: f32, b: f32) -> bool {
        (a - b).abs() < 1e-5
    }
    fn camera() -> DetectorCamera {
        DetectorCamera {
            image_width: 100,
            image_height: 80,
            fx: 100.0,
            fy: 100.0,
            cx: 50.0,
            cy: 40.0,
            x: 0.0,
            y: 0.0,
            z: 0.0,
            quaternion_x: 0.0,
            quaternion_y: 0.0,
            quaternion_z: 0.0,
            quaternion_w: 1.0,
        }
    }

    #[test]
    /// Ports PersonDepthGeometryTests: projection, orientation, letterbox, depth and gate.
    fn geometry_ports() {
        let q = Quaternion::default();
        let p = world_point(50.0, 40.0, 2.0, &camera(), q).unwrap();
        // Y-up (0, 0, -2) is Z-up (2, 0, 0): two metres ahead.
        assert!((p.x - 2.0).abs() < 1e-9 && p.y.abs() < 1e-9 && p.z.abs() < 1e-9);
        assert!(world_point(50.0, 40.0, f32::NAN, &camera(), q).is_none());
        let offset = world_point(75.0, 65.0, 2.0, &camera(), q).unwrap();
        // Y-up (0.5, -0.5, -2) → Z-up (2, -0.5, -0.5).
        assert!(
            (offset.x - 2.0).abs() < 1e-6
                && (offset.y + 0.5).abs() < 1e-6
                && (offset.z + 0.5).abs() < 1e-6
        );
        let shifted = DetectorCamera { x: 3.0, ..camera() };
        assert!((world_point(50.0, 40.0, 2.0, &shifted, q).unwrap().y + 3.0).abs() < 1e-6);
        assert_eq!(measured_depth(&[2.0, 2.0, 2.0, 2.0, 2.0, 10.0]), Some(2.0));
        assert_eq!(measured_depth(&[2.0, 2.0]), None);
        assert_eq!(measured_depth(&[1.0, 2.0, 3.0, 4.0, 5.0, 6.0]), None);
        for (o, e) in [
            (ImageOrientation::Up, (0.2, 0.3)),
            (ImageOrientation::Right, (0.3, 0.8)),
            (ImageOrientation::Down, (0.8, 0.7)),
            (ImageOrientation::Left, (0.7, 0.2)),
        ] {
            let r = raw_point(0.2, 0.7, o);
            assert!(close(r.0, e.0) && close(r.1, e.1), "{o:?}");
            assert_eq!(raw_point(0.5, 0.5, o), (0.5, 0.5));
        }
        assert_eq!(
            detector_image_orientation(-1.0, 0.0),
            Some(ImageOrientation::Right)
        );
        assert_eq!(
            detector_image_orientation(1.0, 0.1),
            Some(ImageOrientation::Left)
        );
        assert_eq!(
            detector_image_orientation(0.2, 0.9),
            Some(ImageOrientation::Up)
        );
        assert_eq!(
            detector_image_orientation(0.0, -1.0),
            Some(ImageOrientation::Down)
        );
        assert_eq!(detector_image_orientation(0.1, 0.1), None);
        let top = vision_point(0.0, 80.0, 640.0, 1920.0, 1440.0);
        assert!(close(top.0, 0.0) && close(top.1, 1.0));
        let centre = vision_point(320.0, 320.0, 640.0, 1440.0, 1920.0);
        assert!(close(centre.0, 0.5) && close(centre.1, 0.5));
        let corner = vision_point(560.0, 640.0, 640.0, 1440.0, 1920.0);
        assert!(close(corner.0, 1.0) && close(corner.1, 0.0));
        let gate = DetectorFrameGate::new();
        assert_eq!(gate.begin(1.0).unwrap(), Some(1));
        assert_eq!(gate.begin(2.0).unwrap(), None);
        gate.finish().unwrap();
        assert_eq!(gate.begin(1.1).unwrap(), None);
        assert_eq!(gate.begin(0.5).unwrap(), None);
        assert_eq!(gate.begin(f64::NAN).unwrap(), None);
        assert_eq!(gate.begin(1.3).unwrap(), Some(2));
    }

    #[test]
    /// Ultralytics names parse into class and colour.
    fn labels() {
        let l = detector_labels(None);
        assert_eq!(l.len(), 3);
        assert_eq!(
            l[1],
            DetectorLabel {
                index: 1,
                class_name: "bucket".into(),
                color: Some("blue".into())
            }
        );
        assert_eq!(l[0].color, None);
        let custom = detector_labels(Some("{0: 'Dark Blue Bucket', 12: 'ball'}".into()));
        assert_eq!(custom[0].color.as_deref(), Some("dark-blue"));
        assert_eq!(custom[1].index, 12);
    }

    #[test]
    /// A centred box over flat 3 m depth lands 3 m ahead; low scores, unknown
    /// classes and untrusted depth are handled; debug keeps near-misses.
    fn decode_places_boxes() {
        let depth = DetectorDepth {
            width: 10,
            height: 8,
            metres: vec![3.0; 80],
            confidence: None,
            min_confidence: 0,
        };
        // 100×80 image letterboxed into 64: 64×51.2 content, 6.4 px bars.
        let rows = vec![
            22.0, 20.0, 42.0, 44.0, 0.9, 1.0, // blue bucket, centred
            22.0, 20.0, 42.0, 44.0, 0.3, 0.0, // below threshold
            22.0, 20.0, 42.0, 44.0, 0.8, 7.0, // unknown class
        ];
        let labels = detector_labels(None);
        let r = decode_detections(
            rows.clone(),
            64.0,
            labels.clone(),
            ImageOrientation::Up,
            camera(),
            depth.clone(),
            false,
        )
        .unwrap();
        assert_eq!(r.detections.len(), 1);
        assert_eq!(r.rejected, 1);
        let d = &r.detections[0];
        assert_eq!(
            (d.class_name.as_str(), d.color.as_deref()),
            ("bucket", Some("blue"))
        );
        assert!((d.world_x - 3.0).abs() < 1e-3 && d.world_y.abs() < 0.05 && d.world_z.abs() < 0.05);
        let dbg = decode_detections(
            rows.clone(),
            64.0,
            labels.clone(),
            ImageOrientation::Up,
            camera(),
            depth.clone(),
            true,
        )
        .unwrap();
        assert_eq!(dbg.debug_boxes.len(), 2);
        assert_eq!(dbg.debug_boxes[1].depth, Some(3.0));
        let untrusted = DetectorDepth {
            confidence: Some(vec![1; 80]),
            min_confidence: 2,
            ..depth.clone()
        };
        let r = decode_detections(
            rows.clone(),
            64.0,
            labels.clone(),
            ImageOrientation::Up,
            camera(),
            untrusted,
            false,
        )
        .unwrap();
        assert!(r.detections.is_empty() && r.rejected == 2);
        assert!(
            decode_detections(
                vec![0.0; 5],
                64.0,
                labels.clone(),
                ImageOrientation::Up,
                camera(),
                depth.clone(),
                false
            )
            .is_err()
        );
        let short = DetectorDepth {
            metres: vec![3.0; 10],
            ..depth
        };
        assert!(
            decode_detections(
                rows,
                64.0,
                labels,
                ImageOrientation::Up,
                camera(),
                short,
                false
            )
            .is_err()
        );
    }
}
