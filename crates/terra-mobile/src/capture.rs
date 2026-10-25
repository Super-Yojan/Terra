//! On-demand full point capture for debugging the mapper's classification.
//!
//! A dashboard `pointcloud/request` arms the mapper's frame copy. Each control
//! step then sends at most one labelled full-resolution frame per interval on
//! `pointcloud/full` and reports progress on `pointcloud/capture`. Nothing is
//! copied or sent while no capture runs. A capture never touches motion state.
//!
//! A live request keeps streaming, optionally decimated, until it is cancelled,
//! replaced, or its lease lapses; repeating it with the same id renews the lease.
use crate::Brain;
use terra_types::point_capture::*;

/// Seconds after the last due frame before a capture gives up waiting for depth.
const GRACE_SECONDS: f64 = 2.0;
/// Progress refresh period, inside the control plane's 500 ms state TTL.
const PROGRESS_SECONDS: f64 = 0.25;

pub(crate) struct Capture {
    request_id: String,
    total: u16,
    /// Live stream: no frame target, and the deadline is a renewable lease.
    live: bool,
    lease: f64,
    stride: u32,
    interval: f64,
    next_due: f64,
    deadline: f64,
    sent: u16,
    detail: &'static str,
    last_progress: Option<f64>,
}

/// Abandon any capture without reporting, for a replaced or lost dashboard link.
pub(crate) fn reset(brain: &mut Brain) {
    if brain.capture.take().is_some() {
        brain.runtime.mapping_handle().disarm_capture();
    }
}

/// Apply pending requests and send the frame that is due, if any.
pub(crate) fn advance(brain: &mut Brain, now: f64) {
    let Some(dashboard) = brain.dashboard.as_ref().filter(|d| !d.failed()) else {
        reset(brain);
        return;
    };
    let rover_id = dashboard.rover_id;
    let requests = dashboard.take_capture_requests();
    let grid = brain.runtime.mapping_handle();
    for request in requests
        .iter()
        .filter_map(|bytes| CaptureRequest::decode(bytes))
    {
        if request.cancel {
            if brain
                .capture
                .as_ref()
                .is_some_and(|c| c.request_id == request.request_id)
            {
                finish(brain, rover_id, CaptureState::Cancelled, "cancelled");
            }
            continue;
        }
        if let Some(running) = brain
            .capture
            .as_mut()
            .filter(|c| c.live && request.live && c.request_id == request.request_id)
        {
            running.deadline = now + running.lease;
            continue;
        }
        if brain.capture.is_some() {
            finish(brain, rover_id, CaptureState::Cancelled, "replaced");
        }
        grid.arm_capture();
        // Only frames accepted after the request belong to this capture.
        let _ = grid.take_capture();
        let total = request.frame_count();
        let interval = 1.0 / request.rate_hz;
        brain.capture = Some(Capture {
            request_id: request.request_id,
            total,
            live: request.live,
            lease: request.duration_seconds,
            stride: request.stride,
            interval,
            next_due: now,
            deadline: if request.live {
                now + request.duration_seconds
            } else {
                now + f64::from(total.saturating_sub(1)) * interval + GRACE_SECONDS
            },
            sent: 0,
            detail: "",
            last_progress: None,
        });
    }
    let Some(capture) = brain.capture.as_mut() else {
        return;
    };
    if now >= capture.next_due {
        // Wait for depth rather than skipping the slot; the deadline bounds it.
        if let Some(depth) = grid.take_capture() {
            let sent = encode(&depth, rover_id, capture).is_some_and(|bytes| {
                brain
                    .dashboard
                    .as_ref()
                    .is_some_and(|d| d.publish_bulk(FRAME_TOPIC, bytes))
            });
            if sent {
                capture.sent = capture.sent.wrapping_add(1);
            } else {
                capture.detail = "link_busy";
            }
            capture.next_due = (capture.next_due + capture.interval).max(now);
        } else if now >= capture.next_due + capture.interval {
            capture.detail = "no_depth";
        }
    }
    if now >= capture.deadline && capture.live {
        finish(brain, rover_id, CaptureState::Done, "lease_expired");
    } else if (!capture.live && capture.sent >= capture.total) || now >= capture.deadline {
        let detail = capture.detail;
        finish(brain, rover_id, CaptureState::Done, detail);
    } else if capture
        .last_progress
        .is_none_or(|t| now - t >= PROGRESS_SECONDS)
    {
        capture.last_progress = Some(now);
        let progress = progress(rover_id, capture, CaptureState::Running, capture.detail);
        if let Some(d) = brain.dashboard.as_ref() {
            d.publish(PROGRESS_TOPIC, progress);
        }
    }
}

fn finish(brain: &mut Brain, rover_id: u64, state: CaptureState, detail: &str) {
    let Some(capture) = brain.capture.take() else {
        return;
    };
    brain.runtime.mapping_handle().disarm_capture();
    if let Some(d) = brain.dashboard.as_ref() {
        d.publish(PROGRESS_TOPIC, progress(rover_id, &capture, state, detail));
    }
}

fn progress(rover_id: u64, capture: &Capture, state: CaptureState, detail: &str) -> String {
    serde_json::to_string(&CaptureProgress {
        version: 1,
        rover_id,
        request_id: capture.request_id.clone(),
        state,
        frames_sent: capture.sent,
        frames_total: capture.total,
        detail: detail.into(),
    })
    .unwrap_or_default()
}

fn encode(
    depth: &terra_mapping::DepthCapture,
    rover_id: u64,
    capture: &Capture,
) -> Option<Vec<u8>> {
    let decimated;
    let depth = if capture.stride > 1 {
        decimated = decimate(depth, capture.stride);
        &decimated
    } else {
        depth
    };
    let points = depth.label().ok()?;
    let q = depth.pose.orientation;
    let p = depth.pose.position;
    let k = depth.intrinsics;
    CaptureFrame {
        rover_id,
        request_id: capture.request_id.clone(),
        index: capture.sent,
        count: capture.total,
        timestamp: depth.timestamp,
        width: k.width,
        height: k.height,
        camera_position: [p.x, p.y, p.z],
        camera_orientation: [q.x, q.y, q.z, q.w],
        intrinsics: [k.fx, k.fy, k.cx, k.cy],
        classifier: match depth.classifier {
            terra_mapping::DepthClassifier::GroundPlane(g) => CaptureClassifier::GroundPlane {
                normal: [g.normal.x, g.normal.y, g.normal.z],
                offset: g.offset,
            },
            terra_mapping::DepthClassifier::HorizonBand(b) => CaptureClassifier::HorizonBand {
                half_angle: b.half_angle,
                max_range: b.max_range,
            },
        },
        min_obstacle_height: depth.config.min_obstacle_height,
        max_obstacle_height: depth.config.max_obstacle_height,
        max_range: depth.config.max_range,
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
    }
    .encode()
    .ok()
}

/// Keep every `stride`-th pixel across and down. Pixel `(u, v)` of the result
/// is pixel `(stride·u, stride·v)` of the source, so dividing the focal lengths
/// and principal point by `stride` keeps every ray exactly where it was.
fn decimate(depth: &terra_mapping::DepthCapture, stride: u32) -> terra_mapping::DepthCapture {
    let k = depth.intrinsics;
    let width = k.width.div_ceil(stride);
    let height = k.height.div_ceil(stride);
    let scale = f64::from(stride);
    terra_mapping::DepthCapture {
        timestamp: depth.timestamp,
        intrinsics: terra_mapping::CameraIntrinsics {
            width,
            height,
            fx: k.fx / scale,
            fy: k.fy / scale,
            cx: k.cx / scale,
            cy: k.cy / scale,
        },
        pose: depth.pose,
        classifier: depth.classifier,
        config: depth.config.clone(),
        depth: (0..height)
            .flat_map(|v| (0..width).map(move |u| (u, v)))
            .map(|(u, v)| depth.depth[(v * stride * k.width + u * stride) as usize])
            .collect(),
    }
}

/// Whether the always-on thinned cloud should go out: only during a recorded
/// capture, not a live stream, which already carries the points it needs.
pub(crate) fn thinned_cloud_wanted(brain: &Brain) -> bool {
    brain.capture.as_ref().is_some_and(|c| !c.live)
}
