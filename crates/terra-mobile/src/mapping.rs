//! UniFFI map facade; depth acquisition and exposure-aligned poses belong to the caller.
use crate::{ControllerError, MobileController};
use std::sync::{Arc, Mutex};
use terra_mapping::{
    BarrierSurface, CameraIntrinsics, CameraPose, FloorSurface, GroundPlane, HorizonBand,
    LabelledReturns, MapConfig, MapError,
};
use terra_types::{Quaternion, Vector3};
#[derive(Clone, uniffi::Record)]
/// Rolling-map dimensions and depth-integration limits. Distances are metres.
pub struct OccupancySettings {
    /// Grid width, cells.
    pub width: u32,
    /// Grid height, cells.
    pub height: u32,
    /// Grid cell size, metres per cell.
    pub resolution: f64,
    /// Minimum height above supporting ground counted as an obstacle, metres.
    pub min_obstacle_height: f64,
    /// Maximum height above supporting ground counted as an obstacle, metres.
    pub max_obstacle_height: f64,
    /// Maximum depth integration range, metres.
    pub max_range: f64,
    /// Positive sampling stride through image pixels.
    pub pixel_stride: u32,
}
#[uniffi::export]
/// Return the portable mapper defaults as a phone-facing record.
pub fn default_occupancy_settings() -> OccupancySettings {
    let c = MapConfig::default();
    OccupancySettings {
        width: c.width,
        height: c.height,
        resolution: c.resolution,
        min_obstacle_height: c.min_obstacle_height,
        max_obstacle_height: c.max_obstacle_height,
        max_range: c.max_range,
        pixel_stride: c.pixel_stride,
    }
}
#[derive(Clone, uniffi::Record)]
/// Exposure-aligned optical depth image and camera-to-world pose.
/// Optical +X points right, +Y down, +Z forward; world +Z points up.
pub struct MappingDepthFrame {
    /// Depth exposure time, seconds on a nonnegative monotonic clock.
    pub timestamp: f64,
    /// Depth image width, pixels.
    pub width: u32,
    /// Depth image height, pixels.
    pub height: u32,
    /// Horizontal focal length, pixels.
    pub fx: f64,
    /// Vertical focal length, pixels.
    pub fy: f64,
    /// Horizontal principal point in integer-pixel coordinates.
    pub cx: f64,
    /// Vertical principal point in integer-pixel coordinates.
    pub cy: f64,
    /// Camera origin world +X coordinate, metres.
    pub camera_x: f64,
    /// Camera origin world +Y coordinate, metres.
    pub camera_y: f64,
    /// Camera origin world +Z coordinate, metres.
    pub camera_z: f64,
    /// Optical-camera-to-world quaternion X component.
    pub quaternion_x: f64,
    /// Optical-camera-to-world quaternion Y component.
    pub quaternion_y: f64,
    /// Optical-camera-to-world quaternion Z component.
    pub quaternion_z: f64,
    /// Optical-camera-to-world quaternion scalar component.
    pub quaternion_w: f64,
    /// Row-major axial optical depths in metres; length must equal width × height.
    pub depth_metres: Vec<f32>,
}
/// Exposure-aligned supporting ground in the depth frame's fixed world coordinates.
/// Signed height in metres is `(normal · point - offset) / |normal|`.
#[derive(Clone, uniffi::Record)]
pub struct MappingGroundPlane {
    /// Upward ground-normal x component.
    pub normal_x: f64,
    /// Upward ground-normal y component.
    pub normal_y: f64,
    /// Upward ground-normal z component.
    pub normal_z: f64,
    /// Plane equation offset, scaled consistently with the normal.
    pub offset: f64,
}
#[derive(Clone, uniffi::Record)]
/// Owned row-major occupancy snapshot in the controller world frame.
pub struct OccupancyGrid {
    /// Grid width, cells.
    pub width: u32,
    /// Grid height, cells.
    pub height: u32,
    /// Grid cell size, metres per cell.
    pub resolution: f64,
    /// World +X coordinate of the grid lower corner, metres.
    pub origin_x: f64,
    /// World +Y coordinate of the grid lower corner, metres.
    pub origin_y: f64,
    /// Row-major cells, rows increasing in world +Y: -1 unknown, 0..=100 occupancy.
    pub occupancy: Vec<i8>,
}
/// Mesh floor near the camera, for seeding the ground policy.
#[derive(Clone, uniffi::Record)]
pub struct FloorEstimate {
    /// Median trusted floor height, world metres.
    pub height: f64,
    /// Area of the trusted floor cells counted, square metres.
    pub area: f64,
}
/// One depth frame labelled by the mapper, for the phone's classification view.
#[derive(Clone, uniffi::Record)]
pub struct ClassifiedFrame {
    /// Depth exposure time, seconds.
    pub timestamp: f64,
    /// Camera origin in world metres, x y z.
    pub camera: Vec<f64>,
    /// `ground_plane` once the floor is locked, `horizon_band` before.
    pub classifier: String,
    /// World x y z triples, metres, for every pixel with a return.
    pub points: Vec<f32>,
    /// One [`terra_types::point_capture::PointLabel`] wire byte per point.
    pub labels: Vec<u8>,
    /// Pixels per label, indexed by wire byte, including no-return pixels.
    pub counts: Vec<u32>,
    /// World x y z centres of trusted mesh-floor cells.
    pub floor_cells: Vec<f32>,
    /// World x y z centres of mesh wall, window and door cells that stop rays.
    pub barrier_cells: Vec<f32>,
    /// World height of the classifier's ground plane under the camera, metres;
    /// `None` in horizon-band mode. Obstacles start 0.15 m above it.
    pub ground_z: Option<f64>,
    /// Median mesh-floor height within 1.5 m of the camera, metres, when ARKit
    /// has classified floor there. Should match `ground_z` within a few cm.
    pub mesh_floor_z: Option<f64>,
    /// 10th-percentile height of this frame's depth returns, metres: the floor
    /// when the camera sees it. Compare with `mesh_floor_z` to spot a frame mismatch.
    pub low_return_z: Option<f64>,
}
/// Half-side of the square of floor samples kept around the camera, metres.
const FLOOR_HALF_EXTENT: f64 = 6.0;
/// Most floor sample coordinates accepted per update (100,000 points).
const MAX_FLOOR_VALUES: usize = 300_000;
fn frame_intrinsics(frame: &MappingDepthFrame) -> CameraIntrinsics {
    CameraIntrinsics {
        width: frame.width,
        height: frame.height,
        fx: frame.fx,
        fy: frame.fy,
        cx: frame.cx,
        cy: frame.cy,
    }
}
fn frame_pose(frame: &MappingDepthFrame) -> CameraPose {
    CameraPose {
        position: Vector3 {
            x: frame.camera_x,
            y: frame.camera_y,
            z: frame.camera_z,
        },
        orientation: Quaternion {
            x: frame.quaternion_x,
            y: frame.quaternion_y,
            z: frame.quaternion_z,
            w: frame.quaternion_w,
        },
    }
}
impl From<MapError> for ControllerError {
    fn from(error: MapError) -> Self {
        Self::InvalidInput {
            message: error.to_string(),
        }
    }
}
#[derive(uniffi::Object)]
/// Rolling occupancy mapper exposed through UniFFI. The mapping state locks internally.
pub struct MobileOccupancyMap {
    map: terra_runtime::mapping::MappingState,
}
#[uniffi::export]
impl MobileOccupancyMap {
    #[uniffi::constructor]
    /// Construct an unknown occupancy grid centered on the world origin.
    ///
    /// # Errors
    /// Returns `InvalidInput` for invalid dimensions, resolution, obstacle heights,
    /// range, or stride (at most one million cells; resolution 0.01..=10 m).
    pub fn new(settings: OccupancySettings) -> Result<Arc<Self>, ControllerError> {
        let map = terra_runtime::mapping::MappingState::new(MapConfig {
            width: settings.width,
            height: settings.height,
            resolution: settings.resolution,
            min_obstacle_height: settings.min_obstacle_height,
            max_obstacle_height: settings.max_obstacle_height,
            max_range: settings.max_range,
            pixel_stride: settings.pixel_stride,
        })?;
        Ok(Arc::new(Self { map }))
    }
    /// Center the rolling window near finite world coordinates `x`, `y` in metres.
    /// Shifts by whole cells, preserves overlap, and initializes new cells as unknown.
    ///
    /// # Errors
    /// Returns `InvalidInput` for invalid or excessive coordinates or a poisoned map lock.
    pub fn recenter(&self, x: f64, y: f64) -> Result<(), ControllerError> {
        self.map.recenter(x, y)?;
        Ok(())
    }
    /// Integrate one exposure-aligned optical depth frame into the rolling map.
    /// Use finite intrinsics, a valid camera pose, and strictly increasing nonnegative
    /// exposure seconds; invalid depth samples are handled by the mapper.
    ///
    /// # Errors
    /// Returns `InvalidInput` for invalid time, pose, dimensions, intrinsics, or
    /// pixel count, or a poisoned map lock.
    pub fn integrate_depth(&self, frame: MappingDepthFrame) -> Result<(), ControllerError> {
        self.integrate_depth_with_ground_plane(
            frame,
            MappingGroundPlane {
                normal_x: 0.0,
                normal_y: 0.0,
                normal_z: 1.0,
                offset: 0.0,
            },
        )
    }
    /// Integrate depth with obstacle heights relative to the supplied local terrain.
    /// World x/y cells and camera pose remain unchanged, including on slopes.
    ///
    /// # Errors
    /// Returns the same errors as [`Self::integrate_depth`], and `InvalidInput`
    /// for a nonfinite/degenerate plane or a normal over 60 degrees from world up.
    pub fn integrate_depth_with_ground_plane(
        &self,
        frame: MappingDepthFrame,
        ground: MappingGroundPlane,
    ) -> Result<(), ControllerError> {
        self.map.integrate_depth_with_ground_plane(
            frame.timestamp,
            frame_intrinsics(&frame),
            frame_pose(&frame),
            &frame.depth_metres,
            GroundPlane {
                normal: Vector3 {
                    x: ground.normal_x,
                    y: ground.normal_y,
                    z: ground.normal_z,
                },
                offset: ground.offset,
            },
        )?;
        Ok(())
    }
    /// Integrate depth without a ground estimate: only rays within
    /// `half_angle_degrees` of world horizontal count, every return among them
    /// within `max_range` metres is an obstacle, and all other rays are ignored.
    ///
    /// # Errors
    /// Returns the same errors as [`Self::integrate_depth`], and `InvalidInput`
    /// for a half-angle outside (0, 90) degrees or a nonpositive range.
    pub fn integrate_depth_in_horizon_band(
        &self,
        frame: MappingDepthFrame,
        half_angle_degrees: f64,
        max_range: f64,
    ) -> Result<(), ControllerError> {
        self.map.integrate_depth_in_horizon_band(
            frame.timestamp,
            frame_intrinsics(&frame),
            frame_pose(&frame),
            &frame.depth_metres,
            HorizonBand {
                half_angle: half_angle_degrees.to_radians(),
                max_range,
            },
        )?;
        Ok(())
    }
    /// Copy the current occupancy grid and world geometry.
    ///
    /// # Errors
    /// None today; the `Result` is part of the exported signature.
    pub fn snapshot(&self) -> Result<OccupancyGrid, ControllerError> {
        let s = self.map.snapshot();
        Ok(OccupancyGrid {
            width: s.width,
            height: s.height,
            resolution: s.resolution,
            origin_x: s.origin_x,
            origin_y: s.origin_y,
            occupancy: s.occupancy,
        })
    }
    /// Clear all occupancy evidence and the previous integration timestamp.
    ///
    /// # Errors
    /// None today; the `Result` is part of the exported signature.
    pub fn clear(&self) -> Result<(), ControllerError> {
        self.map.clear();
        Ok(())
    }
    /// Replace the mesh floor with world-metre `xyz` triples of faces ARKit
    /// classifies as floor, in the same world frame (and offset) as depth
    /// frames. Samples within 6 m of the camera and at least 0.1 m below it
    /// count; raised patches are dropped. An update with no usable sample
    /// turns the veto off. Returns the number of trusted floor cells.
    ///
    /// # Errors
    /// `InvalidInput` for a length that is not a multiple of three, more than
    /// 300,000 values, or a nonfinite camera position.
    pub fn update_floor_surface(
        &self,
        camera_x: f64,
        camera_y: f64,
        camera_z: f64,
        xyz: Vec<f32>,
    ) -> Result<u32, ControllerError> {
        if !xyz.len().is_multiple_of(3)
            || xyz.len() > MAX_FLOOR_VALUES
            || ![camera_x, camera_y, camera_z].iter().all(|v| v.is_finite())
        {
            return Err(ControllerError::InvalidInput {
                message: "floor samples must be finite xyz triples, at most 100,000 points".into(),
            });
        }
        let samples: Vec<Vector3> = xyz
            .chunks_exact(3)
            .map(|p| Vector3 {
                x: p[0] as f64,
                y: p[1] as f64,
                z: p[2] as f64,
            })
            .collect();
        // Build outside the lock: the depth update shares it on the control queue.
        let resolution = self.map.config().resolution;
        let surface = FloorSurface::build(
            &samples,
            camera_x,
            camera_y,
            camera_z,
            FLOOR_HALF_EXTENT,
            resolution,
        );
        let cells = surface.as_ref().map_or(0, |s| s.trusted_cells() as u32);
        self.map.set_floor_surface(surface.map(Arc::new));
        Ok(cells)
    }
    /// Replace the mesh barrier with world-metre `xyz` centres of faces ARKit
    /// classifies as wall, window or door, in the same world frame (and
    /// offset) as depth frames. Samples within 6 m of the camera count. Rays
    /// that pass a barrier cell stop there and mark it occupied, so glass the
    /// LiDAR sees through is not cleared. An update with no usable sample
    /// turns barriers off. Returns the number of barrier cells.
    ///
    /// # Errors
    /// As [`Self::update_floor_surface`].
    pub fn update_barrier_surface(
        &self,
        camera_x: f64,
        camera_y: f64,
        xyz: Vec<f32>,
    ) -> Result<u32, ControllerError> {
        if !xyz.len().is_multiple_of(3)
            || xyz.len() > MAX_FLOOR_VALUES
            || ![camera_x, camera_y].iter().all(|v| v.is_finite())
        {
            return Err(ControllerError::InvalidInput {
                message: "barrier samples must be finite xyz triples, at most 100,000 points"
                    .into(),
            });
        }
        let samples: Vec<Vector3> = xyz
            .chunks_exact(3)
            .map(|p| Vector3 {
                x: p[0] as f64,
                y: p[1] as f64,
                z: p[2] as f64,
            })
            .collect();
        let resolution = self.map.config().resolution;
        let surface =
            BarrierSurface::build(&samples, camera_x, camera_y, FLOOR_HALF_EXTENT, resolution);
        let cells = surface.as_ref().map_or(0, |s| s.cells() as u32);
        self.map.set_barrier_surface(surface.map(Arc::new));
        Ok(cells)
    }
    /// Keep (or stop keeping) a copy of each integrated depth frame for
    /// [`Self::take_classified_frame`]. Independent of dashboard captures.
    pub fn set_classification_debug(&self, enabled: bool) {
        self.map.set_debug_capture(enabled);
    }
    /// Label the newest integrated frame since the last call with exactly the
    /// rules the grid used, keeping every `stride`-th pixel (1..=8). `None`
    /// when debugging is off or no new frame arrived.
    pub fn take_classified_frame(&self, stride: u32) -> Option<ClassifiedFrame> {
        let capture = self.map.take_debug_capture()?.decimated(stride.min(8));
        let returns = LabelledReturns::new(capture.label().ok()?);
        let c = capture.pose.position;
        let flatten = |cells: Vec<Vector3>| {
            cells
                .into_iter()
                .flat_map(|p| [p.x as f32, p.y as f32, p.z as f32])
                .collect()
        };
        Some(ClassifiedFrame {
            timestamp: capture.timestamp,
            camera: vec![c.x, c.y, c.z],
            classifier: capture.classifier.name().into(),
            points: returns.points,
            labels: returns.labels,
            counts: returns.counts,
            floor_cells: capture
                .floor
                .as_ref()
                .map(|f| flatten(f.trusted_centres()))
                .unwrap_or_default(),
            barrier_cells: capture
                .barrier
                .as_ref()
                .map(|b| flatten(b.cell_centres()))
                .unwrap_or_default(),
            ground_z: capture.ground_z(),
            mesh_floor_z: capture
                .floor
                .as_ref()
                .and_then(|f| f.estimate(c.x, c.y, 1.5))
                .map(|(height, _)| height),
            low_return_z: returns.low_return_z,
        })
    }
    /// Median mesh floor height within `radius` metres of world `(x, y)` and
    /// the area it covers, or `None` without trusted floor there.
    pub fn floor_estimate(&self, x: f64, y: f64, radius: f64) -> Option<FloorEstimate> {
        let (height, area) = self.map.floor_surface()?.estimate(x, y, radius)?;
        Some(FloorEstimate { height, area })
    }
}

#[uniffi::export]
impl MobileController {
    pub fn attach_occupancy_map(
        &self,
        map: Arc<MobileOccupancyMap>,
    ) -> Result<(), ControllerError> {
        let handle = map.map.clone();
        self.brain
            .lock()
            .map_err(|_| ControllerError::Internal)?
            .runtime
            .attach_mapping(handle);
        Ok(())
    }
}
#[derive(uniffi::Object)]
pub struct MobileGroundPolicy {
    state: Mutex<terra_runtime::ground::GroundPolicy>,
}
#[uniffi::export]
impl MobileGroundPolicy {
    // These calls are infallible in Swift; a lock poisoned by an earlier panic
    // reads as "no ground estimate" rather than aborting the app.
    #[uniffi::constructor]
    pub fn new() -> Arc<Self> {
        Arc::new(Self {
            state: Mutex::new(Default::default()),
        })
    }
    pub fn reset(&self) {
        if let Ok(mut state) = self.state.lock() {
            state.reset();
        }
    }
    pub fn offset(&self) -> Option<f64> {
        self.state.lock().ok()?.offset
    }
    pub fn clearance(&self) -> Option<f64> {
        self.state.lock().ok()?.clearance
    }
    pub fn observe(
        &self,
        height: Option<f64>,
        camera_height: f64,
        area: f64,
        now: f64,
        up_z: f64,
    ) -> Option<f64> {
        self.state
            .lock()
            .ok()?
            .observe(height, camera_height, area, now, up_z)
    }
    pub fn mount(&self, clearance: f64, camera_height: f64) -> Option<f64> {
        self.state.lock().ok()?.mount(clearance, camera_height)
    }
    pub fn plane(
        &self,
        x: f64,
        y: f64,
        z: f64,
        up_x: f64,
        up_y: f64,
        up_z: f64,
    ) -> Option<MappingGroundPlane> {
        self.state
            .lock()
            .ok()?
            .plane(
                Vector3 { x, y, z },
                Vector3 {
                    x: up_x,
                    y: up_y,
                    z: up_z,
                },
            )
            .map(|p| MappingGroundPlane {
                normal_x: p.normal.x,
                normal_y: p.normal.y,
                normal_z: p.normal.z,
                offset: p.offset,
            })
    }
}
