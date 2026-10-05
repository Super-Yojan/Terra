//! Exposure-aligned depth frames update an independent rolling map per rover.
use crate::{
    depth_camera::{DepthCamera, DepthFrame},
    terra::Rover,
};
use bevy::prelude::*;
use terra_mapping::{CameraIntrinsics, CameraPose, LocalOccupancyMap, MapConfig};
use terra_types::{Quaternion, Vector3};
#[derive(Default)]
pub struct TerraOccupancyMapPlugin;
impl Plugin for TerraOccupancyMapPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(
            Update,
            update_maps.after(crate::depth_camera::receive_depth),
        );
    }
}
#[derive(Component)]
pub struct RoverOccupancyMap {
    pub map: LocalOccupancyMap,
    pub last_sequence: u64,
    pub last_exposure: Option<f64>,
}
/// Convert optical X-right/Y-down/Z-forward into Bevy camera X-right/Y-up/-Z-forward,
/// then Bevy world into robotics world (-Z, -X, Y).
fn camera_pose(transform: &GlobalTransform) -> CameraPose {
    let t = transform.compute_transform();
    let world_from_bevy = Quat::from_mat3(&Mat3::from_cols(
        Vec3::new(0.0, -1.0, 0.0),
        Vec3::new(0.0, 0.0, 1.0),
        Vec3::new(-1.0, 0.0, 0.0),
    ));
    let q = world_from_bevy * t.rotation * Quat::from_rotation_x(std::f32::consts::PI);
    CameraPose {
        position: Vector3 {
            x: -t.translation.z as f64,
            y: -t.translation.x as f64,
            z: t.translation.y as f64,
        },
        orientation: Quaternion {
            x: q.x as f64,
            y: q.y as f64,
            z: q.z as f64,
            w: q.w as f64,
        },
    }
}
fn update_maps(
    mut commands: Commands,
    sensors: Query<(&DepthCamera, &DepthFrame, &ChildOf)>,
    mut rovers: Query<(Entity, &GlobalTransform, Option<&mut RoverOccupancyMap>), With<Rover>>,
) {
    for (entity, transform, state) in &mut rovers {
        if state.is_none() {
            commands.entity(entity).insert(RoverOccupancyMap {
                map: LocalOccupancyMap::new(MapConfig::default()).unwrap(),
                last_sequence: 0,
                last_exposure: None,
            });
            continue;
        }
        let mut state = state.unwrap();
        for (camera, frame, parent) in &sensors {
            if parent.parent() != entity || frame.sequence <= state.last_sequence {
                continue;
            }
            let Some(exposure) = frame.exposure else {
                continue;
            };
            state.last_sequence = frame.sequence;
            if state.last_exposure.is_some_and(|t| exposure.timestamp <= t) {
                continue;
            }
            let p = transform.translation();
            if state.map.recenter(-p.z as f64, -p.x as f64).is_err() {
                continue;
            }
            let fy = frame.height as f64 / (2.0 * (camera.config.vertical_fov as f64 / 2.0).tan());
            let intrinsics = CameraIntrinsics {
                width: frame.width,
                height: frame.height,
                fx: fy,
                fy,
                cx: frame.width as f64 / 2.0 - 0.5,
                cy: frame.height as f64 / 2.0 - 0.5,
            };
            if state
                .map
                .integrate_depth(
                    exposure.timestamp,
                    intrinsics,
                    camera_pose(&exposure.camera_transform),
                    &frame.depth_metres,
                )
                .is_ok()
            {
                state.last_exposure = Some(exposure.timestamp);
            }
        }
    }
}
#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn optical_axes_and_camera_mount_convert_to_robotics_world() {
        let p = camera_pose(&GlobalTransform::from(Transform::from_xyz(2.0, 0.5, -3.0)));
        assert_eq!(
            p.position,
            Vector3 {
                x: 3.0,
                y: -2.0,
                z: 0.5
            }
        );
        let forward = p.orientation.rotate(Vector3 {
            x: 0.0,
            y: 0.0,
            z: 1.0,
        });
        assert!((forward.x - 1.0).abs() < 1e-6);
        let down = p.orientation.rotate(Vector3 {
            x: 0.0,
            y: 1.0,
            z: 0.0,
        });
        assert!((down.z + 1.0).abs() < 1e-6);
    }
    #[test]
    fn late_frame_uses_exposure_pose_and_maps_are_independent() {
        let mut app = App::new();
        app.add_plugins(MinimalPlugins)
            .add_systems(Update, update_maps);
        let rover = app
            .world_mut()
            .spawn((
                Rover,
                GlobalTransform::from(Transform::from_xyz(0.0, 0.0, -1.0)),
            ))
            .id();
        let other = app
            .world_mut()
            .spawn((Rover, GlobalTransform::default()))
            .id();
        app.world_mut().spawn((
            DepthCamera {
                config: crate::depth_camera::DepthCameraConfig {
                    resolution: UVec2::ONE,
                    ..default()
                },
                depth_texture: default(),
                preview_texture: default(),
            },
            DepthFrame {
                width: 1,
                height: 1,
                depth_metres: vec![2.0],
                sequence: 1,
                received_at: 10.0,
                exposure: Some(crate::depth_camera::DepthExposure {
                    sensor: Entity::PLACEHOLDER,
                    timestamp: 1.0,
                    camera_transform: GlobalTransform::from(Transform::from_xyz(-0.25, 0.5, -0.25)),
                }),
            },
            ChildOf(rover),
        ));
        app.update();
        app.update();
        let mapped = app.world().get::<RoverOccupancyMap>(rover).unwrap();
        assert_eq!(mapped.last_exposure, Some(1.0));
        let grid = mapped.map.snapshot();
        let col = ((2.25 - grid.origin_x) / grid.resolution).floor() as usize;
        let row = ((0.25 - grid.origin_y) / grid.resolution).floor() as usize;
        assert!(grid.occupancy[row * grid.width as usize + col] > 50);
        assert!(
            app.world()
                .get::<RoverOccupancyMap>(other)
                .unwrap()
                .map
                .snapshot()
                .occupancy
                .iter()
                .all(|v| *v == -1)
        );
    }
}
