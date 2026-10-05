//! Avian chassis approximation: dynamic body with yaw free and roll/pitch locked.
use avian3d::prelude::*;
use bevy::prelude::*;

pub struct TerraPhysicsPlugin;

impl Plugin for TerraPhysicsPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<RoverPhysicsConfig>()
            .add_plugins(PhysicsPlugins::default());
    }
}

/// Approximate dimensions for the supplied GLB, scaled to metres.
/// Insert before TerraPhysicsPlugin to override these defaults.
#[derive(Resource, Clone)]
pub struct RoverPhysicsConfig {
    pub chassis_size: Vec3,
    pub mass_kg: f32,
    pub model_scale: f32,
}

impl Default for RoverPhysicsConfig {
    fn default() -> Self {
        Self {
            chassis_size: Vec3::new(0.9, 0.16, 0.8),
            mass_kg: 20.0,
            model_scale: 0.025,
        }
    }
}

impl RoverPhysicsConfig {
    pub fn validate(&self) -> Result<(), &'static str> {
        if !self.chassis_size.is_finite()
            || self.chassis_size.min_element() <= 0.0
            || !self.mass_kg.is_finite()
            || self.mass_kg <= 0.0
            || !self.model_scale.is_finite()
            || self.model_scale <= 0.0
        {
            return Err("rover size, mass and visual scale must be finite and positive");
        }
        Ok(())
    }
}

#[derive(Bundle)]
pub struct RoverBody {
    body: RigidBody,
    collider: Collider,
    mass: Mass,
    locked_axes: LockedAxes,
    linear_velocity: LinearVelocity,
    angular_velocity: AngularVelocity,
    friction: Friction,
    restitution: Restitution,
    sleeping_disabled: SleepingDisabled,
    ccd: SweptCcd,
}

impl RoverBody {
    pub fn new(config: &RoverPhysicsConfig) -> Self {
        config
            .validate()
            .expect("invalid rover physics configuration");
        Self {
            body: RigidBody::Dynamic,
            collider: Collider::cuboid(
                config.chassis_size.x,
                config.chassis_size.y,
                config.chassis_size.z,
            ),
            mass: Mass(config.mass_kg),
            locked_axes: LockedAxes::new().lock_rotation_x().lock_rotation_z(),
            linear_velocity: LinearVelocity::ZERO,
            angular_velocity: AngularVelocity::ZERO,
            // Wheel forces and lateral damping model traction, rather than chassis sliding friction.
            friction: Friction::ZERO.with_combine_rule(CoefficientCombine::Multiply),
            restitution: Restitution::ZERO,
            sleeping_disabled: SleepingDisabled,
            ccd: SweptCcd::default(),
        }
    }
}

impl Default for RoverBody {
    fn default() -> Self {
        Self::new(&RoverPhysicsConfig::default())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::terra::{DriveCommand, DriveConfig, Odometry, Rover, WheelSpeeds, drive};
    use bevy::{asset::AssetPlugin, mesh::MeshPlugin, time::TimeUpdateStrategy};
    use std::time::Duration;

    fn simulation_with_world(generated: bool) -> App {
        let mut app = App::new();
        app.add_plugins((
            MinimalPlugins,
            TransformPlugin,
            AssetPlugin::default(),
            MeshPlugin,
            TerraPhysicsPlugin,
        ))
        .insert_resource(TimeUpdateStrategy::ManualDuration(Duration::from_secs_f64(
            1.0 / 60.0,
        )))
        .insert_resource(DriveConfig::default())
        .add_systems(FixedUpdate, drive.before(PhysicsSystems::StepSimulation));
        if generated {
            app.insert_resource(Assets::<StandardMaterial>::default())
                .add_plugins(crate::world::TerraWorldPlugin::default());
        }
        app.finish();
        app.cleanup();
        if !generated {
            app.world_mut().spawn((
                RigidBody::Static,
                Collider::cuboid(20.0, 0.2, 20.0),
                Transform::from_xyz(0.0, -0.1, 0.0),
            ));
        }
        app
    }

    fn simulation() -> App {
        simulation_with_world(false)
    }

    fn rover(app: &mut App, command: DriveCommand) -> Entity {
        app.world_mut()
            .spawn((
                Rover,
                RoverBody::default(),
                command,
                WheelSpeeds::default(),
                Odometry::default(),
                Transform::from_xyz(0.0, 2.0, 0.0),
            ))
            .id()
    }
    fn tick(app: &mut App, frames: usize) {
        for _ in 0..frames {
            app.update();
        }
    }

    #[test]
    fn generated_world_has_scaled_static_colliders() {
        use crate::world::{BoundaryMarker, Ground};
        let mut app = simulation_with_world(true);
        let entity = rover(&mut app, DriveCommand::default());
        app.world_mut()
            .get_mut::<Transform>(entity)
            .unwrap()
            .translation
            .x = 10.0;
        tick(&mut app, 180);
        assert!((app.world().get::<Position>(entity).unwrap().y - 0.08).abs() < 0.03);
        let world = app.world_mut();
        assert_eq!(
            world
                .query_filtered::<&Collider, With<Ground>>()
                .iter(world)
                .count(),
            1
        );
        assert_eq!(
            world
                .query_filtered::<&Collider, With<BoundaryMarker>>()
                .iter(world)
                .count(),
            4
        );
    }

    #[test]
    fn rejects_invalid_rover_body_dimensions() {
        assert!(RoverPhysicsConfig::default().validate().is_ok());
        assert!(
            RoverPhysicsConfig {
                chassis_size: Vec3::ZERO,
                ..default()
            }
            .validate()
            .is_err()
        );
        assert!(
            RoverPhysicsConfig {
                mass_kg: f32::NAN,
                ..default()
            }
            .validate()
            .is_err()
        );
    }

    #[test]
    fn gravity_settles_rover_on_ground() {
        let mut app = simulation();
        let entity = rover(&mut app, DriveCommand::default());
        tick(&mut app, 180);
        let position = app.world().get::<Position>(entity).unwrap().0;
        assert!((position.y - 0.08).abs() < 0.03, "{position:?}");
        assert!(app.world().get::<LinearVelocity>(entity).unwrap().y.abs() < 0.1);
    }

    #[test]
    fn wall_blocks_commanded_motion_and_release_stops() {
        let mut app = simulation();
        app.world_mut().spawn((
            RigidBody::Static,
            Collider::cuboid(5.0, 2.0, 0.2),
            Transform::from_xyz(0.0, 1.0, -2.0),
        ));
        let entity = rover(
            &mut app,
            DriveCommand {
                linear: 1.0,
                angular: 0.0,
            },
        );
        tick(&mut app, 240);
        let z = app.world().get::<Position>(entity).unwrap().z;
        assert!(
            z < -1.0 && z > -1.7,
            "rover penetrated wall or failed to move: {z}"
        );
        *app.world_mut().get_mut::<DriveCommand>(entity).unwrap() = DriveCommand::default();
        tick(&mut app, 60);
        assert!((app.world().get::<Position>(entity).unwrap().z - z).abs() < 0.05);
        // Wheel odometry is ideal and may overestimate travel while blocked.
        assert!(app.world().get::<Odometry>(entity).unwrap().z < z - 1.0);
    }

    #[test]
    fn turning_uses_physics_rotation() {
        let mut app = simulation();
        let entity = rover(
            &mut app,
            DriveCommand {
                linear: 0.0,
                angular: 1.0,
            },
        );
        tick(&mut app, 60);
        let rotation = app.world().get::<Rotation>(entity).unwrap().0;
        let forward = rotation * Vec3::NEG_Z;
        assert!(forward.x < -0.6, "{forward:?}");
        let position = app.world().get::<Position>(entity).unwrap().0;
        assert!(position.x.abs() < 0.05 && position.z.abs() < 0.05);
    }
}
