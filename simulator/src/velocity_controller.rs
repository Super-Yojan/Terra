//! Phone-like IMU/VIO feedback into the portable controller, actuated through Avian forces.
use crate::terra::{DriveCommand, DriveConfig, Odometry, Rover, WheelSpeeds};
use avian3d::prelude::*;
use bevy::prelude::*;
use terra_control::{ControllerConfig, VelocityController};
use terra_state::{EstimatorConfig, VelocityEstimator};
use terra_types::{
    ImuSample, MotorOutput, Quaternion, StopReason, Vector3, VelocityTarget, VioSample,
};

#[derive(Resource, Clone)]
pub struct VelocitySimulationConfig {
    pub enabled: bool,
    pub imu_enabled: bool,
    pub vio_enabled: bool,
    pub vio_rate: f64,
    pub wheel_force: f32,
    pub linear_drag: f32,
    pub yaw_drag: f32,
    pub lateral_drag: f32,
}
impl Default for VelocitySimulationConfig {
    fn default() -> Self {
        Self {
            enabled: true,
            imu_enabled: true,
            vio_enabled: true,
            vio_rate: 20.0,
            wheel_force: 30.0,
            linear_drag: 20.0,
            yaw_drag: 3.6,
            lateral_drag: 40.0,
        }
    }
}
pub struct TerraVelocityControlPlugin;
impl Plugin for TerraVelocityControlPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<VelocitySimulationConfig>()
            .add_systems(
                PreUpdate,
                mount_controllers.after(crate::terra::reconcile_fleet),
            )
            .add_systems(
                FixedPostUpdate,
                control_and_apply
                    .after(PhysicsSystems::Prepare)
                    .before(PhysicsSystems::StepSimulation),
            );
    }
}
pub(crate) fn ideal_drive_enabled(config: Option<Res<VelocitySimulationConfig>>) -> bool {
    config.is_none_or(|config| !config.enabled)
}
#[derive(Component)]
pub struct VelocityControlState {
    estimator: VelocityEstimator,
    controller: VelocityController,
    last_velocity: Option<Vec3>,
    last_vio: Option<f64>,
    pub output: MotorOutput,
}
impl VelocityControlState {
    pub fn healthy(&self, time: f64) -> bool {
        self.estimator.estimate(time).health == terra_types::Health::Ready
    }
}
fn mount_controllers(
    mut commands: Commands,
    config: Res<VelocitySimulationConfig>,
    drive: Res<DriveConfig>,
    rovers: Query<Entity, Added<Rover>>,
) {
    for entity in &rovers {
        let controller_config = ControllerConfig {
            linear_feedforward: (config.linear_drag / (2.0 * config.wheel_force)) as f64,
            yaw_feedforward: (config.yaw_drag / (config.wheel_force * drive.track_width)) as f64,
            ..Default::default()
        };
        commands.entity(entity).insert((
            VelocityControlState {
                estimator: VelocityEstimator::new(EstimatorConfig::default()).unwrap(),
                controller: VelocityController::new(controller_config)
                    .expect("invalid simulated motor parameters"),
                last_velocity: None,
                last_vio: None,
                output: MotorOutput::stopped(StopReason::SensorNotReady),
            },
            ConstantForce::default(),
            ConstantTorque::default(),
        ));
    }
}
fn world_vector(value: Vec3) -> Vector3 {
    Vector3 {
        x: -value.z as f64,
        y: -value.x as f64,
        z: value.y as f64,
    }
}
type ControlledRovers<'w, 's> = Query<
    'w,
    's,
    (
        &'static DriveCommand,
        Option<&'static crate::zenoh_bridge::AutonomyMotorGate>,
        &'static Rotation,
        &'static Position,
        &'static LinearVelocity,
        &'static AngularVelocity,
        &'static mut VelocityControlState,
        &'static mut ConstantForce,
        &'static mut ConstantTorque,
        &'static mut WheelSpeeds,
        &'static mut Odometry,
    ),
    With<Rover>,
>;
fn control_and_apply(
    time: Res<Time<Fixed>>,
    config: Res<VelocitySimulationConfig>,
    drive: Res<DriveConfig>,
    mut rovers: ControlledRovers<'_, '_>,
) {
    if !config.enabled {
        for (_, _, _, _, _, _, _, mut force, mut torque, _, _) in &mut rovers {
            force.0 = Vec3::ZERO;
            torque.0 = Vec3::ZERO;
        }
        return;
    }
    let t = time.elapsed_secs_f64();
    let dt = time.delta_secs();
    if dt <= 0.0 {
        return;
    }
    for (
        request,
        gate,
        rotation,
        position,
        velocity,
        angular,
        mut brain,
        mut force,
        mut torque,
        mut wheels,
        mut odometry,
    ) in &mut rovers
    {
        let yaw = rotation.0.to_euler(EulerRot::YXZ).0 as f64;
        let orientation = Quaternion::from_rotation_vector(Vector3 {
            z: yaw,
            ..Vector3::ZERO
        });
        let acceleration = brain
            .last_velocity
            .map_or(Vec3::ZERO, |last| (velocity.0 - last) / dt);
        brain.last_velocity = Some(velocity.0);
        if config.imu_enabled {
            let sample = ImuSample {
                timestamp: t,
                acceleration: orientation.conjugate().rotate(world_vector(acceleration)),
                angular_velocity: Vector3 {
                    z: angular.y as f64,
                    ..Vector3::ZERO
                },
            };
            let _ = brain.estimator.push_imu(sample);
        }
        if config.vio_enabled
            && brain
                .last_vio
                .is_none_or(|last| t - last >= 1.0 / config.vio_rate)
        {
            let _ = brain.estimator.push_vio(VioSample {
                timestamp: t,
                position: world_vector(position.0),
                orientation,
                velocity: world_vector(velocity.0),
                tracked: true,
            });
            brain.last_vio = Some(t);
        }
        if gate.is_some_and(|g| g.reset || g.hold) {
            brain.controller.reset();
        }
        let _ = brain.controller.set_target(VelocityTarget {
            timestamp: t,
            forward: request.linear as f64,
            yaw_rate: request.angular as f64,
        });
        let estimate = brain.estimator.estimate(t);
        let output = if gate.is_some_and(|g| g.hold) {
            MotorOutput::stopped(StopReason::SensorNotReady)
        } else {
            brain.controller.step(estimate)
        };
        brain.output = output;
        let forward = (rotation.0 * Vec3::NEG_Z).with_y(0.0).normalize_or_zero();
        let speed = velocity.0.dot(forward);
        let lateral = velocity.0.with_y(0.0) - forward * speed;
        force.0 = forward
            * ((output.left + output.right) as f32 * config.wheel_force
                - config.linear_drag * speed)
            - lateral * config.lateral_drag;
        torque.0 = Vec3::Y
            * ((output.right - output.left) as f32 * config.wheel_force * drive.track_width * 0.5
                - config.yaw_drag * angular.y);
        *wheels = drive.wheels(DriveCommand {
            linear: speed,
            angular: angular.y,
        });
        odometry.integrate(speed, angular.y, dt);
    }
}
#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        physics::{RoverBody, TerraPhysicsPlugin},
        terra::{DriveCommand, DriveConfig, Odometry, Rover, WheelSpeeds},
    };
    #[test]
    fn feedback_controller_tracks_velocity_using_motor_forces_and_stops() {
        let mut app = App::new();
        app.add_plugins((
            MinimalPlugins,
            TransformPlugin,
            bevy::asset::AssetPlugin::default(),
            bevy::mesh::MeshPlugin,
            TerraPhysicsPlugin,
            TerraVelocityControlPlugin,
        ))
        .insert_resource(DriveConfig::default())
        .insert_resource(Time::<Fixed>::from_hz(100.0))
        .insert_resource(bevy::time::TimeUpdateStrategy::ManualDuration(
            std::time::Duration::from_millis(10),
        ));
        app.finish();
        app.cleanup();
        app.world_mut().spawn((
            RigidBody::Static,
            Collider::cuboid(100.0, 0.2, 100.0),
            Transform::from_xyz(0.0, -0.1, 0.0),
        ));
        let rover = app
            .world_mut()
            .spawn((
                Rover,
                RoverBody::default(),
                DriveCommand {
                    linear: 1.0,
                    angular: 0.3,
                },
                WheelSpeeds::default(),
                Odometry::default(),
                Transform::from_xyz(0.0, 0.1, 0.0),
            ))
            .id();
        for _ in 0..800 {
            app.update();
        }
        let world = app.world();
        let rotation = world.get::<Rotation>(rover).unwrap().0;
        let velocity = world.get::<LinearVelocity>(rover).unwrap().0;
        let forward = velocity.dot(rotation * Vec3::NEG_Z);
        let yaw = world.get::<AngularVelocity>(rover).unwrap().y;
        assert!(
            (forward - 1.0).abs() < 0.08,
            "forward={forward}, output={:?}, yaw={yaw}, time={}",
            world.get::<VelocityControlState>(rover).unwrap().output,
            world.resource::<Time<Fixed>>().elapsed_secs_f64()
        );
        assert!((yaw - 0.3).abs() < 0.08, "yaw={yaw}");
        app.world_mut()
            .get_mut::<DriveCommand>(rover)
            .unwrap()
            .linear = 0.0;
        app.world_mut()
            .get_mut::<DriveCommand>(rover)
            .unwrap()
            .angular = 0.0;
        for _ in 0..500 {
            app.update();
        }
        assert!(
            app.world()
                .get::<LinearVelocity>(rover)
                .unwrap()
                .0
                .with_y(0.0)
                .length()
                < 0.04
        );
    }
}
