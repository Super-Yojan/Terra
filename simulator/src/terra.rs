//! Physics-driven differential drive: metres, seconds and radians; +Y is up,
//! local -Z is forward, and positive yaw turns left. Wheel odometry is ideal.
use crate::physics::{RoverBody, RoverPhysicsConfig};
use avian3d::prelude::*;
use bevy::prelude::*;

#[derive(Default)]
pub struct TerraPlugin {
    pub config: DriveConfig,
}

impl Plugin for TerraPlugin {
    fn build(&self, app: &mut App) {
        self.config
            .validate()
            .expect("invalid differential drive geometry or limits");
        app.insert_resource(self.config.clone())
            .init_resource::<RoverFleet>()
            .add_systems(PreUpdate, reconcile_fleet)
            .add_systems(FixedUpdate, keyboard_command)
            .add_systems(
                FixedPostUpdate,
                drive
                    .run_if(crate::velocity_controller::ideal_drive_enabled)
                    .after(PhysicsSystems::Prepare)
                    .before(PhysicsSystems::StepSimulation),
            );
    }
}

#[derive(Resource, Clone)]
pub struct DriveConfig {
    /// Wheel radius in metres; replace defaults with measured rover geometry.
    pub wheel_radius: f32,
    /// Distance between the left and right wheel contact lines in metres.
    pub track_width: f32,
    /// Maximum wheel angular velocity in radians/second.
    pub max_wheel_speed: f32,
    pub keyboard_linear_speed: f32,
    pub keyboard_angular_speed: f32,
    pub model_path: String,
}

impl Default for DriveConfig {
    fn default() -> Self {
        Self {
            wheel_radius: 0.15,
            track_width: 0.6,
            max_wheel_speed: 20.0,
            keyboard_linear_speed: 1.5,
            keyboard_angular_speed: 1.5,
            model_path: "models/rover.glb".into(),
        }
    }
}

impl DriveConfig {
    pub fn validate(&self) -> Result<(), &'static str> {
        if [self.wheel_radius, self.track_width, self.max_wheel_speed]
            .iter()
            .any(|v| !v.is_finite() || *v <= 0.0)
        {
            return Err("wheel radius, track width and wheel limit must be finite and positive");
        }
        if [self.keyboard_linear_speed, self.keyboard_angular_speed]
            .iter()
            .any(|v| !v.is_finite() || *v < 0.0)
        {
            return Err("keyboard speeds must be finite and nonnegative");
        }
        Ok(())
    }

    pub fn wheels(&self, command: DriveCommand) -> WheelSpeeds {
        if self.validate().is_err() || !command.linear.is_finite() || !command.angular.is_finite() {
            return WheelSpeeds::default();
        }
        // Calculate in f64 to avoid overflow from large but finite commands.
        let half_track = self.track_width as f64 / 2.0;
        let left = (command.linear as f64 - command.angular as f64 * half_track)
            / self.wheel_radius as f64;
        let right = (command.linear as f64 + command.angular as f64 * half_track)
            / self.wheel_radius as f64;
        let scale = (left.abs().max(right.abs()) / self.max_wheel_speed as f64).max(1.0);
        WheelSpeeds {
            left: (left / scale) as f32,
            right: (right / scale) as f32,
        }
    }

    pub fn twist(&self, wheels: WheelSpeeds) -> (f32, f32) {
        (
            self.wheel_radius * (wheels.left + wheels.right) / 2.0,
            self.wheel_radius * (wheels.right - wheels.left) / self.track_width,
        )
    }
}

#[derive(Component)]
#[require(Transform, Visibility)]
pub struct Rover;

/// Remove this marker to control DriveCommand from another system.
#[derive(Component)]
pub struct KeyboardControlled;

/// Body velocity request: forward m/s and left-turn rad/s. Zero means stop.
#[derive(Component, Default, Clone, Copy)]
pub struct DriveCommand {
    pub linear: f32,
    pub angular: f32,
}

/// Achieved wheel angular velocities, rad/s, after saturation.
#[derive(Component, Default, Clone, Copy)]
pub struct WheelSpeeds {
    pub left: f32,
    pub right: f32,
}

/// Ideal wheel odometry relative to the initial rover pose.
#[derive(Component, Default)]
pub struct Odometry {
    pub x: f32,
    pub z: f32,
    pub yaw: f32,
    pub linear_velocity: f32,
    pub angular_velocity: f32,
}

impl Odometry {
    pub(crate) fn integrate(&mut self, v: f32, w: f32, dt: f32) {
        let angle = w * dt;
        // Exact constant-twist arc, stable for straight and near-straight travel.
        let half = angle / 2.0;
        let sinc = if half.abs() < 1e-4 {
            1.0 - half * half / 6.0
        } else {
            half.sin() / half
        };
        let distance = v * dt * sinc;
        self.x -= distance * (self.yaw + half).sin();
        self.z -= distance * (self.yaw + half).cos();
        self.yaw = (self.yaw + angle + std::f32::consts::PI).rem_euclid(std::f32::consts::TAU)
            - std::f32::consts::PI;
        self.linear_velocity = v;
        self.angular_velocity = w;
    }
}

/// Limit the number of simultaneously rendered rover camera pairs.
pub const MAX_ROVERS: usize = 32;
/// Stable within a run; retired IDs are never reassigned.
#[derive(Component, Clone, Copy, Debug, Eq, PartialEq, Ord, PartialOrd)]
pub struct RoverId(pub u64);
#[derive(Component, Clone, Copy)]
pub struct RoverSlot(pub usize);
#[derive(Resource)]
pub struct RoverFleet {
    count: usize,
    next_id: u64,
}
impl Default for RoverFleet {
    fn default() -> Self {
        Self {
            count: 1,
            next_id: 0,
        }
    }
}
impl RoverFleet {
    pub fn count(&self) -> usize {
        self.count
    }
    pub fn set_count(&mut self, count: usize) -> Result<(), &'static str> {
        if count > MAX_ROVERS {
            return Err("rover count must be between 0 and 32");
        }
        self.count = count;
        Ok(())
    }
    pub fn from_env() -> Result<Self, String> {
        let mut fleet = Self::default();
        if let Ok(value) = std::env::var("TERRA_ROVER_COUNT") {
            let count = value
                .parse::<usize>()
                .map_err(|_| "TERRA_ROVER_COUNT must be an integer".to_owned())?;
            fleet.set_count(count).map_err(str::to_owned)?;
        }
        Ok(fleet)
    }
}
pub(crate) fn reconcile_fleet(
    mut commands: Commands,
    assets: Option<Res<AssetServer>>,
    config: Res<DriveConfig>,
    physics: Res<RoverPhysicsConfig>,
    mut fleet: ResMut<RoverFleet>,
    world: Option<Res<crate::world::WorldConfig>>,
    rovers: Query<(Entity, &RoverId, &RoverSlot, &Transform), With<Rover>>,
) {
    let mut existing: Vec<_> = rovers.iter().collect();
    existing.sort_by_key(|(_, id, _, _)| id.0);
    if existing.len() > fleet.count() {
        for (entity, _, _, _) in &existing[fleet.count()..] {
            commands.entity(*entity).despawn();
        }
        return;
    }
    let mut occupied: std::collections::BTreeSet<_> =
        existing.iter().map(|(_, _, slot, _)| slot.0).collect();
    let spacing = physics.chassis_size.x.max(physics.chassis_size.z) + 0.6;
    let max_distance = world.as_ref().map_or(100.0, |world| world.size) * 0.38;
    let available_slots = 1 + 4 * (max_distance / spacing).floor() as usize;
    let mut positions: Vec<_> = existing
        .iter()
        .map(|(_, _, _, pose)| pose.translation.with_y(0.0))
        .collect();
    let offset_for = |slot: usize| {
        if slot == 0 {
            Vec3::ZERO
        } else {
            let distance = ((slot - 1) / 4 + 1) as f32 * spacing;
            match (slot - 1) % 4 {
                0 => Vec3::X * distance,
                1 => Vec3::NEG_X * distance,
                2 => Vec3::Z * distance,
                _ => Vec3::NEG_Z * distance,
            }
        }
    };
    for _ in existing.len()..fleet.count() {
        // Retry on a later update if moving rovers occupy every safe road spawn.
        let Some(slot) = (0..available_slots).find(|slot| {
            !occupied.contains(slot)
                && positions
                    .iter()
                    .all(|position| position.distance(offset_for(*slot)) >= spacing - 0.1)
        }) else {
            break;
        };
        occupied.insert(slot);
        let offset = offset_for(slot);
        positions.push(offset);
        let id = RoverId(fleet.next_id);
        fleet.next_id = fleet
            .next_id
            .checked_add(1)
            .expect("rover ID space exhausted");
        let mut rover = commands.spawn((
            Name::new(format!("Terra {}", id.0)),
            Rover,
            id,
            RoverSlot(slot),
            DriveCommand::default(),
            WheelSpeeds::default(),
            Odometry::default(),
            RoverBody::new(&physics),
            Transform::from_translation(offset + Vec3::Y * (physics.chassis_size.y / 2.0 + 0.02)),
        ));
        if slot == 0 {
            rover.insert(KeyboardControlled);
        }
        if let Some(assets) = &assets {
            rover.with_children(|parent| {
                parent.spawn((
                    WorldAssetRoot(
                        assets.load(GltfAssetLabel::Scene(0).from_asset(config.model_path.clone())),
                    ),
                    Transform::from_scale(Vec3::splat(physics.model_scale)),
                ));
            });
        }
    }
}

fn keyboard_command(
    keys: Res<ButtonInput<KeyCode>>,
    config: Res<DriveConfig>,
    mut rovers: Query<&mut DriveCommand, (With<Rover>, With<KeyboardControlled>)>,
) {
    let axis = |positive, negative| {
        keys.pressed(positive) as i32 as f32 - keys.pressed(negative) as i32 as f32
    };
    for mut command in &mut rovers {
        *command = if keys.pressed(KeyCode::Space) {
            DriveCommand::default()
        } else {
            DriveCommand {
                linear: axis(KeyCode::KeyW, KeyCode::KeyS) * config.keyboard_linear_speed,
                angular: axis(KeyCode::KeyA, KeyCode::KeyD) * config.keyboard_angular_speed,
            }
        };
    }
}

type RoverDriveQuery<'w, 's> = Query<
    'w,
    's,
    (
        &'static DriveCommand,
        &'static mut WheelSpeeds,
        &'static mut Odometry,
        &'static Rotation,
        &'static mut LinearVelocity,
        &'static mut AngularVelocity,
    ),
    With<Rover>,
>;

pub(crate) fn drive(time: Res<Time<Fixed>>, config: Res<DriveConfig>, mut rovers: RoverDriveQuery) {
    for (command, mut wheels, mut pose, rotation, mut linear, mut angular) in &mut rovers {
        *wheels = config.wheels(*command);
        let (v, w) = if config.validate().is_ok() {
            config.twist(*wheels)
        } else {
            (0.0, 0.0)
        };
        pose.integrate(v, w, time.delta_secs());
        let forward = (rotation.0 * Vec3::NEG_Z).with_y(0.0).normalize_or_zero();
        // Leave vertical velocity to gravity and the contact solver.
        linear.x = forward.x * v;
        linear.z = forward.z * v;
        angular.0 = Vec3::Y * w;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    fn near(a: f32, b: f32) {
        assert!((a - b).abs() < 1e-5, "{a} != {b}");
    }
    #[test]
    fn controller_preserves_gravity_and_does_not_teleport() {
        let mut app = App::new();
        let mut time = Time::<Fixed>::default();
        time.advance_by(std::time::Duration::from_secs(1));
        app.insert_resource(time)
            .insert_resource(DriveConfig::default())
            .insert_resource(ButtonInput::<KeyCode>::default())
            .add_systems(FixedUpdate, (keyboard_command, drive).chain());
        let entity = app
            .world_mut()
            .spawn((
                Rover,
                KeyboardControlled,
                DriveCommand::default(),
                WheelSpeeds::default(),
                Odometry::default(),
                Transform::from_xyz(3.0, 2.0, 4.0),
                Rotation(Quat::from_rotation_y(std::f32::consts::FRAC_PI_2)),
                LinearVelocity(Vec3::new(0.0, -2.0, 0.0)),
                AngularVelocity::ZERO,
            ))
            .id();
        app.world_mut()
            .resource_mut::<ButtonInput<KeyCode>>()
            .press(KeyCode::KeyW);
        app.world_mut().run_schedule(FixedUpdate);
        let velocity = app.world().get::<LinearVelocity>(entity).unwrap().0;
        near(velocity.x, -1.5);
        near(velocity.y, -2.0);
        near(velocity.z, 0.0);
        assert_eq!(
            app.world().get::<Transform>(entity).unwrap().translation,
            Vec3::new(3.0, 2.0, 4.0)
        );
        app.world_mut()
            .resource_mut::<ButtonInput<KeyCode>>()
            .release(KeyCode::KeyW);
        app.world_mut().run_schedule(FixedUpdate);
        let velocity = app.world().get::<LinearVelocity>(entity).unwrap().0;
        near(velocity.x, 0.0);
        near(velocity.y, -2.0);
    }

    #[test]
    fn straight_reverse_and_stop() {
        let config = DriveConfig::default();
        for speed in [1.0, -1.0, 0.0] {
            let wheels = config.wheels(DriveCommand {
                linear: speed,
                angular: 0.0,
            });
            let (v, w) = config.twist(wheels);
            near(v, speed);
            near(w, 0.0);
            let mut pose = Odometry::default();
            pose.integrate(v, w, 1.0);
            near(pose.x, 0.0);
            near(pose.z, -speed);
        }
    }
    #[test]
    fn spin_and_quarter_circle() {
        let mut pose = Odometry::default();
        pose.integrate(0.0, 1.0, 1.0);
        near(pose.x, 0.0);
        near(pose.z, 0.0);
        near(pose.yaw, 1.0);
        let mut pose = Odometry::default();
        pose.integrate(1.0, 1.0, std::f32::consts::FRAC_PI_2);
        near(pose.x, -1.0);
        near(pose.z, -1.0);
    }
    #[test]
    fn wheel_limit_preserves_curvature_and_invalid_input_stops() {
        let config = DriveConfig::default();
        let wheels = config.wheels(DriveCommand {
            linear: 100.0,
            angular: 50.0,
        });
        assert!(wheels.left.abs() <= config.max_wheel_speed);
        assert!(wheels.right.abs() <= config.max_wheel_speed);
        let (v, w) = config.twist(wheels);
        near(w / v, 0.5);
        let wheels = config.wheels(DriveCommand {
            linear: f32::NAN,
            angular: 0.0,
        });
        near(wheels.left, 0.0);
        near(wheels.right, 0.0);
        assert!(
            DriveConfig {
                track_width: 0.0,
                ..config
            }
            .validate()
            .is_err()
        );
    }
    #[test]
    fn fleet_grows_shrinks_and_does_not_reuse_retired_ids() {
        let mut app = App::new();
        app.insert_resource(DriveConfig::default())
            .insert_resource(RoverPhysicsConfig::default())
            .insert_resource(RoverFleet::default())
            .add_systems(Update, reconcile_fleet);
        app.update();
        let world = app.world_mut();
        let original = world
            .query_filtered::<(Entity, &RoverId), With<Rover>>()
            .single(world)
            .unwrap();
        let first = original.0;
        assert_eq!(original.1.0, 0);
        world.resource_mut::<RoverFleet>().set_count(3).unwrap();
        app.update();
        let world = app.world_mut();
        assert_eq!(
            world
                .query_filtered::<&RoverId, With<Rover>>()
                .iter(world)
                .count(),
            3
        );
        world.resource_mut::<RoverFleet>().set_count(1).unwrap();
        app.update();
        assert!(app.world().get::<Rover>(first).is_some());
        app.world_mut()
            .resource_mut::<RoverFleet>()
            .set_count(3)
            .unwrap();
        app.update();
        let world = app.world_mut();
        let mut ids: Vec<_> = world
            .query_filtered::<&RoverId, With<Rover>>()
            .iter(world)
            .map(|id| id.0)
            .collect();
        ids.sort();
        assert_eq!(ids, [0, 3, 4]);
        assert!(
            world
                .resource_mut::<RoverFleet>()
                .set_count(MAX_ROVERS + 1)
                .is_err()
        );
        world.resource_mut::<RoverFleet>().set_count(0).unwrap();
        app.update();
        let world = app.world_mut();
        assert_eq!(
            world
                .query_filtered::<&RoverId, With<Rover>>()
                .iter(world)
                .count(),
            0
        );
    }
    #[test]
    fn live_growth_avoids_a_rover_occupying_a_spawn_location() {
        let mut app = App::new();
        app.insert_resource(DriveConfig::default())
            .insert_resource(RoverPhysicsConfig::default())
            .init_resource::<RoverFleet>()
            .add_systems(Update, reconcile_fleet);
        app.update();
        let world = app.world_mut();
        let first = world
            .query_filtered::<Entity, With<Rover>>()
            .single(world)
            .unwrap();
        world.get_mut::<Transform>(first).unwrap().translation.x = 1.5;
        world.resource_mut::<RoverFleet>().set_count(2).unwrap();
        app.update();
        let world = app.world_mut();
        let positions: Vec<_> = world
            .query_filtered::<&Transform, With<Rover>>()
            .iter(world)
            .map(|pose| pose.translation)
            .collect();
        assert_eq!(positions.len(), 2);
        assert!(positions[0].distance(positions[1]) >= 1.4);
    }
}
