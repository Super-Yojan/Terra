//! NEXT competition practice field for Zatara.
//!
//! A small-sided soccer pitch with tennis balls and open-front deposit buckets.
//! Dimensions are a clean practice layout, not an official surveyed field.
//! The primary rover is named Zatara and uses the existing differential drive.

use avian3d::prelude::*;
use bevy::prelude::*;

/// Square ground that contains the pitch, boards, and camera apron.
pub const WORLD_SIZE: f32 = 32.0;
/// Pitch width along Bevy X, metres.
pub const PITCH_HALF_WIDTH: f32 = 6.0;
/// South goal line. Zatara spawns at the origin, on this half, facing −Z.
pub const PITCH_SOUTH: f32 = 5.0;
/// North goal line. Deposit buckets sit inside this end.
pub const PITCH_NORTH: f32 = PITCH_SOUTH - 18.0;
const BOARD_HEIGHT: f32 = 0.28;
const BOARD_THICKNESS: f32 = 0.12;
const MARKING_Y: f32 = 0.008;

/// Regulation tennis ball, metres and kilograms.
pub const TENNIS_BALL_RADIUS: f32 = 0.0335;
pub const TENNIS_BALL_MASS: f32 = 0.058;

pub fn enabled() -> bool {
    matches!(
        std::env::var("TERRA_NEXT").as_deref(),
        Ok("1") | Ok("true") | Ok("TRUE")
    )
}

pub fn install(app: &mut App) {
    if enabled() {
        app.add_systems(Startup, spawn_field)
            .add_systems(Update, name_zatara)
            .add_systems(FixedUpdate, settle_deposits);
    }
}

pub fn on_pitch(x: f32, z: f32, margin: f32) -> bool {
    x.abs() <= PITCH_HALF_WIDTH - margin && z <= PITCH_SOUTH - margin && z >= PITCH_NORTH + margin
}

#[derive(Clone, Copy, Debug)]
pub struct BucketPose {
    pub center_x: f32,
    pub mouth_z: f32,
    pub interior_width: f32,
    pub interior_depth: f32,
    pub wall_height: f32,
    pub wall_thickness: f32,
}

#[derive(Clone, Copy, Debug)]
pub struct Wall {
    pub center: Vec3,
    pub size: Vec3,
}

impl BucketPose {
    pub fn bucket(self) -> Bucket {
        let half_w = self.interior_width * 0.5;
        Bucket {
            interior_min: Vec3::new(
                self.center_x - half_w,
                0.0,
                self.mouth_z - self.interior_depth,
            ),
            interior_max: Vec3::new(self.center_x + half_w, self.wall_height, self.mouth_z),
            rest: Vec3::new(
                self.center_x,
                TENNIS_BALL_RADIUS,
                self.mouth_z - self.interior_depth + TENNIS_BALL_RADIUS + 0.05,
            ),
        }
    }

    /// Back wall and two side walls. The south face is open so a ball can be pushed in.
    pub fn walls(self) -> [Wall; 3] {
        let half_w = self.interior_width * 0.5;
        let back_z = self.mouth_z - self.interior_depth - self.wall_thickness * 0.5;
        let side_z = self.mouth_z - self.interior_depth * 0.5;
        let y = self.wall_height * 0.5;
        [
            Wall {
                center: Vec3::new(self.center_x, y, back_z),
                size: Vec3::new(
                    self.interior_width + self.wall_thickness * 2.0,
                    self.wall_height,
                    self.wall_thickness,
                ),
            },
            Wall {
                center: Vec3::new(
                    self.center_x - half_w - self.wall_thickness * 0.5,
                    y,
                    side_z,
                ),
                size: Vec3::new(self.wall_thickness, self.wall_height, self.interior_depth),
            },
            Wall {
                center: Vec3::new(
                    self.center_x + half_w + self.wall_thickness * 0.5,
                    y,
                    side_z,
                ),
                size: Vec3::new(self.wall_thickness, self.wall_height, self.interior_depth),
            },
        ]
    }
}

#[derive(Component, Clone, Copy, Debug)]
pub struct Bucket {
    pub interior_min: Vec3,
    pub interior_max: Vec3,
    pub rest: Vec3,
}

impl Bucket {
    pub fn contains(self, point: Vec3) -> bool {
        point.x >= self.interior_min.x
            && point.x <= self.interior_max.x
            && point.z >= self.interior_min.z
            && point.z <= self.interior_max.z
            && point.y <= self.interior_max.y + 0.05
    }
}

pub fn buckets() -> [BucketPose; 2] {
    let pose = BucketPose {
        center_x: 0.0,
        mouth_z: PITCH_NORTH + 1.55,
        interior_width: 1.2,
        interior_depth: 0.75,
        wall_height: 0.30,
        wall_thickness: 0.08,
    };
    [
        BucketPose {
            center_x: -2.6,
            ..pose
        },
        BucketPose {
            center_x: 2.6,
            ..pose
        },
    ]
}

pub fn ball_positions() -> [Vec3; 6] {
    let y = TENNIS_BALL_RADIUS;
    [
        Vec3::new(-2.4, y, -3.2),
        Vec3::new(-0.7, y, -4.5),
        Vec3::new(1.0, y, -3.4),
        Vec3::new(2.6, y, -4.9),
        Vec3::new(-1.7, y, -5.7),
        Vec3::new(0.8, y, -6.3),
    ]
}

pub fn pitch_boards() -> [Wall; 4] {
    let y = BOARD_HEIGHT * 0.5;
    let length = PITCH_SOUTH - PITCH_NORTH;
    let width = PITCH_HALF_WIDTH * 2.0;
    let mid_z = (PITCH_SOUTH + PITCH_NORTH) * 0.5;
    let t = BOARD_THICKNESS;
    [
        Wall {
            center: Vec3::new(0.0, y, PITCH_SOUTH + t * 0.5),
            size: Vec3::new(width + t * 2.0, BOARD_HEIGHT, t),
        },
        Wall {
            center: Vec3::new(0.0, y, PITCH_NORTH - t * 0.5),
            size: Vec3::new(width + t * 2.0, BOARD_HEIGHT, t),
        },
        Wall {
            center: Vec3::new(-PITCH_HALF_WIDTH - t * 0.5, y, mid_z),
            size: Vec3::new(t, BOARD_HEIGHT, length),
        },
        Wall {
            center: Vec3::new(PITCH_HALF_WIDTH + t * 0.5, y, mid_z),
            size: Vec3::new(t, BOARD_HEIGHT, length),
        },
    ]
}

#[derive(Component)]
pub struct NextField;
#[derive(Component)]
pub struct TennisBall;
#[derive(Component)]
pub struct PitchBoard;
/// Primary rover in the NEXT scene.
#[derive(Component)]
pub struct Zatara;

fn spawn_field(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    let cube = meshes.add(Cuboid::new(1.0, 1.0, 1.0));
    let material = |materials: &mut Assets<StandardMaterial>, color, roughness| {
        materials.add(StandardMaterial {
            base_color: color,
            perceptual_roughness: roughness,
            ..default()
        })
    };
    let grass = material(&mut materials, Color::srgb(0.18, 0.52, 0.22), 0.95);
    let line = material(&mut materials, Color::srgb(0.95, 0.95, 0.92), 0.7);
    let board = material(&mut materials, Color::srgb(0.90, 0.92, 0.94), 0.55);
    let bucket_color = material(&mut materials, Color::srgb(0.93, 0.68, 0.08), 0.45);
    let bucket_floor = material(&mut materials, Color::srgb(0.72, 0.48, 0.05), 0.6);
    let ball_color = material(&mut materials, Color::srgb(0.79, 0.95, 0.10), 0.35);
    let ball_mesh = meshes.add(Sphere::new(TENNIS_BALL_RADIUS).mesh().ico(3).unwrap());
    assert!(
        ball_positions()
            .iter()
            .all(|ball| on_pitch(ball.x, ball.z, 0.4)),
        "NEXT tennis balls must sit on the pitch"
    );

    let length = PITCH_SOUTH - PITCH_NORTH;
    let width = PITCH_HALF_WIDTH * 2.0;
    let mid_z = (PITCH_SOUTH + PITCH_NORTH) * 0.5;
    commands.spawn((
        Name::new("NEXT pitch"),
        NextField,
        Mesh3d(cube.clone()),
        MeshMaterial3d(grass),
        Transform::from_xyz(0.0, 0.003, mid_z).with_scale(Vec3::new(width, 0.004, length)),
    ));

    paint_lines(&mut commands, &cube, &line);
    for wall in pitch_boards() {
        commands.spawn((
            Name::new("NEXT board"),
            NextField,
            PitchBoard,
            RigidBody::Static,
            Collider::cuboid(1.0, 1.0, 1.0),
            Friction::new(0.6),
            Restitution::new(0.15),
            Mesh3d(cube.clone()),
            MeshMaterial3d(board.clone()),
            Transform::from_translation(wall.center).with_scale(wall.size),
        ));
    }
    for (index, pose) in buckets().into_iter().enumerate() {
        let bucket = pose.bucket();
        commands.spawn((
            Name::new(format!("NEXT bucket {}", index + 1)),
            NextField,
            bucket,
            Mesh3d(cube.clone()),
            MeshMaterial3d(bucket_floor.clone()),
            Transform::from_xyz(
                pose.center_x,
                0.005,
                pose.mouth_z - pose.interior_depth * 0.5,
            )
            .with_scale(Vec3::new(pose.interior_width, 0.006, pose.interior_depth)),
        ));
        for wall in pose.walls() {
            commands.spawn((
                Name::new("NEXT bucket wall"),
                NextField,
                RigidBody::Static,
                Collider::cuboid(1.0, 1.0, 1.0),
                Friction::new(0.7),
                Restitution::new(0.05),
                Mesh3d(cube.clone()),
                MeshMaterial3d(bucket_color.clone()),
                Transform::from_translation(wall.center).with_scale(wall.size),
            ));
        }
    }
    for (index, position) in ball_positions().into_iter().enumerate() {
        commands.spawn((
            Name::new(format!("Tennis ball {}", index + 1)),
            NextField,
            TennisBall,
            RigidBody::Dynamic,
            Collider::sphere(TENNIS_BALL_RADIUS),
            Mass(TENNIS_BALL_MASS),
            Friction::new(0.65),
            Restitution::new(0.55),
            LinearDamping(0.45),
            AngularDamping(0.55),
            SweptCcd::default(),
            Mesh3d(ball_mesh.clone()),
            MeshMaterial3d(ball_color.clone()),
            Transform::from_translation(position),
        ));
    }
    eprintln!(
        "Terra NEXT: Zatara practice pitch, {} tennis balls, {} deposit buckets. W/A/S/D drives the primary rover when Zenoh is off.",
        ball_positions().len(),
        buckets().len()
    );
}

fn paint_lines(commands: &mut Commands, cube: &Handle<Mesh>, line: &Handle<StandardMaterial>) {
    let length = PITCH_SOUTH - PITCH_NORTH;
    let width = PITCH_HALF_WIDTH * 2.0;
    let mid_z = (PITCH_SOUTH + PITCH_NORTH) * 0.5;
    let y = MARKING_Y;
    let stripe = |commands: &mut Commands, center: Vec3, size: Vec3| {
        commands.spawn((
            Name::new("NEXT line"),
            NextField,
            Mesh3d(cube.clone()),
            MeshMaterial3d(line.clone()),
            Transform::from_translation(center).with_scale(size),
        ));
    };
    // Touchlines, goal lines, and the halfway line.
    stripe(
        commands,
        Vec3::new(0.0, y, PITCH_SOUTH),
        Vec3::new(width, 0.006, 0.08),
    );
    stripe(
        commands,
        Vec3::new(0.0, y, PITCH_NORTH),
        Vec3::new(width, 0.006, 0.08),
    );
    stripe(
        commands,
        Vec3::new(-PITCH_HALF_WIDTH, y, mid_z),
        Vec3::new(0.08, 0.006, length),
    );
    stripe(
        commands,
        Vec3::new(PITCH_HALF_WIDTH, y, mid_z),
        Vec3::new(0.08, 0.006, length),
    );
    stripe(
        commands,
        Vec3::new(0.0, y, mid_z),
        Vec3::new(width, 0.006, 0.08),
    );
    // Penalty areas at each end. Buckets occupy the north area.
    for goal_z in [PITCH_SOUTH, PITCH_NORTH] {
        let sign = if goal_z > 0.0 { -1.0 } else { 1.0 };
        let box_depth = 2.6;
        let box_half = 3.4;
        let near_z = goal_z + sign * box_depth;
        stripe(
            commands,
            Vec3::new(0.0, y, near_z),
            Vec3::new(box_half * 2.0, 0.006, 0.08),
        );
        stripe(
            commands,
            Vec3::new(-box_half, y, goal_z + sign * box_depth * 0.5),
            Vec3::new(0.08, 0.006, box_depth),
        );
        stripe(
            commands,
            Vec3::new(box_half, y, goal_z + sign * box_depth * 0.5),
            Vec3::new(0.08, 0.006, box_depth),
        );
    }
    let segments = 24;
    let radius = 1.7;
    for index in 0..segments {
        let angle = index as f32 * std::f32::consts::TAU / segments as f32;
        stripe(
            commands,
            Vec3::new(angle.cos() * radius, y, mid_z + angle.sin() * radius),
            Vec3::new(0.16, 0.006, 0.16),
        );
    }
    stripe(
        commands,
        Vec3::new(0.0, y, mid_z),
        Vec3::new(0.22, 0.006, 0.22),
    );
}

fn name_zatara(
    mut commands: Commands,
    rovers: Query<(Entity, &crate::terra::RoverSlot), (With<crate::terra::Rover>, Without<Zatara>)>,
) {
    for (entity, slot) in &rovers {
        if slot.0 == 0 {
            commands
                .entity(entity)
                .insert((Name::new("Zatara"), Zatara));
        }
    }
}

/// Draws a ball that has crossed a bucket mouth toward the back wall so it stays deposited.
fn settle_deposits(
    time: Res<Time<Fixed>>,
    mut balls: Query<(&Position, &mut LinearVelocity), With<TennisBall>>,
    bins: Query<&Bucket>,
) {
    let dt = time.delta_secs();
    if dt <= 0.0 {
        return;
    }
    let damp = (-5.0 * dt).exp();
    for (position, mut velocity) in &mut balls {
        for bucket in &bins {
            if !bucket.contains(position.0) {
                continue;
            }
            let to_rest = bucket.rest - position.0;
            let pull = Vec3::new(to_rest.x, 0.0, to_rest.z).clamp_length_max(0.4) * 12.0;
            velocity.x += pull.x * dt;
            velocity.z += pull.z * dt;
            velocity.x *= damp;
            velocity.z *= damp;
            // Mouths open toward +Z. Shed speed that would roll the ball back out.
            if velocity.z > 0.0 {
                velocity.z *= damp;
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        physics::{RoverBody, RoverPhysicsConfig, TerraPhysicsPlugin},
        terra::{
            DriveCommand, DriveConfig, Odometry, Rover, RoverFleet, TerraPlugin, WheelSpeeds, drive,
        },
        velocity_controller::TerraVelocityControlPlugin,
        world::{TerraWorldPlugin, WorldConfig},
    };
    use bevy::{asset::AssetPlugin, mesh::MeshPlugin, time::TimeUpdateStrategy};
    use std::time::Duration;

    #[test]
    fn layout_fits_the_pitch_and_leaves_the_spawn_clear() {
        let spawn = Vec2::ZERO;
        let mut balls = ball_positions();
        assert_eq!(balls.len(), 6);
        for ball in &balls {
            assert!(on_pitch(ball.x, ball.z, 0.5), "{ball:?}");
            assert!((ball.y - TENNIS_BALL_RADIUS).abs() < 1e-4);
            assert!(Vec2::new(ball.x, ball.z).distance(spawn) > 1.5);
        }
        balls.sort_by(|a, b| a.x.total_cmp(&b.x).then(a.z.total_cmp(&b.z)));
        for pair in balls.windows(2) {
            assert!(pair[0].distance(pair[1]) > 0.5, "{pair:?}");
        }
        assert!(WORLD_SIZE * 0.5 > PITCH_HALF_WIDTH + 1.0);
        assert!(WORLD_SIZE * 0.5 > PITCH_SOUTH.abs().max(PITCH_NORTH.abs()) + 1.0);
        let mut seen_x = [false; 2];
        for pose in buckets() {
            let bucket = pose.bucket();
            assert!(bucket.interior_max.z > bucket.interior_min.z);
            assert!(on_pitch(pose.center_x, pose.mouth_z, 0.3));
            assert!(on_pitch(pose.center_x, bucket.interior_min.z, 0.05));
            assert!(
                pose.interior_width > 1.0,
                "Zatara must be able to nose into the bin"
            );
            let [back, left, right] = pose.walls();
            let back_inner_z = back.center.z + back.size.z * 0.5;
            assert!((back_inner_z - bucket.interior_min.z).abs() < 1e-4);
            assert!((left.center.x + left.size.x * 0.5 - bucket.interior_min.x).abs() < 1e-4);
            assert!((right.center.x - right.size.x * 0.5 - bucket.interior_max.x).abs() < 1e-4);
            assert!(bucket.contains(bucket.rest));
            seen_x[usize::from(pose.center_x > 0.0)] = true;
        }
        assert_eq!(seen_x, [true, true]);
        for board in pitch_boards() {
            assert!(board.size.min_element() > 0.0);
            assert!(board.center.y > 0.0);
        }
    }

    #[test]
    fn next_world_replaces_the_practice_town() {
        let mut config = WorldConfig::default();
        config.prepare_next_field();
        assert!(!config.landscape.enabled);
        assert!(!config.tiles.enabled);
        assert_eq!(config.size, WORLD_SIZE);
        assert!(config.validate().is_ok());
    }

    #[test]
    fn scene_spawns_balls_buckets_boards_and_zatara() {
        let mut app = App::new();
        app.insert_resource(Assets::<Mesh>::default())
            .insert_resource(Assets::<StandardMaterial>::default())
            .insert_resource(Time::<()>::default())
            .insert_resource(RoverPhysicsConfig::default())
            .insert_resource(ButtonInput::<KeyCode>::default())
            .add_plugins((
                TerraWorldPlugin {
                    config: {
                        let mut config = WorldConfig::default();
                        config.prepare_next_field();
                        config
                    },
                },
                TerraPlugin::default(),
            ))
            .add_systems(Startup, spawn_field)
            .add_systems(Update, name_zatara);
        app.update();
        let world = app.world_mut();
        assert_eq!(
            world.query::<&TennisBall>().iter(world).count(),
            ball_positions().len()
        );
        assert_eq!(world.query::<&Bucket>().iter(world).count(), 2);
        assert_eq!(world.query::<&PitchBoard>().iter(world).count(), 4);
        let zatara = world
            .query_filtered::<&Transform, With<Zatara>>()
            .single(world)
            .expect("primary rover");
        assert!(zatara.translation.x.abs() < 0.05);
        assert!(on_pitch(zatara.translation.x, zatara.translation.z, 0.5));
        assert_eq!(world.resource::<RoverFleet>().count(), 1);
        let elements = world.query::<&NextField>().iter(world).count();
        app.update();
        let world = app.world_mut();
        assert_eq!(world.query::<&NextField>().iter(world).count(), elements);
        assert_eq!(world.query::<&Zatara>().iter(world).count(), 1);
    }

    fn physics_app() -> App {
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
        .add_systems(
            FixedUpdate,
            (settle_deposits, drive)
                .chain()
                .before(PhysicsSystems::StepSimulation),
        );
        app.finish();
        app.cleanup();
        app.world_mut().spawn((
            RigidBody::Static,
            Collider::cuboid(40.0, 0.2, 40.0),
            Transform::from_xyz(0.0, -0.1, 0.0),
        ));
        app
    }

    fn tick(app: &mut App, frames: usize) {
        for _ in 0..frames {
            app.update();
        }
    }

    #[test]
    fn zatara_pushes_a_tennis_ball() {
        let mut app = physics_app();
        let ball = app
            .world_mut()
            .spawn((
                TennisBall,
                RigidBody::Dynamic,
                Collider::sphere(TENNIS_BALL_RADIUS),
                Mass(TENNIS_BALL_MASS),
                Friction::new(0.65),
                Restitution::new(0.2),
                LinearDamping(0.2),
                SweptCcd::default(),
                Transform::from_xyz(0.0, TENNIS_BALL_RADIUS + 0.02, -1.05),
            ))
            .id();
        let rover = app
            .world_mut()
            .spawn((
                Rover,
                RoverBody::default(),
                DriveCommand {
                    linear: 1.2,
                    angular: 0.0,
                },
                WheelSpeeds::default(),
                Odometry::default(),
                Transform::from_xyz(0.0, 0.2, 0.0),
            ))
            .id();
        tick(&mut app, 180);
        let ball_z = app.world().get::<Position>(ball).unwrap().z;
        let rover_z = app.world().get::<Position>(rover).unwrap().z;
        assert!(
            ball_z < -1.4,
            "ball was not pushed north: ball {ball_z}, rover {rover_z}"
        );
        assert!(
            rover_z < -0.2,
            "rover did not drive toward the ball: {rover_z}"
        );
    }

    #[test]
    fn force_drive_pushes_a_tennis_ball() {
        let mut app = App::new();
        app.add_plugins((
            MinimalPlugins,
            TransformPlugin,
            AssetPlugin::default(),
            MeshPlugin,
            TerraPhysicsPlugin,
            TerraVelocityControlPlugin,
        ))
        .insert_resource(DriveConfig::default())
        .insert_resource(Time::<Fixed>::from_hz(100.0))
        .insert_resource(TimeUpdateStrategy::ManualDuration(Duration::from_millis(
            10,
        )));
        app.finish();
        app.cleanup();
        app.world_mut().spawn((
            RigidBody::Static,
            Collider::cuboid(40.0, 0.2, 40.0),
            Transform::from_xyz(0.0, -0.1, 0.0),
        ));
        let ball = app
            .world_mut()
            .spawn((
                TennisBall,
                RigidBody::Dynamic,
                Collider::sphere(TENNIS_BALL_RADIUS),
                Mass(TENNIS_BALL_MASS),
                Friction::new(0.65),
                Restitution::new(0.2),
                LinearDamping(0.2),
                SweptCcd::default(),
                Transform::from_xyz(0.0, TENNIS_BALL_RADIUS + 0.02, -1.4),
            ))
            .id();
        app.world_mut().spawn((
            Rover,
            RoverBody::default(),
            DriveCommand {
                linear: 1.0,
                angular: 0.0,
            },
            WheelSpeeds::default(),
            Odometry::default(),
            Transform::from_xyz(0.0, 0.1, 0.0),
        ));
        tick(&mut app, 500);
        let ball_z = app.world().get::<Position>(ball).unwrap().z;
        assert!(
            ball_z < -1.8,
            "force-driven rover did not push the ball: {ball_z}"
        );
    }

    #[test]
    fn bucket_keeps_a_deposited_ball() {
        let pose = buckets()[0];
        let bucket = pose.bucket();
        let mut app = physics_app();
        app.world_mut().spawn(bucket);
        for wall in pose.walls() {
            app.world_mut().spawn((
                RigidBody::Static,
                Collider::cuboid(wall.size.x, wall.size.y, wall.size.z),
                Friction::new(0.7),
                Restitution::ZERO,
                Transform::from_translation(wall.center),
            ));
        }
        let start = Vec3::new(
            pose.center_x,
            TENNIS_BALL_RADIUS + 0.02,
            bucket.rest.z + 0.28,
        );
        assert!(bucket.contains(start), "{start:?} vs {bucket:?}");
        let ball = app
            .world_mut()
            .spawn((
                TennisBall,
                RigidBody::Dynamic,
                Collider::sphere(TENNIS_BALL_RADIUS),
                Mass(TENNIS_BALL_MASS),
                Friction::new(0.65),
                Restitution::new(0.2),
                LinearDamping(0.2),
                SweptCcd::default(),
                LinearVelocity(Vec3::new(0.0, 0.0, 1.6)),
                Transform::from_translation(start),
            ))
            .id();
        tick(&mut app, 150);
        let position = app.world().get::<Position>(ball).unwrap().0;
        assert!(
            bucket.contains(position),
            "ball left the bucket: {position:?}"
        );
        assert!(
            position.z < start.z - 0.05,
            "ball was not drawn toward the back of the bucket: start {start:?} now {position:?}"
        );
        assert!(position.z > bucket.interior_min.z);
    }
}
