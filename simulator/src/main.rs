mod depth_camera;
mod geo;
mod landscape;
mod mission;
mod next_competition;
mod occupancy_map;
mod physics;
mod rgb_camera;
mod terra;
mod velocity_controller;
mod voxel_terrain;
mod world;
mod zenoh_bridge;

use bevy::prelude::*;
use depth_camera::TerraDepthCameraPlugin;
use physics::TerraPhysicsPlugin;
use terra::TerraPlugin;
use world::TerraWorldPlugin;

fn main() {
    let mut app = App::new();
    mission::install(&mut app);
    next_competition::install(&mut app);
    let defaults = DefaultPlugins.set(AssetPlugin {
        file_path: concat!(env!("CARGO_MANIFEST_DIR"), "/assets").into(),
        ..default()
    });
    if std::env::var("TERRA_HEADLESS").as_deref() == Ok("1") {
        app.add_plugins(
            defaults
                .disable::<bevy::winit::WinitPlugin>()
                .set(WindowPlugin {
                    primary_window: None,
                    exit_condition: bevy::window::ExitCondition::DontExit,
                    ..default()
                }),
        );
        app.add_plugins(bevy::app::ScheduleRunnerPlugin::run_loop(
            std::time::Duration::from_millis(5),
        ));
    } else {
        app.add_plugins(defaults);
    }

    app.insert_resource(terra::RoverFleet::from_env().expect("invalid fleet configuration"))
        .add_plugins((
            TerraPhysicsPlugin,
            TerraWorldPlugin {
                config: world::WorldConfig::from_env().expect("invalid Terra world configuration"),
            },
            TerraPlugin::default(),
            velocity_controller::TerraVelocityControlPlugin,
            TerraDepthCameraPlugin::default(),
            occupancy_map::TerraOccupancyMapPlugin,
            rgb_camera::TerraRgbCameraPlugin,
            zenoh_bridge::TerraZenohPlugin {
                config: zenoh_bridge::ZenohBridgeConfig::from_env(),
            },
        ))
        .run();
}
