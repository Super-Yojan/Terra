mod depth_camera;
mod landscape;
mod physics;
mod rgb_camera;
mod terra;
mod voxel_terrain;
mod world;
mod zenoh_bridge;

use bevy::prelude::*;
use depth_camera::TerraDepthCameraPlugin;
use physics::TerraPhysicsPlugin;
use terra::TerraPlugin;
use world::TerraWorldPlugin;

fn main() {
    App::new()
        .insert_resource(terra::RoverFleet::from_env().expect("invalid fleet configuration"))
        .add_plugins((
            DefaultPlugins,
            TerraPhysicsPlugin,
            TerraWorldPlugin::default(),
            TerraPlugin::default(),
            TerraDepthCameraPlugin::default(),
            rgb_camera::TerraRgbCameraPlugin,
            zenoh_bridge::TerraZenohPlugin {
                config: zenoh_bridge::ZenohBridgeConfig::from_env(),
            },
        ))
        .run();
}
