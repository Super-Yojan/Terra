//! Seeded roads, town blocks, procedural forests and a small animated pond.
use crate::world::{WorldConfig, WorldElement};
use avian3d::prelude::*;
use bevy::{
    asset::RenderAssetUsages,
    prelude::*,
    render::{
        RenderApp,
        render_resource::{Extent3d, TextureDimension, TextureFormat},
    },
};
use bevy_procedural_tree::{
    meshgen::generate_tree_meshes,
    settings::{BranchRecursionLevel, TreeMeshSettings},
};
use bevy_water::{
    WaterPlugin, WaterQuality, WaterSettings, WaterTile, WaveDirection,
    water::material::{StandardWaterMaterial, WaterMaterial},
};
use bevytiles::{
    config::{MIN_ZOOM, TerrainAnchor, WorldConfig as TileWorldConfig},
    height::{HeightGrid, HeightGrids, ground_height},
    lod::TileKey,
};
use fastrand::Rng;

#[derive(Clone)]
pub struct LandscapeConfig {
    pub enabled: bool,
    pub seed: u64,
    pub road_width: f32,
    pub tree_count: usize,
    pub pond: bool,
    pub voxel_hills: bool,
}

impl Default for LandscapeConfig {
    fn default() -> Self {
        Self {
            enabled: true,
            seed: 42,
            road_width: 6.0,
            tree_count: 96,
            pond: true,
            voxel_hills: true,
        }
    }
}

impl LandscapeConfig {
    pub fn validate(&self, size: f32) -> Result<(), &'static str> {
        if !self.enabled {
            return Ok(());
        }
        if !size.is_finite() || !(40.0..=512.0).contains(&size) {
            return Err("landscape worlds must be 40 to 512 metres wide");
        }
        if !self.road_width.is_finite() || self.road_width < 2.0 || self.road_width > size * 0.12 {
            return Err("road width must be at least 2 metres and at most 12% of the world width");
        }
        if self.tree_count > 1000 {
            return Err("forest is limited to 1000 trees");
        }
        Ok(())
    }
}

/// Offline local tiles in bevytiles' coordinate system; no network or GIS origin required.
/// The flat tile elevations match the Avian ground and the planar rover controller.
#[derive(Resource)]
pub struct LocalTerrain {
    grids: HeightGrids,
    topology: TileWorldConfig,
    anchor: TerrainAnchor,
}

impl LocalTerrain {
    fn new(size: f32) -> Self {
        let topology = TileWorldConfig {
            base_zoom: MIN_ZOOM,
            max_zoom: MIN_ZOOM,
            tile_size: size / 4.0,
            ..default()
        };
        let anchor = TerrainAnchor {
            world_offset: Vec3::new(-size / 2.0, 0.0, -size / 2.0),
        };
        let mut grids = HeightGrids::default();
        for x in 0..4 {
            for z in 0..4 {
                grids.0.insert(
                    TileKey {
                        zoom: MIN_ZOOM,
                        x,
                        z,
                    },
                    HeightGrid {
                        w: 2,
                        h: 2,
                        samples: vec![32768; 4],
                    },
                );
            }
        }
        Self {
            grids,
            topology,
            anchor,
        }
    }
    pub fn height_at(&self, position: Vec3) -> Option<f32> {
        ground_height(&self.grids, &self.topology, &self.anchor, position)
    }
}

pub(crate) fn install(app: &mut App, config: &WorldConfig) {
    app.insert_resource(LocalTerrain::new(config.size))
        .init_resource::<Assets<Image>>();
    if config.landscape.enabled {
        app.init_resource::<Assets<StandardWaterMaterial>>()
            .add_systems(Startup, generate_landscape);
        if app.get_sub_app(RenderApp).is_some() {
            if config.landscape.pond {
                app.insert_resource(WaterSettings {
                    spawn_tiles: None,
                    amplitude: 0.025,
                    alpha_mode: AlphaMode::Opaque,
                    deep_color: Color::srgb(0.03, 0.23, 0.29),
                    update_materials: false,
                    water_quality: WaterQuality::High,
                    ..default()
                })
                .add_plugins(WaterPlugin);
            }
            if config.landscape.voxel_hills {
                app.add_plugins(bevy_voxel_world::prelude::VoxelWorldPlugin::with_config(
                    crate::voxel_terrain::TerraVoxelTerrain {
                        size: config.size,
                        seed: config.landscape.seed,
                    },
                ));
            }
        }
    }
}

#[derive(Component)]
pub struct Road;
#[derive(Component)]
pub struct Building;
#[derive(Component)]
pub struct ForestTree;
#[derive(Component)]
pub struct Pond;

struct BuildingLot {
    center: Vec2,
    size: Vec3,
}
impl BuildingLot {
    fn contains(&self, point: Vec2, margin: f32) -> bool {
        let delta = (point - self.center).abs();
        delta.x < self.size.x / 2.0 + margin && delta.y < self.size.z / 2.0 + margin
    }
}
struct Layout {
    buildings: Vec<BuildingLot>,
    tree_positions: Vec<Vec2>,
    pond_center: Vec2,
    pond_size: f32,
}

fn layout(config: &LandscapeConfig, size: f32) -> Layout {
    let mut rng = Rng::with_seed(config.seed);
    let mut buildings = Vec::new();
    for side in [-1.0, 1.0] {
        for z in [-0.36, -0.25, -0.14, 0.14, 0.25, 0.36] {
            buildings.push(BuildingLot {
                center: Vec2::new(side * size * 0.13, size * z),
                size: Vec3::new(size * 0.08, 3.0 + rng.f32() * 7.0, size * 0.07),
            });
        }
    }
    let pond_center = Vec2::new(size * 0.31, size * 0.25);
    let pond_size = size * 0.14;
    let mut tree_positions: Vec<Vec2> = Vec::new();
    // Bound rejection sampling so high density cannot hang generation.
    for _ in 0..config.tree_count * 200 {
        if tree_positions.len() == config.tree_count {
            break;
        }
        let side = if rng.bool() { 1.0 } else { -1.0 };
        let point = Vec2::new(
            side * size * (0.22 + rng.f32() * 0.18),
            (rng.f32() - 0.5) * size * 0.78,
        );
        if point.x.abs() <= config.road_width / 2.0 + 2.0
            || point.y.abs() <= config.road_width / 2.0 + 2.0
            || buildings
                .iter()
                .any(|building| building.contains(point, 2.0))
            || (config.pond && (point - pond_center).abs().max_element() <= pond_size / 2.0 + 2.0)
            || tree_positions
                .iter()
                .any(|other| other.distance_squared(point) < 3.0_f32.powi(2))
        {
            continue;
        }
        tree_positions.push(point);
    }
    Layout {
        buildings,
        tree_positions,
        pond_center,
        pond_size,
    }
}

struct SceneAssets {
    cube: Handle<Mesh>,
    asphalt: Handle<StandardMaterial>,
    stripe: Handle<StandardMaterial>,
    concrete: Handle<StandardMaterial>,
    roof: Handle<StandardMaterial>,
    glass: Handle<StandardMaterial>,
    facades: Vec<Handle<StandardMaterial>>,
}

fn box_visual(
    commands: &mut Commands,
    assets: &SceneAssets,
    name: &str,
    material: &Handle<StandardMaterial>,
    position: Vec3,
    size: Vec3,
) -> Entity {
    commands
        .spawn((
            Name::new(name.to_owned()),
            WorldElement,
            Mesh3d(assets.cube.clone()),
            MeshMaterial3d(material.clone()),
            Transform::from_translation(position).with_scale(size),
        ))
        .id()
}

fn generate_landscape(
    mut commands: Commands,
    config: Res<WorldConfig>,
    terrain: Res<LocalTerrain>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mut water_materials: ResMut<Assets<StandardWaterMaterial>>,
    mut images: ResMut<Assets<Image>>,
) {
    let material = |materials: &mut Assets<StandardMaterial>, color| {
        materials.add(StandardMaterial {
            base_color: color,
            perceptual_roughness: 0.9,
            ..default()
        })
    };
    let assets = SceneAssets {
        cube: meshes.add(Cuboid::new(1.0, 1.0, 1.0)),
        asphalt: material(&mut materials, Color::srgb(0.09, 0.11, 0.12)),
        stripe: material(&mut materials, Color::srgb(0.93, 0.86, 0.59)),
        concrete: material(&mut materials, Color::srgb(0.55, 0.56, 0.53)),
        roof: material(&mut materials, Color::srgb(0.23, 0.26, 0.28)),
        glass: materials.add(StandardMaterial {
            base_color: Color::srgb(0.15, 0.30, 0.37),
            metallic: 0.25,
            perceptual_roughness: 0.2,
            ..default()
        }),
        facades: [
            Color::srgb(0.69, 0.52, 0.39),
            Color::srgb(0.76, 0.72, 0.61),
            Color::srgb(0.48, 0.55, 0.55),
            Color::srgb(0.68, 0.43, 0.34),
        ]
        .map(|color| material(&mut materials, color))
        .to_vec(),
    };
    let generated = layout(&config.landscape, config.size);
    spawn_roads(&mut commands, &assets, &config);
    for (index, building) in generated.buildings.iter().enumerate() {
        let base = Vec3::new(building.center.x, 0.0, building.center.y);
        let height = terrain
            .height_at(base)
            .expect("building outside local terrain");
        let base = base.with_y(height);
        let body = box_visual(
            &mut commands,
            &assets,
            "Town building",
            &assets.facades[index % assets.facades.len()],
            base + Vec3::Y * building.size.y / 2.0,
            building.size,
        );
        commands.entity(body).insert((
            Building,
            RigidBody::Static,
            Collider::cuboid(1.0, 1.0, 1.0),
            Restitution::ZERO,
        ));
        box_visual(
            &mut commands,
            &assets,
            "Building roof",
            &assets.roof,
            base + Vec3::Y * (building.size.y + 0.12),
            Vec3::new(building.size.x + 0.3, 0.24, building.size.z + 0.3),
        );
        let floors = (building.size.y / 2.5).floor() as usize;
        for floor in 0..floors {
            for side in [-1.0, 1.0] {
                for column in [-0.27, 0.0, 0.27] {
                    box_visual(
                        &mut commands,
                        &assets,
                        "Window",
                        &assets.glass,
                        base + Vec3::new(
                            column * building.size.x,
                            1.5 + floor as f32 * 2.5,
                            side * (building.size.z / 2.0 + 0.01),
                        ),
                        Vec3::new(0.85, 1.0, 0.03),
                    );
                    box_visual(
                        &mut commands,
                        &assets,
                        "Window",
                        &assets.glass,
                        base + Vec3::new(
                            side * (building.size.x / 2.0 + 0.01),
                            1.5 + floor as f32 * 2.5,
                            column * building.size.z,
                        ),
                        Vec3::new(0.03, 1.0, 0.85),
                    );
                }
            }
        }
        // Door faces the central north/south road.
        box_visual(
            &mut commands,
            &assets,
            "Building door",
            &assets.roof,
            base + Vec3::new(
                -building.center.x.signum() * (building.size.x / 2.0 + 0.02),
                1.0,
                0.0,
            ),
            Vec3::new(0.05, 2.0, 1.0),
        );
    }
    spawn_forest(
        &mut commands,
        &config,
        &generated,
        &terrain,
        &mut meshes,
        &mut materials,
        &mut images,
    );
    if config.landscape.pond {
        spawn_pond(
            &mut commands,
            &assets,
            &generated,
            &mut meshes,
            &mut water_materials,
        );
    }
}

fn spawn_roads(commands: &mut Commands, assets: &SceneAssets, config: &WorldConfig) {
    let width = config.landscape.road_width;
    let length = config.size * 0.82;
    for size in [
        Vec3::new(width, 0.002, length),
        Vec3::new(length, 0.002, width),
    ] {
        let road = box_visual(
            commands,
            assets,
            "Road",
            &assets.asphalt,
            Vec3::Y * 0.001,
            size,
        );
        commands.entity(road).insert(Road);
    }
    let dash_count = (length / 8.0).floor() as i32;
    for index in -dash_count..=dash_count {
        let along = index as f32 * 4.0;
        if along.abs() < width || along.abs() + 1.0 > length / 2.0 {
            continue;
        }
        for (position, size) in [
            (Vec3::new(0.0, 0.004, along), Vec3::new(0.12, 0.002, 2.0)),
            (Vec3::new(along, 0.004, 0.0), Vec3::new(2.0, 0.002, 0.12)),
        ] {
            box_visual(
                commands,
                assets,
                "Lane marking",
                &assets.stripe,
                position,
                size,
            );
        }
    }
    // Sidewalks stop at the intersection and remain flush enough for the rover.
    let walk_length = (length - width) / 2.0;
    for side in [-1.0, 1.0] {
        for end in [-1.0, 1.0] {
            let edge = side * (width / 2.0 + 0.6);
            let along = end * (width / 2.0 + walk_length / 2.0);
            for (position, size) in [
                (
                    Vec3::new(edge, 0.003, along),
                    Vec3::new(1.2, 0.006, walk_length),
                ),
                (
                    Vec3::new(along, 0.003, edge),
                    Vec3::new(walk_length, 0.006, 1.2),
                ),
            ] {
                box_visual(
                    commands,
                    assets,
                    "Sidewalk",
                    &assets.concrete,
                    position,
                    size,
                );
            }
        }
    }
    for side in [-1.0, 1.0] {
        for stripe in -2..=2 {
            let across = stripe as f32 * width / 6.0;
            for (position, size) in [
                (
                    Vec3::new(across, 0.006, side * (width / 2.0 + 1.3)),
                    Vec3::new(0.4, 0.002, 1.8),
                ),
                (
                    Vec3::new(side * (width / 2.0 + 1.3), 0.006, across),
                    Vec3::new(1.8, 0.002, 0.4),
                ),
            ] {
                box_visual(
                    commands,
                    assets,
                    "Crosswalk",
                    &assets.stripe,
                    position,
                    size,
                );
            }
        }
    }
}

fn spawn_forest(
    commands: &mut Commands,
    config: &WorldConfig,
    generated: &Layout,
    terrain: &LocalTerrain,
    meshes: &mut Assets<Mesh>,
    materials: &mut Assets<StandardMaterial>,
    images: &mut Assets<Image>,
) {
    if generated.tree_positions.is_empty() {
        return;
    }
    let bark = materials.add(StandardMaterial {
        base_color: Color::srgb(0.25, 0.16, 0.09),
        perceptual_roughness: 1.0,
        ..default()
    });
    let mut leaf_pixels = Vec::with_capacity(32 * 32 * 4);
    for y in 0..32 {
        for x in 0..32 {
            let dx = (x as f32 - 15.5) / 14.5;
            let dy = (y as f32 - 15.5) / 15.5;
            let alpha = if dx * dx + dy * dy < 1.0 { 255 } else { 0 };
            let green = if dx.abs() < 0.07 { 210 } else { 255 };
            leaf_pixels.extend_from_slice(&[green, 255, green, alpha]);
        }
    }
    let leaf_texture = images.add(Image::new(
        Extent3d {
            width: 32,
            height: 32,
            depth_or_array_layers: 1,
        },
        TextureDimension::D2,
        leaf_pixels,
        TextureFormat::Rgba8UnormSrgb,
        RenderAssetUsages::default(),
    ));
    let leaves = materials.add(StandardMaterial {
        base_color_texture: Some(leaf_texture),
        alpha_mode: AlphaMode::Mask(0.5),
        base_color: Color::srgb(0.13, 0.32, 0.10),
        perceptual_roughness: 1.0,
        double_sided: true,
        cull_mode: None,
        ..default()
    });
    let mut settings = TreeMeshSettings::default();
    settings.branch.levels = BranchRecursionLevel::Two;
    settings.branch.children = [5, 4, 5];
    settings.branch.sections = [6, 4, 3, 2];
    settings.branch.segments = [6, 5, 4, 3];
    settings.leaves.size = 0.55;
    settings.leaves.count = 5;
    // Reuse eight generated mesh pairs across the forest, rather than rebuilding every tree.
    let mut variants = Vec::new();
    for variant in 0..8 {
        let mut rng = Rng::with_seed(config.landscape.seed.wrapping_add(variant));
        let (branches, foliage) =
            generate_tree_meshes(&settings, &mut rng).expect("valid tree settings");
        variants.push((meshes.add(branches), meshes.add(foliage)));
    }
    let mut rng = Rng::with_seed(config.landscape.seed ^ 0xF012E57);
    for (index, point) in generated.tree_positions.iter().enumerate() {
        let position = Vec3::new(point.x, 0.0, point.y);
        let position = position.with_y(
            terrain
                .height_at(position)
                .expect("tree outside local terrain"),
        );
        let scale = 0.75 + rng.f32() * 0.45;
        let (branches, foliage) = &variants[index % variants.len()];
        let trunk_height = settings.branch.length[0];
        commands
            .spawn((
                Name::new("Forest tree"),
                WorldElement,
                ForestTree,
                Mesh3d(branches.clone()),
                MeshMaterial3d(bark.clone()),
                Transform::from_translation(position)
                    .with_scale(Vec3::splat(scale))
                    .with_rotation(Quat::from_rotation_y(rng.f32() * std::f32::consts::TAU)),
                RigidBody::Static,
                Restitution::ZERO,
                Collider::compound(vec![(
                    Vec3::Y * trunk_height / 2.0,
                    Quat::IDENTITY,
                    Collider::cylinder(settings.branch.trunk_base_radius, trunk_height),
                )]),
            ))
            .with_children(|parent| {
                parent.spawn((
                    Name::new("Tree foliage"),
                    WorldElement,
                    Mesh3d(foliage.clone()),
                    MeshMaterial3d(leaves.clone()),
                ));
            });
    }
}

fn spawn_pond(
    commands: &mut Commands,
    assets: &SceneAssets,
    generated: &Layout,
    meshes: &mut Assets<Mesh>,
    materials: &mut Assets<StandardWaterMaterial>,
) {
    let size = generated.pond_size;
    let center = Vec3::new(generated.pond_center.x, 0.06, generated.pond_center.y);
    let material = materials.add(StandardWaterMaterial {
        base: StandardMaterial {
            base_color: Color::WHITE,
            alpha_mode: AlphaMode::Opaque,
            perceptual_roughness: 0.22,
            ..default()
        },
        extension: WaterMaterial {
            amplitude: 0.025,
            quality: 3,
            coord_scale: Vec2::splat(size),
            coord_offset: generated.pond_center - Vec2::splat(size / 2.0),
            deep_color: Color::srgb(0.03, 0.23, 0.29),
            ..default()
        },
    });
    commands.spawn((
        Name::new("Forest pond"),
        WorldElement,
        Pond,
        WaterTile::default(),
        WaveDirection::default(),
        Mesh3d(meshes.add(Plane3d::default().mesh().size(size, size).subdivisions(24))),
        MeshMaterial3d(material),
        Transform::from_translation(center),
    ));
    // Low stone banks are physical barriers; the pond is visual, with no buoyancy.
    for (offset, dimensions) in [
        (
            Vec3::new(-size / 2.0, 0.0, 0.0),
            Vec3::new(0.4, 0.3, size + 0.4),
        ),
        (
            Vec3::new(size / 2.0, 0.0, 0.0),
            Vec3::new(0.4, 0.3, size + 0.4),
        ),
        (Vec3::new(0.0, 0.0, -size / 2.0), Vec3::new(size, 0.3, 0.4)),
        (Vec3::new(0.0, 0.0, size / 2.0), Vec3::new(size, 0.3, 0.4)),
    ] {
        let bank = box_visual(
            commands,
            assets,
            "Pond bank",
            &assets.concrete,
            center.with_y(0.15) + offset,
            dimensions,
        );
        commands.entity(bank).insert((
            RigidBody::Static,
            Collider::cuboid(1.0, 1.0, 1.0),
            Restitution::ZERO,
        ));
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::world::{TerraWorldPlugin, WorldConfig};
    fn generated(config: LandscapeConfig) -> App {
        let mut app = App::new();
        app.insert_resource(Assets::<Mesh>::default())
            .insert_resource(Assets::<StandardMaterial>::default())
            .insert_resource(Time::<()>::default())
            .add_plugins(TerraWorldPlugin {
                config: WorldConfig {
                    landscape: config,
                    ..default()
                },
            });
        app.update();
        app
    }
    #[test]
    #[ignore = "requires a GPU; verifies voxel, tree and water rendering together"]
    fn gpu_renders_landscape_and_depth() {
        use crate::{
            depth_camera::{DepthCameraConfig, DepthFrame, TerraDepthCameraPlugin},
            physics::RoverPhysicsConfig,
            terra::Rover,
            voxel_terrain::VoxelTerrainChunk,
            world::WorldCamera,
        };
        use bevy::{
            app::PluginsState,
            asset::AssetPlugin,
            camera::RenderTarget,
            render::gpu_readback::{Readback, ReadbackComplete},
            window::ExitCondition,
            winit::WinitPlugin,
        };
        #[derive(Resource, Default)]
        struct Capture(Vec<u8>);
        let mut app = App::new();
        app.add_plugins(
            DefaultPlugins
                .build()
                .disable::<WinitPlugin>()
                .set(WindowPlugin {
                    primary_window: None,
                    exit_condition: ExitCondition::DontExit,
                    ..default()
                })
                .set(AssetPlugin {
                    file_path: concat!(env!("CARGO_MANIFEST_DIR"), "/assets").into(),
                    ..default()
                }),
        )
        .insert_resource(RoverPhysicsConfig::default())
        .init_resource::<Capture>()
        .add_plugins((
            TerraWorldPlugin {
                config: WorldConfig {
                    camera_offset: Vec3::new(62.0, 80.0, 62.0),
                    follow_rover: false,
                    ..default()
                },
            },
            TerraDepthCameraPlugin {
                config: DepthCameraConfig {
                    preview: false,
                    ..default()
                },
            },
            crate::rgb_camera::TerraRgbCameraPlugin,
        ));
        let deadline = std::time::Instant::now() + std::time::Duration::from_secs(45);
        while app.plugins_state() == PluginsState::Adding {
            assert!(
                std::time::Instant::now() < deadline,
                "renderer init timed out"
            );
            bevy::tasks::tick_global_task_pools_on_main_thread();
            std::thread::sleep(std::time::Duration::from_millis(5));
        }
        app.finish();
        app.cleanup();
        app.world_mut().spawn((
            Rover,
            crate::terra::RoverId(0),
            crate::terra::RoverSlot(0),
            Transform::from_xyz(0.0, 0.1, 0.0),
        ));
        let extra_rover = app
            .world_mut()
            .spawn((
                Rover,
                crate::terra::RoverId(1),
                crate::terra::RoverSlot(1),
                Transform::from_xyz(2.0, 0.1, 0.0),
            ))
            .id();
        app.update();
        let image = Image::new_target_texture(640, 360, TextureFormat::Rgba8UnormSrgb, None);
        let mut image = image;
        image.texture_descriptor.usage |= bevy::render::render_resource::TextureUsages::COPY_SRC;
        let image = app.world_mut().resource_mut::<Assets<Image>>().add(image);
        let world = app.world_mut();
        let camera = world
            .query_filtered::<Entity, With<WorldCamera>>()
            .single(world)
            .unwrap();
        world
            .entity_mut(camera)
            .insert((RenderTarget::Image(image.clone().into()), Msaa::Off));
        let reader = world
            .spawn(Readback::texture(image))
            .observe(
                |event: On<ReadbackComplete>, mut capture: ResMut<Capture>| {
                    capture.0 = event.data.clone();
                },
            )
            .id();
        let mut ready = false;
        let mut frames = 0;
        let mut last_state = String::new();
        while std::time::Instant::now() < deadline {
            app.update();
            frames += 1;
            let world = app.world_mut();
            let chunks = world.query::<&VoxelTerrainChunk>().iter(world).count();
            let depth_frames: Vec<_> = world.query::<&DepthFrame>().iter(world).collect();
            let valid_depth = depth_frames.len() == 2
                && depth_frames
                    .iter()
                    .all(|frame| frame.depth_metres.iter().any(|value| value.is_finite()));
            let rgb_frames: Vec<_> = world
                .query::<&crate::rgb_camera::RgbFrame>()
                .iter(world)
                .collect();
            let valid_rgb = rgb_frames.len() == 2
                && rgb_frames.iter().all(|frame| {
                    frame.sequence > 0
                        && frame.rgba.len() == 256 * 192 * 4
                        && frame
                            .rgba
                            .as_chunks::<4>()
                            .0
                            .iter()
                            .any(|pixel| pixel[0] > 0 || pixel[1] > 0 || pixel[2] > 0)
                });
            let capture = &world.resource::<Capture>().0;
            last_state = format!(
                "frames={frames}, voxel colliders={chunks}, valid depth={valid_depth}, valid RGB={valid_rgb}, color bytes={}",
                capture.len()
            );
            if frames > 100
                && chunks > 0
                && valid_depth
                && valid_rgb
                && capture.len() == 640 * 360 * 4
            {
                ready = true;
                break;
            }
            std::thread::sleep(std::time::Duration::from_millis(10));
        }
        assert!(ready, "landscape was not ready: {last_state}");
        let bytes = app.world().resource::<Capture>().0.clone();
        Image::new(
            Extent3d {
                width: 640,
                height: 360,
                depth_or_array_layers: 1,
            },
            TextureDimension::D2,
            bytes,
            TextureFormat::Rgba8UnormSrgb,
            RenderAssetUsages::default(),
        )
        .try_into_dynamic()
        .unwrap()
        .save("/tmp/terra-world-preview.png")
        .unwrap();
        let world = app.world_mut();
        let sensors: Vec<_> = world
            .query_filtered::<Entity, With<Readback>>()
            .iter(world)
            .collect();
        world.entity_mut(reader).remove::<Readback>();
        for sensor in sensors {
            world
                .entity_mut(sensor)
                .remove::<crate::depth_camera::DepthCapture>();
        }
        for _ in 0..20 {
            app.update();
            std::thread::sleep(std::time::Duration::from_millis(5));
        }
        app.world_mut().entity_mut(extra_rover).despawn();
        app.update();
        let world = app.world_mut();
        assert_eq!(world.query::<&DepthFrame>().iter(world).count(), 1);
        assert_eq!(
            world
                .query::<&crate::rgb_camera::RgbFrame>()
                .iter(world)
                .count(),
            1
        );
    }

    #[test]
    fn creates_connected_roads_buildings_forest_and_pond() {
        let mut app = generated(LandscapeConfig::default());
        let world = app.world_mut();
        assert_eq!(
            world
                .query_filtered::<Entity, With<Road>>()
                .iter(world)
                .count(),
            2
        );
        assert_eq!(
            world
                .query_filtered::<Entity, With<Building>>()
                .iter(world)
                .count(),
            12
        );
        assert!(
            world
                .query_filtered::<Entity, With<ForestTree>>()
                .iter(world)
                .count()
                > 40
        );
        assert_eq!(
            world
                .query_filtered::<Entity, With<Pond>>()
                .iter(world)
                .count(),
            1
        );
        assert_eq!(
            world
                .query_filtered::<Entity, (With<Building>, With<Collider>)>()
                .iter(world)
                .count(),
            12
        );
        let elevations = world.resource::<LocalTerrain>();
        assert_eq!(elevations.height_at(Vec3::ZERO), Some(0.0));
    }
    #[test]
    fn layout_is_seeded_and_keeps_roads_and_spawn_clear() {
        let config = LandscapeConfig::default();
        let a = layout(&config, 100.0);
        let b = layout(&config, 100.0);
        assert_eq!(a.tree_positions, b.tree_positions);
        assert!(!a.tree_positions.is_empty());
        for position in &a.tree_positions {
            assert!(position.x.abs() > config.road_width / 2.0 + 2.0);
            assert!(position.y.abs() > config.road_width / 2.0 + 2.0);
            assert!(position.length() > 5.0);
            assert!(
                !a.buildings
                    .iter()
                    .any(|building| building.contains(*position, 2.0))
            );
            assert!((*position - a.pond_center).abs().max_element() > a.pond_size / 2.0 + 2.0);
        }
        let c = layout(
            &LandscapeConfig {
                seed: config.seed + 1,
                ..config
            },
            100.0,
        );
        assert_ne!(a.tree_positions, c.tree_positions);
    }
    #[test]
    fn content_can_be_disabled_and_small_maps_are_validated() {
        let mut app = generated(LandscapeConfig {
            enabled: false,
            ..default()
        });
        let world = app.world_mut();
        assert_eq!(
            world
                .query_filtered::<Entity, With<Building>>()
                .iter(world)
                .count(),
            0
        );
        assert!(
            LandscapeConfig {
                road_width: -1.0,
                ..default()
            }
            .validate(100.0)
            .is_err()
        );
        assert!(
            LandscapeConfig {
                tree_count: 10001,
                ..default()
            }
            .validate(100.0)
            .is_err()
        );
    }
}
