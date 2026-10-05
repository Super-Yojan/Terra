//! Front-mounted rendered depth sensor. Bevy reverse-Z is converted to axial
//! depth in metres; no return / outside range is NaN. Readback is asynchronous.
use crate::{physics::RoverPhysicsConfig, terra::Rover};
use bevy::{
    asset::RenderAssetUsages,
    camera::RenderTarget,
    core_pipeline::{
        prepass::DepthPrepass,
        schedule::{Core3d, Core3dSystems},
    },
    image::ImageSampler,
    prelude::*,
    render::{
        RenderApp,
        extract_component::{ExtractComponent, ExtractComponentPlugin},
        render_asset::RenderAssets,
        render_resource::{
            Buffer, BufferDescriptor, BufferUsages, Extent3d, MapMode, Origin3d,
            TexelCopyBufferInfo, TexelCopyBufferLayout, TexelCopyTextureInfo, TextureAspect,
            TextureDimension, TextureFormat, TextureUsages,
        },
        renderer::{RenderContext, RenderDevice, ViewQuery},
        texture::GpuImage,
        view::ViewDepthTexture,
    },
    transform::TransformSystems,
};
use std::sync::{
    Arc, Mutex,
    atomic::{AtomicUsize, Ordering},
};

#[derive(Default)]
pub struct TerraDepthCameraPlugin {
    pub config: DepthCameraConfig,
}

impl Plugin for TerraDepthCameraPlugin {
    fn build(&self, app: &mut App) {
        self.config
            .validate()
            .expect("invalid depth camera configuration");
        app.insert_resource(self.config.clone())
            .add_systems(
                Update,
                (mount_cameras, remove_orphan_previews, receive_depth),
            )
            .init_resource::<CaptureQueue>()
            .add_systems(
                PostUpdate,
                stamp_exposures.after(TransformSystems::Propagate),
            );
        if app.get_sub_app(RenderApp).is_some() {
            app.add_plugins((
                ExtractComponentPlugin::<DepthCamera>::default(),
                ExtractComponentPlugin::<DepthExposure>::default(),
                ExtractComponentPlugin::<DepthCapture>::default(),
            ));
            let queue = app.world().resource::<CaptureQueue>().clone();
            app.get_sub_app_mut(RenderApp)
                .unwrap()
                .insert_resource(queue)
                .init_resource::<PendingCaptures>()
                .add_systems(
                    bevy::render::Render,
                    map_captures.in_set(bevy::render::RenderSystems::Cleanup),
                );
            app.get_sub_app_mut(RenderApp).unwrap().add_systems(
                Core3d,
                copy_camera_depth
                    .after(Core3dSystems::Prepass)
                    .before(Core3dSystems::MainPass),
            );
        }
    }
}

#[derive(Resource, Clone)]
pub struct DepthCameraConfig {
    pub resolution: UVec2,
    /// Vertical field of view in radians.
    pub vertical_fov: f32,
    pub near: f32,
    pub far: f32,
    /// Mount height above the rover body origin, in metres.
    pub mount_height: f32,
    /// Distance ahead of the front face of the chassis, in metres.
    pub front_clearance: f32,
    pub preview: bool,
}

impl Default for DepthCameraConfig {
    fn default() -> Self {
        Self {
            resolution: UVec2::new(256, 192),
            vertical_fov: 60.0_f32.to_radians(),
            near: 0.05,
            far: 30.0,
            mount_height: 0.2,
            front_clearance: 0.05,
            preview: true,
        }
    }
}

impl DepthCameraConfig {
    pub fn validate(&self) -> Result<(), &'static str> {
        if self.resolution.min_element() == 0 || self.resolution.max_element() > 2048 {
            return Err("depth resolution must be between 1 and 2048 pixels per axis");
        }
        if !self.vertical_fov.is_finite()
            || self.vertical_fov <= 0.0
            || self.vertical_fov >= std::f32::consts::PI
        {
            return Err("vertical field of view must lie between zero and pi radians");
        }
        if !self.near.is_finite()
            || self.near <= 0.0
            || !self.far.is_finite()
            || self.far <= self.near
        {
            return Err("depth range must be finite with 0 < near < far");
        }
        if !self.mount_height.is_finite()
            || self.mount_height < 0.0
            || !self.front_clearance.is_finite()
            || self.front_clearance <= 0.0
        {
            return Err("mount height must be nonnegative and front clearance positive");
        }
        Ok(())
    }
}

#[derive(Component, Clone, ExtractComponent)]
pub struct DepthCamera {
    pub config: DepthCameraConfig,
    /// Raw GPU reverse-Z depth texture. Use DepthFrame for distances in metres.
    pub depth_texture: Handle<Image>,
    pub preview_texture: Handle<Image>,
}

/// Latest completed GPU capture, row-major from top left. Axial depth in metres
/// (camera-space -Z), not Euclidean ray length. NaN indicates an invalid return.
#[derive(Component, Default)]
pub struct DepthFrame {
    pub width: u32,
    pub height: u32,
    pub depth_metres: Vec<f32>,
    pub sequence: u64,
    /// CPU receipt time in Bevy elapsed seconds, not exposure time.
    pub received_at: f64,
    /// Simulation exposure time, optical-camera pose, and rover body pose, paired with this GPU copy.
    pub exposure: Option<DepthExposure>,
}

#[derive(Component, Clone, ExtractComponent)]
pub(crate) struct DepthCapture;

#[derive(Component, Clone, Copy, ExtractComponent)]
pub struct DepthExposure {
    pub sensor: Entity,
    pub timestamp: f64,
    /// Optical camera pose in Bevy world at `timestamp`.
    pub camera_transform: GlobalTransform,
    /// Rover body pose in Bevy world at `timestamp`. Falls back to the camera when unparented.
    pub body_transform: GlobalTransform,
}
struct CompletedCapture {
    exposure: DepthExposure,
    data: Vec<u8>,
}
#[derive(Resource, Clone, Default)]
pub(crate) struct CaptureQueue(Arc<Mutex<Vec<CompletedCapture>>>, Arc<AtomicUsize>);
#[derive(Resource, Default)]
struct PendingCaptures(Vec<(Buffer, DepthExposure)>);
fn stamp_exposures(
    mut commands: Commands,
    time: Res<Time>,
    sensors: Query<(Entity, &GlobalTransform, Option<&ChildOf>), With<DepthCamera>>,
    bodies: Query<&GlobalTransform, With<Rover>>,
) {
    for (sensor, transform, parent) in &sensors {
        let body_transform = parent
            .and_then(|child| bodies.get(child.parent()).ok())
            .copied()
            .unwrap_or(*transform);
        commands.entity(sensor).insert(DepthExposure {
            sensor,
            timestamp: time.elapsed_secs_f64(),
            camera_transform: *transform,
            body_transform,
        });
    }
}
fn map_captures(mut pending: ResMut<PendingCaptures>, queue: Res<CaptureQueue>) {
    for (buffer, exposure) in pending.0.drain(..) {
        let mapped = buffer.clone();
        let in_flight = queue.1.clone();
        let queue = queue.0.clone();
        buffer.slice(..).map_async(MapMode::Read, move |result| {
            in_flight.fetch_sub(1, Ordering::Relaxed);
            if result.is_err() {
                return;
            }
            let view = mapped.slice(..).get_mapped_range();
            let data = view.to_vec();
            drop(view);
            mapped.unmap();
            if let Ok(mut completed) = queue.lock() {
                // Bound CPU storage if the main thread stalls; older frames may be dropped.
                if completed.len() < 64 {
                    completed.push(CompletedCapture { exposure, data });
                }
            }
        });
    }
}

#[derive(Component)]
struct DepthPreview(Entity);

type NewRovers<'w, 's> = Query<
    'w,
    's,
    (
        Entity,
        Option<&'static crate::terra::RoverId>,
        Option<&'static crate::terra::RoverSlot>,
    ),
    Added<Rover>,
>;

fn mount_cameras(
    mut commands: Commands,
    config: Res<DepthCameraConfig>,
    physics: Res<RoverPhysicsConfig>,
    rovers: NewRovers<'_, '_>,
    mut images: ResMut<Assets<Image>>,
) {
    for (rover, id, slot) in &rovers {
        let extent = Extent3d {
            width: config.resolution.x,
            height: config.resolution.y,
            depth_or_array_layers: 1,
        };
        let mut depth = Image::new_uninit(
            extent,
            TextureDimension::D2,
            TextureFormat::Depth32Float,
            RenderAssetUsages::RENDER_WORLD,
        );
        depth.texture_descriptor.usage =
            TextureUsages::COPY_DST | TextureUsages::COPY_SRC | TextureUsages::TEXTURE_BINDING;
        let depth_texture = images.add(depth);
        let mut preview = Image::new_fill(
            extent,
            TextureDimension::D2,
            &[0, 0, 0, 255],
            TextureFormat::Rgba8UnormSrgb,
            RenderAssetUsages::default(),
        );
        preview.sampler = ImageSampler::nearest();
        let preview_texture = images.add(preview);
        let sensor = commands
            .spawn((
                Name::new("Terra front depth camera"),
                id.copied().unwrap_or(crate::terra::RoverId(0)),
                DepthCamera {
                    config: config.clone(),
                    depth_texture: depth_texture.clone(),
                    preview_texture: preview_texture.clone(),
                },
                DepthFrame {
                    width: extent.width,
                    height: extent.height,
                    ..default()
                },
                Camera3d {
                    depth_texture_usages: (TextureUsages::RENDER_ATTACHMENT
                        | TextureUsages::COPY_SRC)
                        .into(),
                    ..default()
                },
                Camera {
                    order: -1 - slot.map_or(0, |slot| slot.0 as isize),
                    ..default()
                },
                RenderTarget::None {
                    size: config.resolution,
                },
                Projection::Perspective(PerspectiveProjection {
                    fov: config.vertical_fov,
                    near: config.near,
                    far: config.far,
                    aspect_ratio: extent.width as f32 / extent.height as f32,
                    ..default()
                }),
                Msaa::Off,
                DepthPrepass,
                Transform::from_xyz(
                    0.0,
                    config.mount_height,
                    -physics.chassis_size.z / 2.0 - config.front_clearance,
                ),
                ChildOf(rover),
                DepthCapture,
            ))
            .id();
        if config.preview && slot.is_none_or(|slot| slot.0 == 0) {
            commands
                .spawn((
                    Name::new("Front depth preview"),
                    DepthPreview(sensor),
                    Node {
                        position_type: PositionType::Absolute,
                        right: px(16.0),
                        bottom: px(16.0),
                        flex_direction: FlexDirection::Column,
                        padding: UiRect::all(px(8.0)),
                        row_gap: px(6.0),
                        ..default()
                    },
                    BackgroundColor(Color::srgba(0.02, 0.02, 0.02, 0.9)),
                ))
                .with_children(|parent| {
                    parent.spawn((
                        Text::new("Front depth · metres · near = bright"),
                        TextFont {
                            font_size: FontSize::Px(14.0),
                            ..default()
                        },
                    ));
                    parent.spawn((
                        ImageNode::new(preview_texture),
                        Node {
                            width: px(256.0),
                            height: px(256.0 * extent.height as f32 / extent.width as f32),
                            ..default()
                        },
                    ));
                });
        }
    }
}

type DepthView = (
    Option<&'static DepthCamera>,
    Option<&'static DepthExposure>,
    Option<&'static DepthCapture>,
    &'static ViewDepthTexture,
);

fn copy_camera_depth(
    view: ViewQuery<DepthView>,
    device: Res<RenderDevice>,
    queue: Res<CaptureQueue>,
    mut pending: ResMut<PendingCaptures>,
    images: Res<RenderAssets<GpuImage>>,
    mut ctx: RenderContext,
) {
    let (Some(sensor), Some(exposure), Some(_), depth) = view.into_inner() else {
        return;
    };
    let Some(target) = images.get(sensor.depth_texture.id()) else {
        return;
    };
    if queue
        .1
        .try_update(Ordering::Relaxed, Ordering::Relaxed, |n| {
            (n < 64).then_some(n + 1)
        })
        .is_err()
    {
        return;
    }
    let stride = (sensor.config.resolution.x * 4).div_ceil(256) * 256;
    let buffer = device.create_buffer(&BufferDescriptor {
        label: Some("Terra exposure-aligned depth"),
        size: u64::from(stride) * u64::from(sensor.config.resolution.y),
        usage: BufferUsages::COPY_DST | BufferUsages::MAP_READ,
        mapped_at_creation: false,
    });
    ctx.command_encoder().copy_texture_to_buffer(
        TexelCopyTextureInfo {
            texture: &depth.texture,
            mip_level: 0,
            origin: Origin3d::ZERO,
            aspect: TextureAspect::DepthOnly,
        },
        TexelCopyBufferInfo {
            buffer: &buffer,
            layout: TexelCopyBufferLayout {
                offset: 0,
                bytes_per_row: Some(stride),
                rows_per_image: None,
            },
        },
        Extent3d {
            width: sensor.config.resolution.x,
            height: sensor.config.resolution.y,
            depth_or_array_layers: 1,
        },
    );
    pending.0.push((buffer, *exposure));
    ctx.command_encoder().copy_texture_to_texture(
        TexelCopyTextureInfo {
            texture: &depth.texture,
            mip_level: 0,
            origin: Origin3d::ZERO,
            aspect: TextureAspect::DepthOnly,
        },
        TexelCopyTextureInfo {
            texture: &target.texture,
            mip_level: 0,
            origin: Origin3d::ZERO,
            aspect: TextureAspect::DepthOnly,
        },
        Extent3d {
            width: sensor.config.resolution.x,
            height: sensor.config.resolution.y,
            depth_or_array_layers: 1,
        },
    );
}

fn decode_depth(bytes: &[u8], config: &DepthCameraConfig) -> Option<Vec<f32>> {
    let width = config.resolution.x as usize;
    let height = config.resolution.y as usize;
    let stride = (width * 4).div_ceil(256) * 256;
    if bytes.len() < stride * (height - 1) + width * 4 {
        return None;
    }
    let mut result = Vec::with_capacity(width * height);
    for row in 0..height {
        for pixel in bytes[row * stride..row * stride + width * 4]
            .as_chunks::<4>()
            .0
        {
            let raw = f32::from_le_bytes(*pixel);
            let metres = config.near / raw;
            result.push(
                if raw > 0.0 && raw <= 1.0 && metres.is_finite() && metres <= config.far {
                    metres
                } else {
                    f32::NAN
                },
            );
        }
    }
    Some(result)
}

pub(crate) fn receive_depth(
    queue: Res<CaptureQueue>,
    time: Res<Time>,
    mut sensors: Query<(&DepthCamera, &mut DepthFrame)>,
    mut images: ResMut<Assets<Image>>,
) {
    let mut completed = queue.0.lock().expect("depth capture queue poisoned");
    let mut captures = std::mem::take(&mut *completed);
    drop(completed);
    captures.sort_by(|a, b| a.exposure.timestamp.total_cmp(&b.exposure.timestamp));
    for capture in captures {
        let Ok((sensor, mut frame)) = sensors.get_mut(capture.exposure.sensor) else {
            continue;
        };
        let Some(depth) = decode_depth(&capture.data, &sensor.config) else {
            continue;
        };
        if depth.len() != frame.width as usize * frame.height as usize {
            continue;
        }
        if let Some(mut image) = images.get_mut(&sensor.preview_texture) {
            let mut rgba = Vec::with_capacity(depth.len() * 4);
            for &metres in &depth {
                let shade = if metres.is_finite() {
                    (255.0
                        * (1.0
                            - (metres - sensor.config.near)
                                / (sensor.config.far - sensor.config.near))
                            .clamp(0.0, 1.0)) as u8
                } else {
                    0
                };
                rgba.extend_from_slice(&[shade, shade, shade, 255]);
            }
            image.data = Some(rgba);
        }
        if frame
            .exposure
            .is_some_and(|old| capture.exposure.timestamp <= old.timestamp)
        {
            continue;
        }
        frame.exposure = Some(capture.exposure);
        frame.depth_metres = depth;
        frame.sequence += 1;
        frame.received_at = time.elapsed_secs_f64();
    }
}

fn remove_orphan_previews(
    mut commands: Commands,
    previews: Query<(Entity, &DepthPreview)>,
    sensors: Query<(), With<DepthCamera>>,
) {
    for (entity, preview) in &previews {
        if sensors.get(preview.0).is_err() {
            commands.entity(entity).despawn();
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    #[ignore = "requires a GPU; run explicitly to verify depth rendering and readback"]
    fn gpu_captures_wall_distance_in_metres() {
        use bevy::{app::PluginsState, window::ExitCondition, winit::WinitPlugin};
        let mut app = App::new();
        app.add_plugins(
            DefaultPlugins
                .build()
                .disable::<WinitPlugin>()
                .set(WindowPlugin {
                    primary_window: None,
                    exit_condition: ExitCondition::DontExit,
                    ..default()
                }),
        )
        .insert_resource(RoverPhysicsConfig::default())
        .add_plugins(TerraDepthCameraPlugin {
            config: DepthCameraConfig {
                resolution: UVec2::new(65, 49),
                preview: false,
                ..default()
            },
        })
        .add_plugins(crate::occupancy_map::TerraOccupancyMapPlugin);
        let deadline = std::time::Instant::now() + std::time::Duration::from_secs(45);
        while app.plugins_state() == PluginsState::Adding {
            assert!(
                std::time::Instant::now() < deadline,
                "renderer initialization timed out"
            );
            bevy::tasks::tick_global_task_pools_on_main_thread();
            std::thread::sleep(std::time::Duration::from_millis(5));
        }
        app.finish();
        app.cleanup();
        let mesh = app
            .world_mut()
            .resource_mut::<Assets<Mesh>>()
            .add(Cuboid::new(4.0, 4.0, 0.2));
        let material = app
            .world_mut()
            .resource_mut::<Assets<StandardMaterial>>()
            .add(StandardMaterial::default());
        app.world_mut().spawn((
            Mesh3d(mesh),
            MeshMaterial3d(material),
            Transform::from_xyz(0.0, 0.0, -3.0),
        ));
        app.world_mut().spawn((Rover, Transform::default()));
        let mut captured = false;
        while std::time::Instant::now() < deadline {
            app.update();
            let world = app.world_mut();
            if let Ok(frame) = world.query::<&DepthFrame>().single(world) {
                let index = (frame.height / 2 * frame.width + frame.width / 2) as usize;
                if let Some(&metres) = frame.depth_metres.get(index)
                    && metres.is_finite()
                {
                    assert!((metres - 2.45).abs() < 0.05, "wall depth: {metres}");
                    let exposure = frame
                        .exposure
                        .expect("GPU copy must carry its exposure pose");
                    assert!(exposure.timestamp <= frame.received_at);
                    assert!((exposure.camera_transform.translation().z + 0.45).abs() < 1e-5);
                    captured = true;
                    break;
                }
            }
            std::thread::sleep(std::time::Duration::from_millis(5));
        }
        assert!(captured, "no valid depth frame received within 45 seconds");
        let world = app.world_mut();
        let mapped = world
            .query::<&crate::occupancy_map::RoverOccupancyMap>()
            .single(world)
            .unwrap();
        assert!(mapped.last_exposure.is_some());
        assert!(
            mapped.map.snapshot().occupancy.iter().any(|p| *p > 50),
            "rendered wall must reach occupancy grid"
        );
        // Stop capture and drain pending readbacks before the GPU app is dropped.
        let sensors: Vec<_> = app
            .world_mut()
            .query_filtered::<Entity, With<DepthCamera>>()
            .iter(app.world())
            .collect();
        for sensor in sensors {
            app.world_mut().entity_mut(sensor).remove::<DepthCapture>();
        }
        for _ in 0..20 {
            app.update();
            std::thread::sleep(std::time::Duration::from_millis(5));
        }
    }

    #[test]
    fn converts_reverse_depth_and_removes_row_padding() {
        let config = DepthCameraConfig {
            resolution: UVec2::new(3, 2),
            near: 0.1,
            far: 10.0,
            ..default()
        };
        let mut bytes = vec![0u8; 512];
        for (row, values) in [[1.0_f32, 0.05, 0.0], [0.1, 0.005, f32::NAN]]
            .iter()
            .enumerate()
        {
            for (column, value) in values.iter().enumerate() {
                let offset = row * 256 + column * 4;
                bytes[offset..offset + 4].copy_from_slice(&value.to_le_bytes());
            }
        }
        let frame = decode_depth(&bytes, &config).unwrap();
        assert_eq!(frame.len(), 6);
        assert!((frame[0] - 0.1).abs() < 1e-5);
        assert!((frame[1] - 2.0).abs() < 1e-5);
        assert!(frame[2].is_nan());
        assert!((frame[3] - 1.0).abs() < 1e-5);
        assert!(frame[4].is_nan() && frame[5].is_nan());
        assert!(decode_depth(&bytes[..10], &config).is_none());
    }

    #[test]
    fn validates_sensor_parameters() {
        assert!(DepthCameraConfig::default().validate().is_ok());
        assert!(
            DepthCameraConfig {
                resolution: UVec2::ZERO,
                ..default()
            }
            .validate()
            .is_err()
        );
        assert!(
            DepthCameraConfig {
                near: 0.0,
                ..default()
            }
            .validate()
            .is_err()
        );
        assert!(
            DepthCameraConfig {
                far: 0.01,
                ..default()
            }
            .validate()
            .is_err()
        );
        assert!(
            DepthCameraConfig {
                vertical_fov: std::f32::consts::PI,
                ..default()
            }
            .validate()
            .is_err()
        );
    }

    #[test]
    fn mounts_once_and_follows_rover_pose() {
        let mut app = App::new();
        app.add_plugins((MinimalPlugins, TransformPlugin))
            .insert_resource(Assets::<Image>::default())
            .insert_resource(RoverPhysicsConfig::default())
            .add_plugins(TerraDepthCameraPlugin {
                config: DepthCameraConfig {
                    preview: false,
                    ..default()
                },
            });
        let rover = app
            .world_mut()
            .spawn((Rover, Transform::from_xyz(2.0, 1.0, 3.0)))
            .id();
        app.update();
        let world = app.world_mut();
        let (parent, local, global) = world
            .query_filtered::<(&ChildOf, &Transform, &GlobalTransform), With<DepthCamera>>()
            .single(world)
            .unwrap();
        assert_eq!(parent.parent(), rover);
        assert!(local.translation.z < -0.4);
        assert!((global.translation().x - 2.0).abs() < 1e-5);
        world.get_mut::<Transform>(rover).unwrap().rotation =
            Quat::from_rotation_y(std::f32::consts::FRAC_PI_2);
        app.update();
        app.update();
        let world = app.world_mut();
        let global = world
            .query_filtered::<&GlobalTransform, With<DepthCamera>>()
            .single(world)
            .unwrap();
        assert!((global.rotation() * Vec3::NEG_Z).x < -0.99);
        assert_eq!(
            world
                .query_filtered::<Entity, With<DepthCamera>>()
                .iter(world)
                .count(),
            1
        );
        world.despawn(rover);
        app.update();
        let world = app.world_mut();
        assert_eq!(
            world
                .query_filtered::<Entity, With<DepthCamera>>()
                .iter(world)
                .count(),
            0
        );
    }
}
