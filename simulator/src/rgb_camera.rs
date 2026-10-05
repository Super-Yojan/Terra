//! RGB camera colocated with the front depth sensor, with asynchronous readback.
use crate::depth_camera::{DepthCamera, DepthCameraConfig};
use bevy::{
    asset::RenderAssetUsages,
    camera::RenderTarget,
    prelude::*,
    render::{
        gpu_readback::{Readback, ReadbackComplete},
        render_resource::{Extent3d, TextureDimension, TextureFormat, TextureUsages},
    },
};

pub struct TerraRgbCameraPlugin;
impl Plugin for TerraRgbCameraPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(Update, mount_rgb_cameras);
    }
}
#[derive(Component)]
struct RgbMounted;
#[derive(Component)]
pub struct RgbCamera {
    pub config: DepthCameraConfig,
}
#[derive(Component, Default)]
pub struct RgbFrame {
    pub width: u32,
    pub height: u32,
    pub rgba: Vec<u8>,
    pub sequence: u64,
    pub received_at: f64,
}
fn mount_rgb_cameras(
    mut commands: Commands,
    sensors: Query<(Entity, &DepthCamera, &crate::terra::RoverId), Without<RgbMounted>>,
    mut images: ResMut<Assets<Image>>,
) {
    for (entity, sensor, id) in &sensors {
        let config = &sensor.config;
        let mut image = Image::new_fill(
            Extent3d {
                width: config.resolution.x,
                height: config.resolution.y,
                depth_or_array_layers: 1,
            },
            TextureDimension::D2,
            &[0, 0, 0, 255],
            TextureFormat::Rgba8UnormSrgb,
            RenderAssetUsages::default(),
        );
        image.texture_descriptor.usage |=
            TextureUsages::COPY_SRC | TextureUsages::RENDER_ATTACHMENT;
        let target = images.add(image);
        commands.entity(entity).insert(RgbMounted);
        commands
            .spawn((
                Name::new("Terra front RGB camera"),
                *id,
                RgbCamera {
                    config: config.clone(),
                },
                RgbFrame {
                    width: config.resolution.x,
                    height: config.resolution.y,
                    ..default()
                },
                Camera3d::default(),
                Camera {
                    order: -2,
                    ..default()
                },
                RenderTarget::Image(target.clone().into()),
                Projection::Perspective(PerspectiveProjection {
                    fov: config.vertical_fov,
                    near: config.near,
                    far: config.far,
                    aspect_ratio: config.resolution.x as f32 / config.resolution.y as f32,
                    ..default()
                }),
                Msaa::Off,
                Transform::IDENTITY,
                ChildOf(entity),
                Readback::texture(target),
            ))
            .observe(receive_rgb);
    }
}
fn strip_padding(data: &[u8], width: u32, height: u32) -> Option<Vec<u8>> {
    if width == 0 || height == 0 {
        return None;
    }
    let row = (width as usize).checked_mul(4)?;
    let stride = row.div_ceil(256).checked_mul(256)?;
    if data.len() != stride.checked_mul(height as usize)? {
        return None;
    }
    Some(
        data.chunks_exact(stride)
            .flat_map(|row_data| row_data[..row].iter().copied())
            .collect(),
    )
}
fn receive_rgb(event: On<ReadbackComplete>, time: Res<Time>, mut frames: Query<&mut RgbFrame>) {
    let Ok(mut frame) = frames.get_mut(event.entity) else {
        return;
    };
    let Some(rgba) = strip_padding(&event.data, frame.width, frame.height) else {
        return;
    };
    frame.rgba = rgba;
    frame.sequence += 1;
    frame.received_at = time.elapsed_secs_f64();
}
#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn removes_gpu_row_padding_without_flipping_or_changing_channels() {
        let mut padded = vec![0; 512];
        padded[..8].copy_from_slice(&[1, 2, 3, 255, 4, 5, 6, 255]);
        padded[256..264].copy_from_slice(&[7, 8, 9, 255, 10, 11, 12, 255]);
        assert_eq!(
            strip_padding(&padded, 2, 2).unwrap(),
            [1, 2, 3, 255, 4, 5, 6, 255, 7, 8, 9, 255, 10, 11, 12, 255]
        );
        assert!(strip_padding(&padded[..260], 2, 2).is_none());
    }
}
