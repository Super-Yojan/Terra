//! Occupancy render (~800×800) and a labelled camera schematic.

use image::{DynamicImage, ImageFormat, Rgba, RgbaImage};
use std::io::Cursor;
use terra_mapping::MapSnapshot;
use terra_navigation::Pose;

use crate::font::{self, GLYPH_H, GLYPH_W};
use crate::geom::cell_center;
use crate::objects::in_camera_wedge;
use crate::scene::{CameraModel, SceneObject};

const CELL_PX: i32 = 4;
const FREE: Rgba<u8> = Rgba([214, 214, 214, 255]);
const OCCUPIED: Rgba<u8> = Rgba([0, 0, 0, 255]);
const UNKNOWN: Rgba<u8> = Rgba([55, 55, 55, 255]);
const GRID: Rgba<u8> = Rgba([168, 168, 168, 255]);
const ROVER: Rgba<u8> = Rgba([210, 32, 32, 255]);
const FOV_TINT: [u8; 3] = [70, 150, 230];

pub fn render_map_png(
    map: &MapSnapshot,
    pose: Pose,
    camera: &CameraModel,
    objects: &[SceneObject],
) -> Result<Vec<u8>, String> {
    encode_png(&render_map(map, pose, camera, objects))
}

pub fn render_camera_png(
    scenario_id: &str,
    title: &str,
    pose: Pose,
    camera: &CameraModel,
    objects: &[SceneObject],
) -> Result<Vec<u8>, String> {
    encode_png(&render_camera(scenario_id, title, pose, camera, objects))
}

pub fn render_map(
    map: &MapSnapshot,
    pose: Pose,
    camera: &CameraModel,
    objects: &[SceneObject],
) -> RgbaImage {
    let width = map.width as i32 * CELL_PX;
    let height = map.height as i32 * CELL_PX;
    let mut image = RgbaImage::from_pixel(width as u32, height as u32, UNKNOWN);
    for row in 0..map.height as i32 {
        for col in 0..map.width as i32 {
            let value = map.occupancy[row as usize * map.width as usize + col as usize];
            let mut color = cell_color(value);
            let (x, y) = cell_center(map, col, row);
            if value >= 0 && value < 65 && on_metre_line(map, col, row) {
                color = GRID;
            }
            let (px, py) = world_to_pixel(map, x, y);
            fill_rect(&mut image, px, py, px + CELL_PX, py + CELL_PX, color);
        }
    }
    tint_fov(&mut image, map, pose, camera);
    draw_fov_rays(&mut image, map, pose, camera);
    draw_axis(&mut image);
    draw_scale_bar(&mut image);
    draw_legend(&mut image);
    draw_rover(&mut image, map, pose);
    for object in objects {
        draw_object(&mut image, map, object);
    }
    image
}

pub fn cell_color(value: i8) -> Rgba<u8> {
    if value < 0 {
        UNKNOWN
    } else if value >= 65 {
        OCCUPIED
    } else {
        FREE
    }
}

fn on_metre_line(map: &MapSnapshot, col: i32, row: i32) -> bool {
    let cells = (1.0 / map.resolution).round() as i32;
    col % cells == 0 || row % cells == 0
}

pub fn world_to_pixel(map: &MapSnapshot, x: f64, y: f64) -> (i32, i32) {
    let col = ((x - map.origin_x) / map.resolution).floor() as i32;
    let row = ((y - map.origin_y) / map.resolution).floor() as i32;
    let px = col * CELL_PX;
    let py = (map.height as i32 - 1 - row) * CELL_PX;
    (px, py)
}

fn tint_fov(image: &mut RgbaImage, map: &MapSnapshot, pose: Pose, camera: &CameraModel) {
    for row in 0..map.height as i32 {
        for col in 0..map.width as i32 {
            let (x, y) = cell_center(map, col, row);
            if !in_camera_wedge(pose.x, pose.y, pose.yaw, camera, x, y) {
                continue;
            }
            let (px, py) = world_to_pixel(map, x, y);
            for dy in 0..CELL_PX {
                for dx in 0..CELL_PX {
                    blend(image, px + dx, py + dy, FOV_TINT, 0.38);
                }
            }
        }
    }
}

fn draw_fov_rays(image: &mut RgbaImage, map: &MapSnapshot, pose: Pose, camera: &CameraModel) {
    let half = camera.half_fov();
    for sign in [-1.0_f64, 1.0] {
        let angle = pose.yaw + sign * half;
        let end = [
            pose.x + angle.cos() * camera.range_m,
            pose.y + angle.sin() * camera.range_m,
        ];
        let (x0, y0) = world_to_pixel(map, pose.x, pose.y);
        let (x1, y1) = world_to_pixel(map, end[0], end[1]);
        draw_line(
            image,
            x0 + 2,
            y0 + 2,
            x1 + 2,
            y1 + 2,
            Rgba([40, 110, 200, 255]),
        );
    }
}

fn draw_rover(image: &mut RgbaImage, map: &MapSnapshot, pose: Pose) {
    let (x0, y0) = world_to_pixel(map, pose.x, pose.y);
    let tip = [pose.x + pose.yaw.cos() * 0.9, pose.y + pose.yaw.sin() * 0.9];
    let (x1, y1) = world_to_pixel(map, tip[0], tip[1]);
    let cx = x0 + 2;
    let cy = y0 + 2;
    let tx = x1 + 2;
    let ty = y1 + 2;
    draw_line(image, cx, cy, tx, ty, ROVER);
    let back = pose.yaw + std::f64::consts::PI;
    for spread in [2.4_f64, -2.4] {
        let wing = [
            tip[0] + (back + spread).cos() * 0.28,
            tip[1] + (back + spread).sin() * 0.28,
        ];
        let (wx, wy) = world_to_pixel(map, wing[0], wing[1]);
        draw_line(image, tx, ty, wx + 2, wy + 2, ROVER);
    }
    fill_disk(image, cx, cy, 4, ROVER);
}

fn draw_object(image: &mut RgbaImage, map: &MapSnapshot, object: &SceneObject) {
    if object.carried {
        return;
    }
    let color = class_color(&object.class);
    let (px, py) = world_to_pixel(map, object.x, object.y);
    stroke_disk(image, px + 2, py + 2, 7, color);
    if object.goal_eligible {
        let (ax, ay) = world_to_pixel(map, object.approach_x, object.approach_y);
        stroke_disk(image, ax + 2, ay + 2, 4, color);
    }
    draw_text(image, px + 12, py - 8, &object.id, color, 2);
}

fn class_color(class: &str) -> Rgba<u8> {
    match class {
        "bucket" => Rgba([230, 150, 20, 255]),
        "person" => Rgba([230, 40, 150, 255]),
        "ball" => Rgba([40, 90, 220, 255]),
        "frontier" => Rgba([30, 180, 80, 255]),
        _ => Rgba([245, 245, 245, 255]),
    }
}

fn draw_legend(image: &mut RgbaImage) {
    fill_rect(image, 8, 8, 132, 78, Rgba([16, 16, 16, 230]));
    draw_swatch(image, 14, 14, FREE, "FREE");
    draw_swatch(image, 14, 30, OCCUPIED, "OCC");
    draw_swatch(image, 14, 46, UNKNOWN, "UNK");
    draw_swatch(
        image,
        14,
        62,
        Rgba([FOV_TINT[0], FOV_TINT[1], FOV_TINT[2], 255]),
        "FOV",
    );
}

fn draw_swatch(image: &mut RgbaImage, x: i32, y: i32, color: Rgba<u8>, label: &str) {
    fill_rect(image, x, y, x + 12, y + 10, color);
    draw_text(image, x + 16, y - 1, label, Rgba([255, 255, 255, 255]), 1);
}

fn draw_scale_bar(image: &mut RgbaImage) {
    let x = 16;
    let y = 772;
    fill_rect(
        image,
        x - 4,
        y - 16,
        x + 52,
        y + 14,
        Rgba([16, 16, 16, 220]),
    );
    fill_rect(image, x, y, x + 40, y + 4, Rgba([255, 255, 255, 255]));
    draw_text(image, x, y - 12, "1 M", Rgba([255, 255, 255, 255]), 1);
}

fn draw_axis(image: &mut RgbaImage) {
    let x = 700;
    let y = 28;
    fill_rect(
        image,
        x - 8,
        y - 16,
        x + 88,
        y + 36,
        Rgba([16, 16, 16, 220]),
    );
    draw_line(image, x, y, x + 36, y, Rgba([255, 255, 255, 255]));
    draw_line(image, x + 36, y, x + 30, y - 4, Rgba([255, 255, 255, 255]));
    draw_line(image, x + 36, y, x + 30, y + 4, Rgba([255, 255, 255, 255]));
    draw_text(image, x + 40, y - 4, "+X", Rgba([255, 255, 255, 255]), 1);
    draw_line(image, x, y, x, y - 28, Rgba([255, 255, 255, 255]));
    draw_line(image, x, y - 28, x - 4, y - 22, Rgba([255, 255, 255, 255]));
    draw_line(image, x, y - 28, x + 4, y - 22, Rgba([255, 255, 255, 255]));
    draw_text(image, x + 8, y - 28, "+Y", Rgba([255, 255, 255, 255]), 1);
}

pub fn render_camera(
    scenario_id: &str,
    title: &str,
    pose: Pose,
    camera: &CameraModel,
    objects: &[SceneObject],
) -> RgbaImage {
    let mut image = RgbaImage::from_pixel(camera.width, camera.height, Rgba([28, 30, 34, 255]));
    fill_rect(
        &mut image,
        0,
        0,
        camera.width as i32,
        36,
        Rgba([140, 30, 30, 255]),
    );
    draw_text(
        &mut image,
        8,
        8,
        "PLACEHOLDER  NOT A ZORVANE RENDER",
        Rgba([255, 255, 255, 255]),
        2,
    );
    draw_text(
        &mut image,
        8,
        48,
        &scenario_id.to_ascii_uppercase(),
        Rgba([230, 230, 230, 255]),
        2,
    );
    let title = title.to_ascii_uppercase();
    draw_text(&mut image, 8, 70, &title, Rgba([190, 190, 190, 255]), 1);
    let mut lane = 0;
    for object in objects {
        if object.class == "frontier" || object.class == "obstacle" {
            continue;
        }
        let visible =
            object.carried || in_camera_wedge(pose.x, pose.y, pose.yaw, camera, object.x, object.y);
        if !visible {
            continue;
        }
        let (u, depth) = if object.carried {
            (camera.cx as i32, 0.8)
        } else {
            project(pose, camera, object.x, object.y).unwrap_or((camera.cx as i32, 2.0))
        };
        let box_w = (90.0 / depth.max(0.6)).clamp(28.0, 120.0) as i32;
        let box_h = box_w;
        let left = (u - box_w / 2).clamp(8, camera.width as i32 - box_w - 8);
        let top = 150 + lane * 8;
        let color = class_color(&object.class);
        stroke_rect(&mut image, left, top, left + box_w, top + box_h, color);
        let label = format!("{} {:.2}", object.id, object.confidence);
        draw_text(&mut image, left, top - 12, &label, color, 1);
        if object.carried {
            draw_text(&mut image, left, top + box_h + 2, "CARRIED", color, 1);
        }
        lane += 1;
    }
    draw_text(
        &mut image,
        8,
        camera.height as i32 - 28,
        "SYNTHETIC SCHEMATIC  CAMERA_SOURCE=PLACEHOLDER",
        Rgba([220, 80, 80, 255]),
        1,
    );
    image
}

fn project(pose: Pose, camera: &CameraModel, x: f64, y: f64) -> Option<(i32, f64)> {
    let dx = x - pose.x;
    let dy = y - pose.y;
    let forward = dx * pose.yaw.cos() + dy * pose.yaw.sin();
    if forward <= 0.2 {
        return None;
    }
    let right_x = pose.yaw.sin();
    let right_y = -pose.yaw.cos();
    let right = dx * right_x + dy * right_y;
    let u = camera.cx + camera.fx * (right / forward);
    Some((u.round() as i32, forward))
}

fn draw_text(image: &mut RgbaImage, x: i32, y: i32, text: &str, color: Rgba<u8>, scale: i32) {
    let mut cursor = x;
    for ch in text.chars() {
        let rows = font::glyph(ch);
        for row in 0..GLYPH_H {
            let bits = rows[row as usize];
            for col in 0..GLYPH_W {
                if bits & (1 << (4 - col)) == 0 {
                    continue;
                }
                let px = cursor + col * scale;
                let py = y + row as i32 * scale;
                fill_rect(image, px, py, px + scale, py + scale, color);
            }
        }
        cursor += (GLYPH_W + 1) * scale;
    }
}

fn blend(image: &mut RgbaImage, x: i32, y: i32, tint: [u8; 3], alpha: f32) {
    if x < 0 || y < 0 || x >= image.width() as i32 || y >= image.height() as i32 {
        return;
    }
    let pixel = image.get_pixel_mut(x as u32, y as u32);
    for channel in 0..3 {
        let mixed = f32::from(pixel[channel]) * (1.0 - alpha) + f32::from(tint[channel]) * alpha;
        pixel[channel] = mixed.round() as u8;
    }
}

fn fill_rect(image: &mut RgbaImage, x0: i32, y0: i32, x1: i32, y1: i32, color: Rgba<u8>) {
    let left = x0.max(0);
    let top = y0.max(0);
    let right = x1.min(image.width() as i32);
    let bottom = y1.min(image.height() as i32);
    for y in top..bottom {
        for x in left..right {
            image.put_pixel(x as u32, y as u32, color);
        }
    }
}

fn fill_disk(image: &mut RgbaImage, cx: i32, cy: i32, radius: i32, color: Rgba<u8>) {
    for y in cy - radius..=cy + radius {
        for x in cx - radius..=cx + radius {
            if (x - cx) * (x - cx) + (y - cy) * (y - cy) <= radius * radius {
                if x >= 0 && y >= 0 && x < image.width() as i32 && y < image.height() as i32 {
                    image.put_pixel(x as u32, y as u32, color);
                }
            }
        }
    }
}

fn stroke_disk(image: &mut RgbaImage, cx: i32, cy: i32, radius: i32, color: Rgba<u8>) {
    let r2 = radius * radius;
    let inner = (radius - 1).max(0) * (radius - 1).max(0);
    for y in cy - radius..=cy + radius {
        for x in cx - radius..=cx + radius {
            let d = (x - cx) * (x - cx) + (y - cy) * (y - cy);
            if d <= r2 && d >= inner {
                if x >= 0 && y >= 0 && x < image.width() as i32 && y < image.height() as i32 {
                    image.put_pixel(x as u32, y as u32, color);
                }
            }
        }
    }
}

fn stroke_rect(image: &mut RgbaImage, x0: i32, y0: i32, x1: i32, y1: i32, color: Rgba<u8>) {
    draw_line(image, x0, y0, x1, y0, color);
    draw_line(image, x1, y0, x1, y1, color);
    draw_line(image, x1, y1, x0, y1, color);
    draw_line(image, x0, y1, x0, y0, color);
}

fn draw_line(image: &mut RgbaImage, mut x0: i32, mut y0: i32, x1: i32, y1: i32, color: Rgba<u8>) {
    let dx = (x1 - x0).abs();
    let sx = if x0 < x1 { 1 } else { -1 };
    let dy = -(y1 - y0).abs();
    let sy = if y0 < y1 { 1 } else { -1 };
    let mut err = dx + dy;
    loop {
        if x0 >= 0 && y0 >= 0 && x0 < image.width() as i32 && y0 < image.height() as i32 {
            image.put_pixel(x0 as u32, y0 as u32, color);
        }
        if x0 == x1 && y0 == y1 {
            break;
        }
        let e2 = 2 * err;
        if e2 >= dy {
            err += dy;
            x0 += sx;
        }
        if e2 <= dx {
            err += dx;
            y0 += sy;
        }
    }
}

fn encode_png(image: &RgbaImage) -> Result<Vec<u8>, String> {
    let mut bytes = Vec::new();
    DynamicImage::ImageRgba8(image.clone())
        .write_to(&mut Cursor::new(&mut bytes), ImageFormat::Png)
        .map_err(|err| err.to_string())?;
    Ok(bytes)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geom::{blank, fill_disk, fill_rect};
    use terra_navigation::Pose;

    /// The map renders at 800 px square with distinguishable unknown and free
    /// colors, plus the red rover overlay.
    #[test]
    fn map_is_800_and_uses_the_occupancy_palette() {
        let mut map = blank();
        fill_rect(&mut map, -2.0, -2.0, 2.0, 2.0, 0);
        fill_disk(&mut map, 0.0, 0.0, 0.25, 100);
        let pose = Pose {
            x: 0.0,
            y: 0.0,
            yaw: 0.0,
        };
        let image = render_map(&map, pose, &CameraModel::standard(), &[]);
        assert_eq!(image.width(), 800);
        assert_eq!(image.height(), 800);
        let (ux, uy) = world_to_pixel(&map, 8.5, -8.5);
        let unknown = image.get_pixel(ux as u32 + 1, uy as u32 + 1);
        assert!(unknown[0] < 80 && unknown[1] < 80 && unknown[2] < 80);
        let (fx, fy) = world_to_pixel(&map, -1.5, -1.5);
        let free = image.get_pixel(fx as u32 + 1, fy as u32 + 1);
        assert!(free[0] > 180 && (free[0] as i16 - free[1] as i16).abs() < 8);
        assert!(
            image
                .pixels()
                .any(|pixel| pixel[0] > 180 && pixel[1] < 60 && pixel[2] < 60)
        );
    }
}
