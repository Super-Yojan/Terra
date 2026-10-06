//! Optional real-world ground patch from Terrarium elevation tiles.
//!
//! The flat practice world stays the default. `TERRA_TILES=1` anchors a square
//! patch on a web-mercator tile, samples it with bevytiles, and turns steep
//! cells into static obstacles. The rover's roll and pitch stay locked, so the
//! Avian ground plane remains flat; elevation is colour plus collision blocks,
//! not a displaced heightfield. The full bevytiles `TerrainPlugin` is not
//! mounted: its tiles are tens of kilometres, displacement is GPU-only, and a
//! streaming camera would fight the rover overview camera.

use avian3d::prelude::*;
use bevy::prelude::*;
use bevytiles::{
    config::{MIN_ZOOM, NetworkConfig, TerrainAnchor, WorldConfig as TileTopology},
    height::{HeightGrids, ground_height},
    lod::TileKey,
    source::{TileDrop, TileRequest, TileSource},
};
use std::path::{Path, PathBuf};
use std::time::{Duration, Instant};

/// Matches the circumference bevytiles uses for `WorldConfig::from_lat_lon`.
const EQUATOR_CIRCUMFERENCE_M: f64 = 40_075_016.686;
const METRES_PER_DEGREE: f64 = EQUATOR_CIRCUMFERENCE_M / 360.0;
/// Native Mapzen / AWS terrain zoom. Higher zooms are synthesized; a rover
/// wants the native ~4 m samples.
const NATIVE_ZOOM: u8 = 15;
const CLEARANCE_M: f32 = 3.0;
const BUNDLE_ZOOM: u8 = 15;
const BUNDLE_X: i32 = 9347;
const BUNDLE_Z: i32 = 12543;
const BUNDLE_PNG: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/assets/geo/terrarium.png");

/// George W. Johnson Center, George Mason University Fairfax campus.
/// Published location 38.8297 N, 77.3075 W. The bundled Terrarium PNG is this
/// zoom-15 tile, so the default location works with fetch disabled.
pub const DEFAULT_LATITUDE: f64 = 38.8297;
pub const DEFAULT_LONGITUDE: f64 = -77.3075;

#[derive(Clone, Debug)]
pub struct GeoTileConfig {
    pub enabled: bool,
    pub latitude: f64,
    pub longitude: f64,
    pub zoom: u8,
    pub fetch: bool,
    pub cache_dir: PathBuf,
    /// Rise/run above which a cell becomes a static obstacle.
    pub slope_limit: f32,
}

impl Default for GeoTileConfig {
    fn default() -> Self {
        Self {
            enabled: false,
            latitude: DEFAULT_LATITUDE,
            longitude: DEFAULT_LONGITUDE,
            zoom: NATIVE_ZOOM,
            fetch: true,
            cache_dir: PathBuf::from(concat!(env!("CARGO_MANIFEST_DIR"), "/.cache/tiles")),
            slope_limit: 0.35,
        }
    }
}

impl GeoTileConfig {
    pub fn from_env() -> Result<Self, String> {
        let mut config = Self::default();
        config.enabled = env_flag("TERRA_TILES");
        if !config.enabled {
            return Ok(config);
        }
        if let Ok(value) = std::env::var("TERRA_LAT") {
            config.latitude = value
                .parse()
                .map_err(|_| "TERRA_LAT must be decimal degrees".to_owned())?;
        }
        if let Ok(value) = std::env::var("TERRA_LON") {
            config.longitude = value
                .parse()
                .map_err(|_| "TERRA_LON must be decimal degrees".to_owned())?;
        }
        if let Ok(value) = std::env::var("TERRA_ZOOM") {
            config.zoom = value
                .parse()
                .map_err(|_| "TERRA_ZOOM must be an integer".to_owned())?;
        }
        config.fetch = env_flag_default("TERRA_TILES_FETCH", true);
        if let Ok(value) = std::env::var("TERRA_TILE_CACHE") {
            config.cache_dir = PathBuf::from(value);
        }
        config.validate().map_err(str::to_owned)?;
        Ok(config)
    }

    pub fn validate(&self) -> Result<(), &'static str> {
        if !self.enabled {
            return Ok(());
        }
        if !self.latitude.is_finite() || !(-85.0..=85.0).contains(&self.latitude) {
            return Err("TERRA_LAT must be between -85 and 85 degrees");
        }
        if !self.longitude.is_finite() || !(-180.0..=180.0).contains(&self.longitude) {
            return Err("TERRA_LON must be between -180 and 180 degrees");
        }
        if !(MIN_ZOOM..=NATIVE_ZOOM).contains(&self.zoom) {
            return Err("TERRA_ZOOM must be from 9 through 15");
        }
        if !self.slope_limit.is_finite() || !(0.05..=2.0).contains(&self.slope_limit) {
            return Err("tile slope limit must be between 0.05 and 2");
        }
        if self.cache_dir.as_os_str().is_empty() {
            return Err("tile cache directory must not be empty");
        }
        Ok(())
    }
}

#[derive(Clone)]
pub struct GeoAnchor {
    pub latitude: f64,
    pub longitude: f64,
    pub zoom: u8,
    pub tile_metres: f32,
    topology: TileTopology,
    world_offset: Vec3,
}

impl GeoAnchor {
    pub fn try_new(latitude: f64, longitude: f64, zoom: u8) -> Result<Self, &'static str> {
        if !latitude.is_finite()
            || !longitude.is_finite()
            || !(-85.0..=85.0).contains(&latitude)
            || !(-180.0..=180.0).contains(&longitude)
            || !(MIN_ZOOM..=NATIVE_ZOOM).contains(&zoom)
        {
            return Err("geographic anchor is out of range");
        }
        let mut topology = TileTopology::from_lat_lon(latitude, longitude);
        topology.max_zoom = zoom;
        let shift = zoom - topology.base_zoom;
        let tile_metres = topology.tile_size / (1u32 << u32::from(shift)) as f32;
        if !tile_metres.is_finite() || tile_metres <= 1.0 {
            return Err("tile size is not usable");
        }
        // User origin is the requested lat/lon. bevytiles puts that point at
        // `origin_offset` inside the base tile, so shift absolute space back.
        let world_offset = -topology.origin_offset;
        Ok(Self {
            latitude,
            longitude,
            zoom,
            tile_metres,
            topology,
            world_offset,
        })
    }

    /// Robotics metres from this anchor: +x north, +y west.
    /// Bevy +X is east and Bevy +Z is south, matching web-mercator tile axes,
    /// and the simulator's robotics frame is `x = -Z`, `y = -X`.
    pub fn local_xy(&self, latitude: f64, longitude: f64) -> (f64, f64) {
        let north = (latitude - self.latitude) * METRES_PER_DEGREE;
        let east =
            (longitude - self.longitude) * METRES_PER_DEGREE * self.latitude.to_radians().cos();
        (north, -east)
    }
}

#[derive(Clone, Copy, Debug)]
pub struct PatchCell {
    pub bevy_x: f32,
    pub bevy_z: f32,
    pub relative_height: f32,
    pub obstacle: bool,
    pub block_height: f32,
}

#[derive(Clone, Resource)]
pub struct GeoPatch {
    pub anchor: GeoAnchor,
    pub step: f32,
    pub origin_elevation: f32,
    pub cells: Vec<PatchCell>,
}

#[derive(Component)]
pub struct GeoObstacle;
#[derive(Component)]
pub struct GeoTint;

struct TileRef {
    key: TileKey,
    provider_x: i32,
    provider_z: i32,
}

pub fn load_patch(config: &GeoTileConfig, extent: f32) -> Result<GeoPatch, String> {
    config.validate().map_err(str::to_owned)?;
    if !extent.is_finite() || !(40.0..=512.0).contains(&extent) {
        return Err("tile worlds must be 40 to 512 metres wide".into());
    }
    let anchor = GeoAnchor::try_new(config.latitude, config.longitude, config.zoom)
        .map_err(str::to_owned)?;
    let tiles = tiles_for_square(&anchor, extent);
    if tiles.is_empty() || tiles.len() > 16 {
        return Err(
            "tile area does not fit in 16 elevation tiles; reduce the world size or raise the zoom"
                .into(),
        );
    }
    seed_bundle(&config.cache_dir, &tiles)?;
    if !config.fetch {
        for tile in &tiles {
            if !height_cache(&config.cache_dir, tile).exists() {
                return Err(format!(
                    "no cached Terrarium tile {}/{}/{} and TERRA_TILES_FETCH=0",
                    tile.key.zoom, tile.provider_x, tile.provider_z
                ));
            }
        }
    }
    let mut grids = HeightGrids::default();
    {
        let source = TileSource::new(&NetworkConfig {
            threads: 2,
            cache_dir: config.cache_dir.clone(),
            texture_url: terrarium_template(),
            heightmap_url: terrarium_template(),
            normals_url: terrarium_template(),
            native_terrain_zoom: NATIVE_ZOOM,
            connect_timeout: Duration::from_secs(4),
            read_timeout: Duration::from_secs(8),
            ..default()
        });
        for tile in &tiles {
            source.request(TileRequest {
                key: tile.key,
                x: tile.provider_x,
                z: tile.provider_z,
            });
        }
        let deadline = Instant::now() + Duration::from_secs(if config.fetch { 20 } else { 8 });
        let mut ready = Vec::new();
        let mut dropped = Vec::new();
        while grids.0.len() < tiles.len() {
            source.drain(&mut ready, &mut dropped);
            for payload in ready.drain(..) {
                grids.0.insert(payload.key, payload.grid);
            }
            if let Some(drop) = dropped.drain(..).next() {
                return Err(match drop {
                    TileDrop::Failed(key, reason) => {
                        format!("elevation tile {key:?} failed: {reason}")
                    }
                    TileDrop::Cancelled(key) => format!("elevation tile {key:?} was cancelled"),
                });
            }
            if Instant::now() > deadline {
                return Err("timed out loading elevation tiles".into());
            }
            if grids.0.len() < tiles.len() {
                std::thread::sleep(Duration::from_millis(10));
            }
        }
    }
    build_patch(&grids, &anchor, extent, config.slope_limit)
}

pub fn build_patch(
    grids: &HeightGrids,
    anchor: &GeoAnchor,
    extent: f32,
    slope_limit: f32,
) -> Result<GeoPatch, String> {
    let step = sample_step(anchor.tile_metres);
    let terrain_anchor = TerrainAnchor {
        world_offset: anchor.world_offset,
    };
    let origin = ground_height(grids, &anchor.topology, &terrain_anchor, Vec3::ZERO)
        .ok_or("no elevation sample at the geographic anchor")?;
    let half = extent * 0.5;
    let mut cells = Vec::new();
    let mut z = -half + step * 0.5;
    while z < half {
        let mut x = -half + step * 0.5;
        while x < half {
            let position = Vec3::new(x, 0.0, z);
            if let Some(height) = ground_height(grids, &anchor.topology, &terrain_anchor, position)
            {
                let slope = slope_at(grids, anchor, &terrain_anchor, position, step);
                let clear = x.hypot(z) <= CLEARANCE_M;
                let obstacle = !clear && slope > slope_limit;
                let block_height = if obstacle {
                    (slope * step).max(0.9).min(3.5)
                } else {
                    0.0
                };
                cells.push(PatchCell {
                    bevy_x: x,
                    bevy_z: z,
                    relative_height: height - origin,
                    obstacle,
                    block_height,
                });
            }
            x += step;
        }
        z += step;
    }
    if cells.is_empty() {
        return Err("elevation patch produced no samples".into());
    }
    Ok(GeoPatch {
        anchor: anchor.clone(),
        step,
        origin_elevation: origin,
        cells,
    })
}

pub(crate) fn spawn_patch(
    mut commands: Commands,
    patch: Res<GeoPatch>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    let cube = meshes.add(Cuboid::new(1.0, 1.0, 1.0));
    let obstacle_material = materials.add(StandardMaterial {
        base_color: Color::srgb(0.42, 0.38, 0.33),
        perceptual_roughness: 0.95,
        ..default()
    });
    let tints: Vec<_> = (0..9)
        .map(|bucket| {
            let relative = (bucket as f32 - 4.0) * 2.5;
            materials.add(StandardMaterial {
                base_color: tint(relative),
                perceptual_roughness: 1.0,
                ..default()
            })
        })
        .collect();
    let step = patch.step;
    for cell in &patch.cells {
        if cell.obstacle {
            commands.spawn((
                Name::new("Tile obstacle"),
                crate::world::WorldElement,
                GeoObstacle,
                RigidBody::Static,
                Collider::cuboid(1.0, 1.0, 1.0),
                Restitution::ZERO,
                Mesh3d(cube.clone()),
                MeshMaterial3d(obstacle_material.clone()),
                Transform::from_xyz(cell.bevy_x, cell.block_height * 0.5, cell.bevy_z)
                    .with_scale(Vec3::new(step * 0.85, cell.block_height, step * 0.85)),
            ));
        } else {
            let bucket = ((cell.relative_height / 2.5) + 4.0).clamp(0.0, 8.0) as usize;
            commands.spawn((
                Name::new("Tile tint"),
                crate::world::WorldElement,
                GeoTint,
                Mesh3d(cube.clone()),
                MeshMaterial3d(tints[bucket].clone()),
                Transform::from_xyz(cell.bevy_x, 0.02, cell.bevy_z).with_scale(Vec3::new(
                    step * 0.98,
                    0.04,
                    step * 0.98,
                )),
            ));
        }
    }
}

fn tint(relative: f32) -> Color {
    let t = (relative / 10.0).clamp(-1.0, 1.0);
    if t >= 0.0 {
        Color::srgb(0.42 + 0.28 * t, 0.52 - 0.12 * t, 0.30 - 0.08 * t)
    } else {
        Color::srgb(0.22, 0.40 + 0.12 * -t, 0.30)
    }
}

fn sample_step(tile_metres: f32) -> f32 {
    (tile_metres / 256.0).clamp(2.0, 12.0)
}

fn slope_at(
    grids: &HeightGrids,
    anchor: &GeoAnchor,
    terrain_anchor: &TerrainAnchor,
    position: Vec3,
    step: f32,
) -> f32 {
    let Some(center) = ground_height(grids, &anchor.topology, terrain_anchor, position) else {
        return 0.0;
    };
    let mut rise = 0.0_f32;
    for offset in [
        Vec3::X * step,
        Vec3::NEG_X * step,
        Vec3::Z * step,
        Vec3::NEG_Z * step,
    ] {
        if let Some(height) =
            ground_height(grids, &anchor.topology, terrain_anchor, position + offset)
        {
            rise = rise.max((height - center).abs());
        }
    }
    rise / step
}

fn tiles_for_square(anchor: &GeoAnchor, extent: f32) -> Vec<TileRef> {
    let half = extent * 0.5;
    let stride = (anchor.tile_metres * 0.5).max(1.0);
    let mut tiles = Vec::new();
    let mut z = -half;
    loop {
        let mut x = -half;
        loop {
            if let Some(tile) = tile_at(anchor, Vec3::new(x, 0.0, z))
                && !tiles
                    .iter()
                    .any(|existing: &TileRef| existing.key == tile.key)
            {
                tiles.push(tile);
            }
            if x >= half - 0.01 {
                break;
            }
            x = (x + stride).min(half);
        }
        if z >= half - 0.01 {
            break;
        }
        z = (z + stride).min(half);
    }
    tiles
}

fn tile_at(anchor: &GeoAnchor, position: Vec3) -> Option<TileRef> {
    let (key, _, _) = absolute_tile(anchor, position)?;
    let shift = u32::from(anchor.zoom - anchor.topology.base_zoom);
    let provider_x = (anchor.topology.anchor_x << shift) + key.x;
    let provider_z = (anchor.topology.anchor_z << shift) + key.z;
    Some(TileRef {
        key,
        provider_x,
        provider_z,
    })
}

fn absolute_tile(anchor: &GeoAnchor, position: Vec3) -> Option<(TileKey, f32, f32)> {
    let abs = position - anchor.world_offset;
    let zoom = anchor.zoom;
    let shift = zoom - anchor.topology.base_zoom;
    let size = f64::from(anchor.topology.tile_size) / f64::from(1u32 << u32::from(shift));
    if !size.is_finite() || size <= 0.0 {
        return None;
    }
    let tx = (f64::from(abs.x) / size).floor() as i32;
    let tz = (f64::from(abs.z) / size).floor() as i32;
    let u = ((f64::from(abs.x) - f64::from(tx) * size) / size) as f32;
    let v = ((f64::from(abs.z) - f64::from(tz) * size) / size) as f32;
    (u.is_finite() && v.is_finite()).then_some((TileKey { zoom, x: tx, z: tz }, u, v))
}

#[cfg(test)]
fn slippy_tile(latitude: f64, longitude: f64, zoom: u8) -> (i32, i32) {
    let n = 2f64.powi(i32::from(zoom));
    let x = (longitude + 180.0) / 360.0 * n;
    let lat = latitude.to_radians();
    let y = (1.0 - (lat.tan() + 1.0 / lat.cos()).ln() / std::f64::consts::PI) / 2.0 * n;
    (x.floor() as i32, y.floor() as i32)
}

fn terrarium_template() -> String {
    "https://s3.amazonaws.com/elevation-tiles-prod/terrarium/:zoom:/:x:/:y:.png".into()
}

fn height_cache(cache: &Path, tile: &TileRef) -> PathBuf {
    cache
        .join("heightmap")
        .join(tile.key.zoom.to_string())
        .join(tile.provider_x.to_string())
        .join(format!("{}.png", tile.provider_z))
}

fn seed_bundle(cache: &Path, tiles: &[TileRef]) -> Result<(), String> {
    for tile in tiles {
        if tile.key.zoom != BUNDLE_ZOOM
            || tile.provider_x != BUNDLE_X
            || tile.provider_z != BUNDLE_Z
        {
            continue;
        }
        for kind in ["texture", "heightmap", "normals"] {
            let dest = cache
                .join(kind)
                .join(tile.key.zoom.to_string())
                .join(tile.provider_x.to_string())
                .join(format!("{}.png", tile.provider_z));
            if dest.exists() {
                continue;
            }
            if let Some(parent) = dest.parent() {
                std::fs::create_dir_all(parent).map_err(|error| error.to_string())?;
            }
            std::fs::copy(BUNDLE_PNG, &dest)
                .map_err(|error| format!("copy bundled elevation tile: {error}"))?;
        }
    }
    Ok(())
}

fn env_flag(name: &str) -> bool {
    matches!(
        std::env::var(name).as_deref(),
        Ok("1") | Ok("true") | Ok("TRUE")
    )
}

fn env_flag_default(name: &str, default: bool) -> bool {
    match std::env::var(name).as_deref() {
        Ok("0") | Ok("false") | Ok("FALSE") => false,
        Ok("1") | Ok("true") | Ok("TRUE") => true,
        _ => default,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use bevytiles::height::HeightGrid;

    #[test]
    fn anchor_matches_slippy_map_and_local_axes() {
        for (latitude, longitude) in [
            (DEFAULT_LATITUDE, DEFAULT_LONGITUDE),
            (0.0, 0.0),
            (-33.8688, 151.2093),
            (46.206889, 9.497194),
        ] {
            let anchor = GeoAnchor::try_new(latitude, longitude, 15).unwrap();
            let tile = tile_at(&anchor, Vec3::ZERO).unwrap();
            assert_eq!(
                (tile.provider_x, tile.provider_z),
                slippy_tile(latitude, longitude, 15),
                "provider tile drifted at {latitude},{longitude}"
            );
            let (x, y) = anchor.local_xy(latitude, longitude);
            assert!(x.abs() < 1e-6 && y.abs() < 1e-6);
            let (north, west) = anchor.local_xy(latitude + 0.0002, longitude);
            assert!((north - 0.0002 * METRES_PER_DEGREE).abs() < 1e-6);
            assert!(west.abs() < 1e-6, "north must stay on +x, got y={west}");
            let (_, east_as_west) = anchor.local_xy(latitude, longitude + 0.0002);
            assert!(
                east_as_west < 0.0,
                "east of the anchor is robotics -y (Bevy +X)"
            );
        }
        assert_eq!(
            slippy_tile(DEFAULT_LATITUDE, DEFAULT_LONGITUDE, BUNDLE_ZOOM),
            (BUNDLE_X, BUNDLE_Z)
        );
    }

    #[test]
    fn flat_grid_has_no_obstacles_and_a_ridge_does() {
        let anchor = GeoAnchor::try_new(0.0, 0.0, 15).unwrap();
        let key = tile_at(&anchor, Vec3::ZERO).unwrap().key;
        let mut flat = HeightGrids::default();
        flat.0.insert(
            key,
            HeightGrid {
                w: 2,
                h: 2,
                samples: vec![32_768 + 40; 4],
            },
        );
        let patch = build_patch(&flat, &anchor, 40.0, 0.35).unwrap();
        assert!((patch.origin_elevation - 40.0).abs() < 1e-3);
        assert!(patch.cells.iter().all(|cell| !cell.obstacle));

        // Sit mid-tile so bilinear sampling is not clamped to a single edge texel.
        let anchor = GeoAnchor::try_new(0.005, 0.005, 15).unwrap();
        let key = tile_at(&anchor, Vec3::ZERO).unwrap().key;
        let mut ridge = HeightGrids::default();
        ridge.0.insert(
            key,
            HeightGrid {
                w: 2,
                h: 2,
                samples: vec![32_768 + 10, 32_768 + 4_000, 32_768 + 10, 32_768 + 4_000],
            },
        );
        let patch = build_patch(&ridge, &anchor, 40.0, 0.35).unwrap();
        assert!(
            patch.cells.iter().any(|cell| cell.obstacle),
            "a tall ridge must create obstacles"
        );
        assert!(
            patch
                .cells
                .iter()
                .filter(|cell| cell.bevy_x.hypot(cell.bevy_z) <= CLEARANCE_M)
                .all(|cell| !cell.obstacle),
            "spawn clearance stays open"
        );
    }

    #[test]
    fn bundled_gmu_tile_builds_a_mixed_patch_offline() {
        let directory = std::env::temp_dir().join(format!(
            "terra-tiles-{}-{}",
            std::process::id(),
            std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap()
                .as_nanos()
        ));
        let _ = std::fs::remove_dir_all(&directory);
        let config = GeoTileConfig {
            enabled: true,
            latitude: DEFAULT_LATITUDE,
            longitude: DEFAULT_LONGITUDE,
            zoom: BUNDLE_ZOOM,
            fetch: false,
            cache_dir: directory.clone(),
            slope_limit: 0.35,
        };
        let patch = load_patch(&config, 100.0).unwrap();
        let _ = std::fs::remove_dir_all(&directory);
        assert!(
            (110.0..=160.0).contains(&patch.origin_elevation),
            "origin elevation {}",
            patch.origin_elevation
        );
        let obstacles = patch.cells.iter().filter(|cell| cell.obstacle).count();
        assert!(obstacles > 4, "obstacles={obstacles}");
        assert!(
            obstacles * 4 < patch.cells.len(),
            "obstacles={obstacles} cells={}",
            patch.cells.len()
        );
        assert!(patch.cells.iter().any(|cell| !cell.obstacle));
        // Documented demo goal: 12 m north. Obstacle centres must stay outside the chassis.
        let goal_x = 12.0_f32;
        let goal_y = 0.0_f32;
        let mut nearest = f32::MAX;
        for cell in &patch.cells {
            if !cell.obstacle {
                continue;
            }
            let along = -cell.bevy_z * goal_x + -cell.bevy_x * goal_y;
            let cross = -cell.bevy_z * goal_y - -cell.bevy_x * goal_x;
            let progress = along / 12.0;
            if progress > -1.0 && progress < 14.0 {
                nearest = nearest.min(cross.abs() / 12.0);
            }
        }
        assert!(
            nearest > 2.4,
            "12 m north is blocked; nearest obstacle centre is {nearest:.2} m off the path"
        );
    }

    #[test]
    fn disabled_config_accepts_placeholder_coordinates() {
        assert!(GeoTileConfig::default().validate().is_ok());
        assert!(
            GeoTileConfig {
                enabled: true,
                zoom: 4,
                ..GeoTileConfig::default()
            }
            .validate()
            .is_err()
        );
    }
}
