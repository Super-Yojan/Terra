//! Finite voxel hills around the town, with mesh-matched static Avian colliders.
use avian3d::prelude::*;
use bevy::prelude::*;
use bevy_voxel_world::{custom_meshing::generate_chunk_mesh_for_shape, prelude::*};

#[derive(Resource, Clone)]
pub struct TerraVoxelTerrain {
    pub size: f32,
    pub seed: u64,
}
impl Default for TerraVoxelTerrain {
    fn default() -> Self {
        Self {
            size: 100.0,
            seed: 42,
        }
    }
}
#[derive(Component, Clone)]
pub struct VoxelTerrainChunk;
#[derive(Bundle, Clone)]
pub struct VoxelColliderBundle {
    body: RigidBody,
    collider: Collider,
    restitution: Restitution,
    marker: VoxelTerrainChunk,
}
impl TerraVoxelTerrain {
    pub fn voxel_at(&self, position: IVec3) -> WorldVoxel<u8> {
        let border = position.x.abs().max(position.z.abs()) as f32 / self.size;
        if !(0.43..0.50).contains(&border) || position.y < 0 {
            return WorldVoxel::Air;
        }
        let phase = (self.seed % 1000) as f32 * 0.01;
        let wave =
            (position.x as f32 * 0.14 + phase).sin() * (position.z as f32 * 0.11 - phase).cos();
        let height = (2.0 + (border - 0.43) * 65.0 + wave * 1.5).max(1.0) as i32;
        if position.y < height {
            WorldVoxel::Solid(if position.y == height - 1 { 0 } else { 1 })
        } else {
            WorldVoxel::Air
        }
    }
}
impl VoxelWorldConfig for TerraVoxelTerrain {
    type MaterialIndex = u8;
    type ChunkUserBundle = VoxelColliderBundle;
    fn spawning_distance(&self) -> u32 {
        (self.size / 64.0).ceil() as u32 + 2
    }
    fn min_despawn_distance(&self) -> u32 {
        self.spawning_distance()
    }
    fn chunk_spawn_strategy(&self) -> ChunkSpawnStrategy {
        ChunkSpawnStrategy::Close
    }
    fn chunk_despawn_strategy(&self) -> ChunkDespawnStrategy {
        ChunkDespawnStrategy::FarAway
    }
    fn spawning_rays(&self) -> usize {
        0
    }
    fn max_spawn_per_frame(&self) -> usize {
        // The upstream scheduler bounds its pending queue, including protected chunks.
        // Keep that queue large enough; worker concurrency below bounds meshing work.
        10000
    }
    fn max_active_chunk_threads(&self) -> usize {
        4
    }
    fn texture_index_mapper(&self) -> TextureIndexMapperFn<u8> {
        std::sync::Arc::new(|material| if material == 0 { [0, 1, 1] } else { [1, 1, 1] })
    }
    fn voxel_lookup_delegate(&self) -> VoxelLookupDelegate<u8> {
        let terrain = self.clone();
        Box::new(move |_, _, _| {
            let terrain = terrain.clone();
            Box::new(move |position, _| terrain.voxel_at(position))
        })
    }
    fn chunk_meshing_delegate(&self) -> ChunkMeshingDelegate<u8, VoxelColliderBundle> {
        Some(Box::new(|position, _, _, _, _| {
            Box::new(move |voxels, data_shape, mesh_shape, mapper| {
                let mesh =
                    generate_chunk_mesh_for_shape(voxels, position, data_shape, mesh_shape, mapper);
                let bundle =
                    Collider::trimesh_from_mesh(&mesh).map(|collider| VoxelColliderBundle {
                        body: RigidBody::Static,
                        collider,
                        restitution: Restitution::ZERO,
                        marker: VoxelTerrainChunk,
                    });
                (mesh, bundle)
            })
        }))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn hills_stay_outside_town_and_inside_world() {
        let terrain = TerraVoxelTerrain::default();
        assert!(matches!(terrain.voxel_at(IVec3::ZERO), WorldVoxel::Air));
        assert!(matches!(
            terrain.voxel_at(IVec3::new(20, 0, 20)),
            WorldVoxel::Air
        ));
        assert!(matches!(
            terrain.voxel_at(IVec3::new(46, 0, 20)),
            WorldVoxel::Solid(_)
        ));
        assert!(matches!(
            terrain.voxel_at(IVec3::new(51, 0, 20)),
            WorldVoxel::Air
        ));
        assert!(matches!(
            terrain.voxel_at(IVec3::new(46, 20, 20)),
            WorldVoxel::Air
        ));
    }
}
