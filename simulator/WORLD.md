# Terra world

`TerraWorldPlugin::default()` creates a 100 m square practice environment:

- Two connected roads with dashed lane markings, sidewalks and crosswalks.
- Twelve buildings with seeded heights, windows and doors.
- Up to 96 procedural trees, with eight shared mesh variants.
- A small animated pond with stone borders.
- Voxel hills around the outside of the map.

The first rover starts at the clear intersection. Additional rovers spawn along
the roads; see [ZENOH.md](ZENOH.md) for configurable fleet size and remote control. Buildings, tree trunks, pond borders
and voxel hill meshes have static Avian colliders. Roads sit on the flat ground
collider. Leaves and branches are visual geometry; water has no buoyancy model.
The pond is a surface over the flat ground, rather than an excavated basin.

## Configuration

Pass a `WorldConfig` to `TerraWorldPlugin`, for example:

```rust
TerraWorldPlugin {
    config: WorldConfig {
        size: 100.0,
        landscape: LandscapeConfig {
            seed: 123,
            tree_count: 120,
            road_width: 6.0,
            pond: true,
            voxel_hills: true,
            ..default()
        },
        ..default()
    },
}
```

Import `world::WorldConfig` and `landscape::LandscapeConfig`. Set
`landscape.enabled = false` for the previous flat practice world; set
`show_grid = true` to display its grid. Landscape sizes are limited to 40–512 m.
Dense tree requests may produce fewer trees to preserve clearance and spacing.
Identical seeds reproduce building heights, tree placement, meshes, and hills.

## Libraries

- [bevy_voxel_world](https://github.com/splashdust/bevy_voxel_world), version
  0.17.0: streams the perimeter hills and generates mesh-matched colliders.
- [bevy_procedural_tree](https://github.com/Affinator/bevy_procedural_tree),
  version 0.4.0: generates branching trunks and leaf meshes.
- [bevy_water](https://crates.io/crates/bevy_water), version 0.18.1: supplies
  the pond material and wave animation, using the local Bevy 0.19 port described
  in `vendor/bevy_water/TERRA_PORT.md`.
- [bevytiles](https://crates.io/crates/bevytiles), version 0.1.0: supplies the
  offline height-grid representation and elevation lookup used to place the
  environment. The current grids are flat and match the ground collider. This
  integration does not download geographic or satellite tiles; a real-world
  location and terrain collision pipeline would be needed for that mode.

## Verification

From `simulator`, run `cargo test` and
`cargo clippy --all-targets -- -D warnings`. GPU tests are ignored by default;
run `cargo test -- --include-ignored --test-threads=1` on a graphics-capable host.
The GPU world test verifies that voxel colliders load, the rover receives finite
depth readings, and the overview camera produces an image. It writes its capture
to `/tmp/terra-world-preview.png`.

![World preview](docs/world-preview.png)
