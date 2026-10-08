# terra-mapping

A rolling 2D occupancy grid. It depends only on `terra-types`. TerraPhone uses it through `MobileOccupancyMap`. Zorvane can run the same integrator on depth frames that carry an exposure pose.

[API](https://super-yojan.dev/Terra/api/terra_mapping/index.html) · [crate readme](https://github.com/Super-Yojan/Terra/blob/main/crates/terra-mapping/README.md)

## Defaults

`MapConfig` builds a **20 m × 20 m** grid of **10 cm** cells (`width` and `height` 200, `resolution` 0.1). Cells start unknown. Obstacle heights are 0.15 m to 2.0 m above the world ground plane. `max_range` is 15 m. `pixel_stride` is 4, so thin obstacles can be missed.

`recenter(x, y)` shifts the window by whole cells and keeps overlapping evidence. Evidence does not decay. Call `clear()` when localization resets.

## Depth

`integrate_depth` expects **axial Z distance**, which is not ray length. Optical axes are X right, Y down, Z forward. The camera quaternion rotates optical vectors into a metric, Z-up world. Pass the pose at exposure time. Timestamps must increase. Invalid input returns an error and leaves the map unchanged. NaN, infinite, non-positive, and out-of-range depths leave those cells unknown.

Rays mark visible cells free and endpoints occupied inside the height band. Ground endpoints are free. Overhead endpoints are ignored. Hits win conflicts inside one frame. Rays stop at the grid boundary, so cells behind an obstacle stay unknown.

`snapshot()` returns row-major probabilities. **−1** is unknown. **0–100** is occupied probability. Rows increase in world +Y. The origin is the lower-left cell boundary. Cell centres are `(origin_x + (col + 0.5) * resolution, origin_y + (row + 0.5) * resolution)`.

## Where the map is shown

The phone draws this grid. Simulated mode synthesizes room depth. Phone mode uses ARKit scene depth when the device supports it, with an assumed camera height of 0.5 m that still needs calibration. Simulator Zenoh mode integrates `terra/rover/<id>/camera/depth` and uses the published camera height instead. ARKit pose tracking alone does not produce a depth map.

Zorvane's in-process map is local to that process. The phone publishes an observed snapshot on `map/occupancy` when its loopback dashboard is running. See [Zenoh](../zenoh.md).

```sh
cargo test -p terra-mapping
cargo run -p terra-mapping --example wall
```

This crate is a local mapping block. It assumes a consistent pose frame and a flat ground reference. It does not track dynamic objects by itself: a moving object clears only after repeated free observations.
