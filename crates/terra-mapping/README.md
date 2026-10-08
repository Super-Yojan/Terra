# terra-mapping

A dependency-light, rolling 2D occupancy grid for Terra. Depends only on `terra-types`; usable directly from Rust and through `terra-mobile`'s `MobileOccupancyMap` UniFFI object.

Default map: 20 × 20 metres, 10 cm cells, unknown initially. `recenter(x, y)` shifts the window in whole cells and preserves overlapping world evidence. Call `integrate_depth(timestamp, intrinsics, camera_pose, depth_metres)`, then `snapshot()` for row-major probabilities: -1 unknown, 0–100 occupied probability. Rows increase in world +Y; origin is the lower-left cell boundary. Cell centres are `(origin_x + (col + 0.5) * resolution, origin_y + (row + 0.5) * resolution)`.

Depth is **axial Z distance**, not ray length. Optical camera axes are X right, Y down, Z forward. Camera quaternion rotates optical vectors into metric, Z-up world axes. Supply the camera pose at frame exposure time and calibrated intrinsics/extrinsics. Timestamps must increase; recenter around the rover before integrating. Invalid inputs return errors before changing the map. NaN, infinite, nonpositive and out-of-range depths leave space unknown.

Rays mark visible cells free and endpoints occupied within the configurable obstacle height band relative to the world ground plane. Ground endpoints mark free space; overhead endpoints are ignored. Evidence accumulates as bounded log odds. Updates are deduplicated per frame, with hits winning conflicts. Rays stop at the grid boundary; cells behind an obstacle remain unknown.

```sh
cargo test -p terra-mapping
cargo run -p terra-mapping --example wall
```

This is a local mapping building block, not SLAM, terrain reconstruction, navigation or dynamic-object tracking. It assumes a consistent pose frame and flat ground reference. Evidence does not decay automatically; call `clear()` on localization resets. Moving objects require repeated free observations to clear. Depth subsampling is configurable and can miss thin obstacles.

[Zorvane](https://super-yojan.dev/Zorvane/) feeds each rover’s local map through `TerraOccupancyMapPlugin`. Each GPU depth copy carries its extracted camera transform and simulation exposure time; asynchronous completion retains that pairing. Intrinsics are derived from the configured field of view and resolution. Map state lives in the rover’s `RoverOccupancyMap` component and is removed with that rover. CPU receipt time remains available separately. Maps are currently consumed locally, with no Zenoh map topic. Zorvane publishes the exposure camera pose and rover body pose inside each depth frame so another process can run this same integrator. The iOS app displays the map and supplies simulated room depth, ARKit scene depth on supported devices, or that Zorvane depth stream while a Zenoh session is open. Prefix stays `terra/rover`. ARKit pose tracking alone supplies no depth map. The phone adapter rescales intrinsics, filters low-confidence returns and uses the same ARFrame pose and timestamp. Its initial camera-height assumption requires calibration; Zenoh mode uses the published camera height instead. See `docs/MOBILE_CONTROL.md`. The UniFFI API is available after regenerating bindings with `./scripts/build-ios.sh`.
