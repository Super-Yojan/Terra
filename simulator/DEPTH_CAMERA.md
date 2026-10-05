# Terra front depth camera

`TerraDepthCameraPlugin` is registered in `main.rs`. It mounts one sensor as a
child of each newly spawned `Rover`, ahead of the chassis front face, facing the
rover's local -Z direction. The mount uses metres and is independent of GLB scale.

Defaults: 256 × 192 pixels, 60° vertical field of view, 0.05–30 metre range,
0.2 metres above the body origin and 0.05 metres ahead of the chassis.
A grayscale preview appears at the lower right: nearby surfaces are bright;
faraway surfaces and missing returns are dark. WASD drives the rover and Space stops it.

Configure the sensor when adding the plugin:

```rust
TerraDepthCameraPlugin {
    config: DepthCameraConfig {
        resolution: UVec2::new(640, 480),
        vertical_fov: 60.0_f32.to_radians(),
        near: 0.05,
        far: 20.0,
        mount_height: 0.25,
        front_clearance: 0.05,
        preview: true,
    },
}
```

## Reading the sensor

Query `(&DepthCamera, &DepthFrame)` to access the latest completed capture.
`DepthFrame.depth_metres` is a row-major `Vec<f32>` indexed by
`row * frame.width + column`, with the first row at the top of the image.
An empty vector / sequence zero means no GPU frame has arrived yet.
`NaN` means sky, no return or a surface outside the configured range.

Values measure axial depth along camera-space -Z, in metres, rather than
Euclidean distance along each viewing ray. The camera frame has +X right and +Y up.
`DepthCamera.depth_texture` holds raw reverse-Z GPU values; these are not metres.
The plugin converts them using `near / raw_depth` and removes GPU row padding.

Readback is asynchronous and may lag the current rover pose.
`DepthFrame.sequence` counts completed captures and `received_at` records CPU
receipt time in Bevy elapsed seconds, not exposure time. Captures occur each render
frame. The sensor renders opaque and alpha-masked scene geometry, including meshes
without physics colliders; transparent materials may not produce depth returns.
This is an ideal sensor without noise or a physical camera housing.

## Verification

Run the ordinary test suite with `cargo test` from the simulator directory.
The real GPU wall-distance test is opt-in because CI may not have a graphics device:

```sh
cargo test gpu_captures_wall_distance_in_metres -- --ignored --nocapture
```

It checks the central pixel against a wall 2.45 metres from the mounted camera,
using a non-aligned image width to exercise GPU row padding.
