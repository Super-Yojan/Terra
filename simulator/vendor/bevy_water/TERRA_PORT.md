# Local Bevy 0.19 compatibility port

Source: the published `bevy_water` 0.18.1 crate from
https://crates.io/crates/bevy_water/0.18.1, upstream repository
https://github.com/Neopallium/bevy_water, recorded upstream commit
`da5fc110edf2d031831e72794f3793e44efcd8f9`.

The published version targets Bevy 0.18. Terra uses Bevy 0.19.1. This vendored
copy preserves the upstream library source, shaders, README, original manifest
(`Cargo.toml.orig`), author attribution, and declared `MIT OR Apache-2.0`
license metadata. Neither the published archive nor the inspected upstream
repository included standalone license files; none have been fabricated here.

Changes:

- Set the Bevy dependency to 0.19.
- Omit upstream example-only development dependencies from the active manifest.
- Make the material binding mutable in `src/water.rs` for Bevy 0.19's mutable
  asset wrapper.

Terra enables only `embed_shaders`. The optional easing and inspector integrations
retain their upstream versions and are not validated for Bevy 0.19. Do not enable
those features without porting their dependencies too.

The pond material has been checked in the simulator's real GPU rendering test
alongside voxel terrain, procedural trees and the rover depth prepass.
