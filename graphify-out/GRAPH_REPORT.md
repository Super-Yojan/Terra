# Graph Report - .  (2026-10-05)

## Corpus Check
- 54 files · ~57,549 words
- Verdict: corpus is large enough that graph structure adds value.

## Summary
- 1193 nodes · 2312 edges · 55 communities (51 shown, 4 thin omitted)
- Extraction: 99% EXTRACTED · 1% INFERRED · 0% AMBIGUOUS · INFERRED: 31 edges (avg confidence: 0.8)
- Token cost: 0 input · 0 output

## Community Hubs (Navigation)
- [[_COMMUNITY_Motor Safety and PWM|Motor Safety and PWM]]
- [[_COMMUNITY_World Scene Assembly|World Scene Assembly]]
- [[_COMMUNITY_Rover Drive Control|Rover Drive Control]]
- [[_COMMUNITY_GPU Depth Rendering|GPU Depth Rendering]]
- [[_COMMUNITY_Elevation Tile Loading|Elevation Tile Loading]]
- [[_COMMUNITY_Water and Terrain Rendering|Water and Terrain Rendering]]
- [[_COMMUNITY_iOS Sensor and Control Runtime|iOS Sensor and Control Runtime]]
- [[_COMMUNITY_Waypoint Goal Protocol|Waypoint Goal Protocol]]
- [[_COMMUNITY_Landscape and Rover Tracking|Landscape and Rover Tracking]]
- [[_COMMUNITY_Zenoh Frame Transport|Zenoh Frame Transport]]
- [[_COMMUNITY_Mobile Brain and Benchmark|Mobile Brain and Benchmark]]
- [[_COMMUNITY_Physics and Collision Tests|Physics and Collision Tests]]
- [[_COMMUNITY_Water Material Shaders|Water Material Shaders]]
- [[_COMMUNITY_Occupancy Grid Core|Occupancy Grid Core]]
- [[_COMMUNITY_Simulator Motor Feedback|Simulator Motor Feedback]]
- [[_COMMUNITY_Mobile Waypoint Controller|Mobile Waypoint Controller]]
- [[_COMMUNITY_Shared Robotics Types|Shared Robotics Types]]
- [[_COMMUNITY_Voxel Terrain Generation|Voxel Terrain Generation]]
- [[_COMMUNITY_Depth Frame Readback|Depth Frame Readback]]
- [[_COMMUNITY_Simulator Fleet Interaction|Simulator Fleet Interaction]]
- [[_COMMUNITY_Water Wave Mathematics|Water Wave Mathematics]]
- [[_COMMUNITY_Depth to Occupancy Integration|Depth to Occupancy Integration]]
- [[_COMMUNITY_Water Image Utilities|Water Image Utilities]]
- [[_COMMUNITY_IMU and VIO Estimation|IMU and VIO Estimation]]
- [[_COMMUNITY_PI Velocity Control|PI Velocity Control]]
- [[_COMMUNITY_Mobile Mapping Adapter|Mobile Mapping Adapter]]
- [[_COMMUNITY_iOS Occupancy Map Interface|iOS Occupancy Map Interface]]
- [[_COMMUNITY_Mobile Zenoh Adapter|Mobile Zenoh Adapter]]
- [[_COMMUNITY_Depth and Goal Packets|Depth and Goal Packets]]
- [[_COMMUNITY_Simulator Integration Tests|Simulator Integration Tests]]
- [[_COMMUNITY_Command Validation and Watchdog|Command Validation and Watchdog]]
- [[_COMMUNITY_Python Zenoh Client|Python Zenoh Client]]
- [[_COMMUNITY_Core Robotics Architecture|Core Robotics Architecture]]
- [[_COMMUNITY_Practice Town and Geography|Practice Town and Geography]]
- [[_COMMUNITY_Zenoh Runtime Configuration|Zenoh Runtime Configuration]]
- [[_COMMUNITY_Mobile Transport and Safety|Mobile Transport and Safety]]
- [[_COMMUNITY_Depth Mapping Contracts|Depth Mapping Contracts]]
- [[_COMMUNITY_Phone Sensing and Calibration|Phone Sensing and Calibration]]
- [[_COMMUNITY_Waypoint Transport Semantics|Waypoint Transport Semantics]]
- [[_COMMUNITY_Town Preview Components|Town Preview Components]]
- [[_COMMUNITY_Transport Shared State|Transport Shared State]]
- [[_COMMUNITY_Water Runtime Parameters|Water Runtime Parameters]]
- [[_COMMUNITY_Motor Output Contract|Motor Output Contract]]
- [[_COMMUNITY_Depth Wall Verification|Depth Wall Verification]]
- [[_COMMUNITY_Simulator Depth Pipeline|Simulator Depth Pipeline]]
- [[_COMMUNITY_Swift Zenoh Verification|Swift Zenoh Verification]]
- [[_COMMUNITY_iOS Application Entry|iOS Application Entry]]
- [[_COMMUNITY_Terrain Raster Features|Terrain Raster Features]]
- [[_COMMUNITY_Swift Smoke Check|Swift Smoke Check]]
- [[_COMMUNITY_iOS Build Script|iOS Build Script]]
- [[_COMMUNITY_Swift Check Script|Swift Check Script]]

## God Nodes (most connected - your core abstractions)
1. `PhoneController` - 37 edges
2. `drive_goals()` - 21 edges
3. `WaveDirection` - 20 edges
4. `MotorAdapter` - 19 edges
5. `generate_landscape()` - 19 edges
6. `reconcile_fleet()` - 19 edges
7. `Option` - 16 edges
8. `publish_frames()` - 16 edges
9. `WaterMaterial` - 16 edges
10. `TerraVoxelTerrain` - 15 edges

## Surprising Connections (you probably didn't know these)
- `terra-waypoint` --references--> `North-west robotics frame`  [EXTRACTED]
  README.md → simulator/WORLD.md
- `Zenoh bridge` --references--> `Per-rover cmd_vel`  [EXTRACTED]
  simulator/ZENOH.md → crates/terra-transport/README.md
- `run_transport()` --calls--> `decode_goal()`  [INFERRED]
  simulator/src/zenoh_bridge.rs → crates/terra-waypoint/src/lib.rs
- `drive_goals()` --calls--> `encode_status()`  [INFERRED]
  simulator/src/zenoh_bridge.rs → crates/terra-waypoint/src/lib.rs
- `MobileWaypoint` --references--> `terra-waypoint`  [EXTRACTED]
  docs/MOBILE_CONTROL.md → README.md

## Import Cycles
- 1-file cycle: `crates/terra-mobile/src/waypoint.rs -> crates/terra-mobile/src/waypoint.rs`
- 1-file cycle: `crates/terra-mobile/src/transport.rs -> crates/terra-mobile/src/transport.rs`
- 1-file cycle: `crates/terra-state/src/lib.rs -> crates/terra-state/src/lib.rs`
- 1-file cycle: `crates/terra-waypoint/src/lib.rs -> crates/terra-waypoint/src/lib.rs`
- 1-file cycle: `simulator/src/geo.rs -> simulator/src/geo.rs`
- 1-file cycle: `simulator/src/zenoh_bridge.rs -> simulator/src/zenoh_bridge.rs`
- 1-file cycle: `simulator/vendor/bevy_water/src/water.rs -> simulator/vendor/bevy_water/src/water.rs`

## Hyperedges (group relationships)
- **Phone sensor-to-effort pipeline** — docs_mobile_control_sensorcontract, docs_mobile_control_estimator, docs_mobile_control_pi, terra_motors_readme_effort [EXTRACTED 1.00]
- **Exposure aligned depth mapping** — simulator_depth_camera_depthframe, simulator_zenoh_packet, terra_mapping_readme_mapmobile, docs_mobile_control_remotemap [EXTRACTED 1.00]
- **Latched mission goal contract** — simulator_zenoh_goal, readme_waypoint, simulator_zenoh_goalstatus, simulator_world_frame [EXTRACTED 1.00]

## Communities (55 total, 4 thin omitted)

### Community 0 - "Motor Safety and PWM"
Cohesion: 0.08
Nodes (35): Default, Display, Formatter, InputError, MotorOutput, Option, Result, Self (+27 more)

### Community 1 - "World Scene Assembly"
Cohesion: 0.08
Nodes (39): AlphaMode, EaseMethod, EasingType, Into, Name, App, Assets, Color (+31 more)

### Community 2 - "Rover Drive Control"
Cohesion: 0.07
Nodes (46): LinearVelocity, RoverDriveQuery, App, AssetServer, ButtonInput, Commands, Default, DriveCommand (+38 more)

### Community 3 - "GPU Depth Rendering"
Cohesion: 0.07
Nodes (51): DepthExposure, DepthView, NewRovers, RenderContext, RenderDevice, App, Assets, ChildOf (+43 more)

### Community 4 - "Elevation Tile Loading"
Cohesion: 0.09
Nodes (44): Path, PathBuf, Assets, Color, Commands, Default, HeightGrids, Mesh (+36 more)

### Community 5 - "Water and Terrain Rendering"
Cohesion: 0.10
Nodes (43): MeshMaterial3d, App, Assets, Commands, Default, Entity, Handle, HeightGrids (+35 more)

### Community 6 - "iOS Sensor and Control Runtime"
Cohesion: 0.09
Nodes (26): ARFrame, ARSession, ARSessionDelegate, ControlOutput, DispatchSourceTimer, Float, Double, Error (+18 more)

### Community 7 - "Waypoint Goal Protocol"
Cohesion: 0.13
Nodes (32): Default, InputError, Option, Result, Self, String, Value, Vec (+24 more)

### Community 8 - "Landscape and Rover Tracking"
Cohesion: 0.07
Nodes (34): FollowRovers, GeoTileConfig, LandscapeConfig, App, Assets, Commands, Default, Mesh (+26 more)

### Community 9 - "Zenoh Frame Transport"
Cohesion: 0.11
Nodes (30): Arc, Display, Drop, Error, Formatter, Instant, Mutex, Option (+22 more)

### Community 10 - "Mobile Brain and Benchmark"
Cohesion: 0.11
Nodes (25): ControllerConfig, Arc, From, InputError, MotorOutput, Mutex, Result, Self (+17 more)

### Community 11 - "Physics and Collision Tests"
Cohesion: 0.11
Nodes (28): AngularVelocity, Friction, LockedAxes, Mass, App, Collider, Default, DriveCommand (+20 more)

### Community 12 - "Water Material Shaders"
Cohesion: 0.09
Nodes (25): AsBindGroupShaderType, MaterialExtension, MaterialExtensionKey, MaterialExtensionPipeline, MeshVertexBufferLayoutRef, RenderPipelineDescriptor, ShaderRef, App (+17 more)

### Community 13 - "Occupancy Grid Core"
Cohesion: 0.14
Nodes (21): Default, Display, Error, Formatter, Option, Result, Self, Vec (+13 more)

### Community 14 - "Simulator Motor Feedback"
Cohesion: 0.09
Nodes (28): Added, ControlledRovers, App, Commands, Default, DriveConfig, Entity, Fixed (+20 more)

### Community 15 - "Mobile Waypoint Controller"
Cohesion: 0.14
Nodes (23): ControllerError, Arc, From, Mutex, Option, Result, Self, String (+15 more)

### Community 16 - "Shared Robotics Types"
Cohesion: 0.11
Nodes (17): Default, Display, Error, Formatter, Option, Result, Self, Health (+9 more)

### Community 17 - "Voxel Terrain Generation"
Cohesion: 0.08
Nodes (17): ChunkDespawnStrategy, ChunkMeshingDelegate, ChunkSpawnStrategy, IVec3, Collider, Default, Restitution, RigidBody (+9 more)

### Community 18 - "Depth Frame Readback"
Cohesion: 0.08
Nodes (25): On, ReadbackComplete, App, Assets, Commands, DepthCamera, DepthCameraConfig, Entity (+17 more)

### Community 19 - "Simulator Fleet Interaction"
Cohesion: 0.11
Nodes (27): GeoPatch, Local, RemoteRovers, RgbCamera, RgbFrame, RoverFleet, Arc, ButtonInput (+19 more)

### Community 20 - "Water Wave Mathematics"
Cohesion: 0.20
Nodes (22): Vec3, Vec2, Vec3, WaterParam<'w>, cubic_hermite_curve_2d(), fbm(), fbm_half(), fract() (+14 more)

### Community 21 - "Depth to Occupancy Integration"
Cohesion: 0.11
Nodes (22): CameraPose, App, ChildOf, Commands, DepthCamera, DepthFrame, Entity, GlobalTransform (+14 more)

### Community 22 - "Water Image Utilities"
Cohesion: 0.17
Nodes (17): App, Assets, AssetServer, Commands, Entity, Handle, Image, Plugin (+9 more)

### Community 23 - "IMU and VIO Estimation"
Cohesion: 0.22
Nodes (16): Default, InputError, Option, Result, Self, VelocityEstimate, ImuSample, EstimatorConfig (+8 more)

### Community 24 - "PI Velocity Control"
Cohesion: 0.20
Nodes (13): Default, InputError, MotorOutput, Option, Result, Self, VelocityEstimate, closes_the_loop_on_a_motor_plant_with_load_and_recovers_from_saturation() (+5 more)

### Community 25 - "Mobile Mapping Adapter"
Cohesion: 0.17
Nodes (14): Arc, From, LocalOccupancyMap, Mutex, Result, Self, Vec, MapError (+6 more)

### Community 26 - "iOS Occupancy Map Interface"
Cohesion: 0.17
Nodes (14): CGPoint, CGRect, CGSize, Int, Double, OccupancyGrid, SIMD3, String (+6 more)

### Community 27 - "Mobile Zenoh Adapter"
Cohesion: 0.15
Nodes (12): Arc, From, Option, Result, Self, String, Vec, RoverConnection (+4 more)

### Community 28 - "Depth and Goal Packets"
Cohesion: 0.19
Nodes (18): BodyPoseHeader, BTreeMap, CameraPoseHeader, DepthCameraConfig, DepthFrame, Instant, Option, Vec (+10 more)

### Community 29 - "Simulator Integration Tests"
Cohesion: 0.26
Nodes (10): App, RoverId, Self, depth_packet_pose_round_trips_into_the_phone_decoder(), drive_commands_are_isolated_between_rovers(), fleet_requests_validate_count_and_apply_to_the_resource(), mobile_adapter_drives_avian_and_disconnect_stops(), waypoint_goal_drives_north_and_publishes_status() (+2 more)

### Community 30 - "Command Validation and Watchdog"
Cohesion: 0.16
Nodes (12): Duration, Quat, DriveCommand, Vec3, accepts_finite_twist_and_rejects_invalid_or_oversized_messages(), BodyPoseHeader, CameraPoseHeader, decode_command() (+4 more)

### Community 31 - "Python Zenoh Client"
Cohesion: 0.21
Nodes (6): FrameTests, decode_frame(), encode_goal(), main(), Return wire metadata and a contiguous numpy array; NaN depth stays NaN., JSON body for terra/rover/<id>/goal. One publish; Terra latches it.

### Community 32 - "Core Robotics Architecture"
Cohesion: 0.27
Nodes (10): VIO anchored bounded IMU prediction, Differential-drive PI velocity controller, Monotonic IMU and VIO sensor contract, Synthetic Avian IMU and VIO feedback, terra-control, terra-mapping, terra-motors, terra-state (+2 more)

### Community 33 - "Practice Town and Geography"
Cohesion: 0.22
Nodes (10): Dynamic water waves, Local Bevy 0.19 water compatibility port, bevytiles, North-west robotics frame, GMU Johnson Center anchor, Terrarium elevation tile patch, bevy_procedural_tree, bevy_voxel_world (+2 more)

### Community 34 - "Zenoh Runtime Configuration"
Cohesion: 0.24
Nodes (9): Config, Default, Plugin, Result, String, decode_fleet_request(), run_transport(), TerraZenohPlugin (+1 more)

### Community 35 - "Mobile Transport and Safety"
Cohesion: 0.22
Nodes (9): Stale iOS Zenoh transport statement, Swift UniFFI Rust Zenoh integration verification, terra-mobile, terra-transport, UniFFI, Per-rover cmd_vel, RoverConnection, Monotonic command lease (+1 more)

### Community 36 - "Depth Mapping Contracts"
Cohesion: 0.22
Nodes (9): DepthFrame, GPU wall-distance verification, Posed depth frame packet, Unsynchronized RGB and depth streams, Axial Z depth, Rolling local occupancy grid, TerraOccupancyMapPlugin, Bounded log odds evidence (+1 more)

### Community 37 - "Phone Sensing and Calibration"
Cohesion: 0.38
Nodes (7): MobileWaypoint, Phone mounting calibration, Phone-local simulator occupancy map, TerraPhone, terra-waypoint, ARKit scene depth, MobileOccupancyMap

### Community 38 - "Waypoint Transport Semantics"
Cohesion: 0.29
Nodes (7): ARGOS goal-topic follow-up, Zenoh bridge, Latched per-rover waypoint goal, Waypoint goal status, Waypoint obstacle planning limitation, Simulator incoming command watchdog, eclipse-zenoh Python dependency

### Community 39 - "Town Preview Components"
Cohesion: 0.33
Nodes (7): Low-rise buildings, Central marked intersection, Green perimeter barriers, Cross-shaped road network, Tree clusters, Rectangular water feature, Isometric simulated town

### Community 40 - "Transport Shared State"
Cohesion: 0.29
Nodes (7): AtomicBool, AtomicU64, GoalCommand, LeasedCommand, Mutex, GoalInbox, Shared

### Community 41 - "Water Runtime Parameters"
Cohesion: 0.33
Nodes (5): GlobalWaveState, Res, Time, WaterParam, WaterSettings

### Community 42 - "Motor Output Contract"
Cohesion: 0.40
Nodes (5): MotorAdapter, Zero-effort coast, Signed wheel effort to PWM, ChassisPwm, Motor command watchdog

### Community 43 - "Depth Wall Verification"
Cohesion: 0.40
Nodes (4): Box, Error, Result, main()

### Community 44 - "Simulator Depth Pipeline"
Cohesion: 0.50
Nodes (4): Bevy Avian rover simulator, TerraDepthCameraPlugin, Asynchronous GPU depth readback, RoverFleet

### Community 46 - "iOS Application Entry"
Cohesion: 0.50
Nodes (3): App, Scene, TerraPhoneApp

### Community 47 - "Terrain Raster Features"
Cohesion: 0.67
Nodes (3): Branching ridge and valley patterns, Color-encoded terrain surface, Terrarium terrain raster

## Knowledge Gaps
- **332 isolated node(s):** `Bounded log odds evidence`, `TerraOccupancyMapPlugin`, `ARKit scene depth`, `ChassisPwm`, `Motor command watchdog` (+327 more)
  These have ≤1 connection - possible missing edges or undocumented components.
- **4 thin communities (<3 nodes) omitted from report** — run `graphify query` to explore isolated nodes.

## Suggested Questions
_Questions this graph is uniquely positioned to answer:_

- **Why does `MeshMaterial3d` connect `Water and Terrain Rendering` to `Landscape and Rover Tracking`, `World Scene Assembly`, `GPU Depth Rendering`, `Elevation Tile Loading`?**
  _High betweenness centrality (0.039) - this node is a cross-community bridge._
- **Why does `RoverBody` connect `Physics and Collision Tests` to `Rover Drive Control`?**
  _High betweenness centrality (0.032) - this node is a cross-community bridge._
- **Why does `Duration` connect `Command Validation and Watchdog` to `Zenoh Runtime Configuration`, `Physics and Collision Tests`?**
  _High betweenness centrality (0.031) - this node is a cross-community bridge._
- **What connects `Bounded log odds evidence`, `TerraOccupancyMapPlugin`, `ARKit scene depth` to the rest of the system?**
  _334 weakly-connected nodes found - possible documentation gaps or missing edges._
- **Should `Motor Safety and PWM` be split into smaller, more focused modules?**
  _Cohesion score 0.07783783783783783 - nodes in this community are weakly interconnected._
- **Should `World Scene Assembly` be split into smaller, more focused modules?**
  _Cohesion score 0.07627118644067797 - nodes in this community are weakly interconnected._
- **Should `Rover Drive Control` be split into smaller, more focused modules?**
  _Cohesion score 0.06704260651629072 - nodes in this community are weakly interconnected._
## Token Accounting Limitation

Agent usage counts were unavailable. Zero token values are placeholders, not measured zero cost.
