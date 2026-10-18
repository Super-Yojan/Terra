# terra-mobile

!!! tip "TL;DR"
    UniFFI boundary for TerraPhone.
    Other crates stay free of UniFFI.
    Bindings are generated on a Mac and are not committed.

```mermaid
flowchart LR
  Swift[SwiftUI] --> FFI[terra-mobile]
  FFI --> Ctrl[control and autonomy]
  FFI --> Map[mapping]
  FFI --> Zen[Zenoh client]
  FFI --> Act[actuators]
```

*[Build steps](../phone/build.md). Deployment target is iOS 17.*

[API](https://super-yojan.dev/Terra/api/terra_mobile/index.html)

| Swift name | Rust |
| --- | --- |
| `MobileController` | Estimator, PI, arbiter, recorder. |
| `MobileWaypoint` | `terra-waypoint`. |
| `MobileOccupancyMap` | `terra-mapping`. |
| `MobileZenohClient` | `RoverConnection`. Simulator path. |
| `host_dashboard` | Loopback `ControlPlane`. |
| `actuator_route` | `route_commands`. |

![Wireframe of the screen that calls these objects](../assets/phone-home.svg){ width="240" }

*Placeholder. The real view is SwiftUI in `ContentView`.*

When the dashboard is open, ticks can publish `autonomy/status`, `goal/status`, `exploration/status` while a run is active, `map/occupancy` (at most every 0.2 s), and `pose`. The mission picker can request `explore` with a budget in seconds.

```sh
./scripts/build-ios.sh
./scripts/check-swift.sh
```
