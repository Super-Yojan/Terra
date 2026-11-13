# Architecture

> [!TIP]
> **TL;DR**
> ARGOS decides. Terra drives. Zorvane pretends to be the world.
> Onboard loop: IMU and VIO, then a velocity PI, then left and right effort.
> Zero effort coasts. It does not brake.

![Top view of body axes: plus X forward, plus Y left](assets/body-axes.svg)

*Every crate uses these axes. Timestamps are seconds on one clock.*

```mermaid
flowchart TB
  IMU[IMU] --> State[terra-state]
  VIO[VIO] --> State
  State --> Ctrl[terra-control]
  Goal[terra-waypoint] --> Auto[terra-autonomy]
  Map[terra-mapping] --> Nav[terra-navigation]
  Nav --> Exp[terra-exploration]
  Exp --> Auto
  Nav --> Auto
  Auto --> Ctrl
  Ctrl --> Mot[terra-motors]
  Mot --> PWM[PWM or Zorvane forces]
```

*Onboard loop. The Pi does not link these crates. It speaks Bluetooth in Python.*

## Two phone builds

| Target | Hardware link |
| --- | --- |
| iOS Simulator | **Simulated rover** only. No Bluetooth radio. |
| iPhone | Bluetooth. Path to a rover. |

`#if targetEnvironment(simulator)` chooses the section. Neither build connects to Zorvane.

**Simulated rover** is a motor plant inside the app. It is on both targets. It is not Zorvane.

![Placeholder wireframe of the iPhone Bluetooth section](assets/phone-iphone.svg)

*Device build.*

## Pi and Zenoh

![Phone, Bluetooth, Pi, Fusion HAT](assets/pi-stack.svg)

*Customer pairing uses the USR button. Arming is a later step.*

```mermaid
flowchart LR
  Dash[ControlPlane] -->|status on loopback| Local[tcp/127.0.0.1 only]
```

*A listening `ControlPlane` refuses every address except loopback.*

## Planned

![Placeholder of the planned tap-to-configure popup](assets/phone-tap.svg)

*#27 is not built. `rover.usdz` orbits and zooms. It does not edit actuators.*

## Current platform split

ARGOS is the React web operator interface backed by a Rust Axum server. The server
bridges browser commands and telemetry to Zenoh. ARGOS Swift is a legacy fallback;
Unity is a historical prototype. TerraPhone iOS remains the physical vehicle's
sensor, Bluetooth and shared Rust runtime. Android is an additional mobile adapter.
Zorvane owns simulation; Terra owns onboard control, mapping and autonomy.
The VLM evaluator under `tools/vlm-eval` is offline research infrastructure.
