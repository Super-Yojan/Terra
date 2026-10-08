# Architecture

!!! tip "TL;DR"
    ARGOS decides. Terra drives. Zorvane pretends to be the world.
    Onboard loop: IMU and VIO, then a velocity PI, then left and right effort.
    Zero effort coasts. It does not brake.

![Top view of body axes: plus X forward, plus Y left](assets/body-axes.svg)

*Every crate uses these axes. Timestamps are seconds on one clock.*

```mermaid
flowchart TB
  IMU[IMU] --> State[terra-state]
  VIO[VIO] --> State
  State --> Ctrl[terra-control]
  Goal[terra-waypoint] --> Auto[terra-autonomy]
  Map[terra-mapping] --> Nav[terra-navigation]
  Nav --> Auto
  Auto --> Ctrl
  Ctrl --> Mot[terra-motors]
  Mot --> PWM[PWM or Zorvane forces]
```

*Onboard loop. The Pi does not link these crates. It speaks Bluetooth in Python.*

## Two phone builds

![Placeholder wireframe of the Simulator Zenoh section](assets/phone-simulator.svg){ width="260" }

*Not a Simulator capture. No Xcode in this docs build. Labels match `ContentView`.*

| Target | Zenoh to Zorvane | Bluetooth |
| --- | --- | --- |
| iOS Simulator | Shown. Default `tcp/127.0.0.1:7447`. | Hidden. No radio. |
| iPhone | Hidden. Saved endpoint is deleted. | Shown. Path to a rover. |

`#if targetEnvironment(simulator)` chooses the section. There is no setting that turns Zenoh on for an iPhone.

**Simulated rover** is a motor plant inside the app. It is on both targets. It is not Zorvane.

![Placeholder wireframe of the iPhone Bluetooth section](assets/phone-iphone.svg){ width="260" }

*Device build. Bevy controls are compiled out.*

## Pi and Zenoh

![Phone, Bluetooth, Pi, Fusion HAT](assets/pi-stack.svg)

*Customer pairing uses the USR button. Arming is a later step.*

```mermaid
flowchart LR
  Client[RoverConnection client] -->|cmd_vel and actions| Peer[Zorvane or peer]
  Peer -->|depth and autonomy/status| Client
  Dash[ControlPlane] -->|status on loopback| Local[tcp/127.0.0.1 only]
```

*`RoverConnection` is the Simulator client. `ControlPlane` refuses every address except loopback.*

## Planned

![Placeholder of the planned tap-to-configure popup](assets/phone-tap.svg){ width="260" }

*#27 is not built. `rover.usdz` orbits and zooms. It does not edit actuators.*
