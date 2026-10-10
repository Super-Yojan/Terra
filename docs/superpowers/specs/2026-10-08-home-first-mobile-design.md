# Home-first Terra mobile interface

Date: 2026-10-08
Status: Implemented; software checks complete, physical hardware acceptance pending.

## Confirmed purpose

The phone app's main tasks are connecting to its rover and connecting that rover to the ARGOS fleet management dashboard. Most everyday actions belong on Home. Manual driving is rarely used and is a debugging tool. Maps do not belong on Home. Fleet operation belongs in the dashboard.

The user approved a Home/Settings structure with separate rover and fleet connection status, contextual next actions, secondary connection details, and advanced configuration/debug tools outside Home.

## Existing problem

Home, Robots, Missions, and More duplicate navigation. Most destinations open ControllerDetailView's large form and scroll to an anchor; unrelated controls remain visible. Rover connection is embedded in actuator layout editing. Home replaces its disconnected layout with a mission/control dashboard as soon as the controller starts, despite the app's primary job being connectivity.

The previous automatic pairing/reconnection work and the running rover service are already implemented. Preserve those behaviors and the existing uncommitted dashboard, simulator, and connection work.

## Navigation

Use two pages rather than persistent tabs, with Settings accessible from an icon in the bottom-left corner of Home: Home and Settings. They share a NavigationStack and the single PhoneController owned by the app root.

Home retains one stable hierarchy in every connection state. Remove its manual joystick, maps, mission controls, autonomy selector, telemetry grid, shortcut grid, and other purely decorative elements. Retain the large rover preview as Home's centerpiece, using it to display the rover's connection state, show its actuators, and identify which rover the phone is currently connected to. Retain Terra's existing identity through its mark, native typography, forest accent, adaptive system surfaces, and light/dark support. Do not add invented metrics, fleet counts, or membership acknowledgements.

Settings is a short list of focused destinations:

- Fleet connection: saved TCP router endpoint, topic prefix, and rover ID.
- Rover setup: automatic connection preference, ARGOS-managed phone tracking status/retry, and actuator configuration.
- Diagnostics: detailed status and recording/export information actually available from the controller.
- Debug tools: manual drive, local simulated rover, simulator-only Bevy connection, sensor inspection, and existing map/waypoint tools if retained.

Existing functionality remains reachable in those destinations, but no destination opens the entire catch-all form. Actuator editing remains its dedicated screen. Move everyday connection actions and duplicate Arm/Disarm controls out of the actuator editor; hardware safety/reset and configuration requirements remain available there where relevant. Debug driving retains explicit arming and emergency-stop controls.

## Home hierarchy

### Header and overall state

Use a compact Terra header and a simple connection summary. Two stacked sections underneath establish the dependency: Rover first, Fleet dashboard second.

Overall copy:

- No authenticated rover: “Connect your rover”.
- Rover available but fleet settings missing: “Set up fleet connection”.
- Rover available and fleet attempting: “Connecting to fleet”.
- Rover and router sessions established: “Connected to fleet”.
- Fault or layout preventing rover readiness: “Rover needs attention”, with the actual next action.

“Connected to fleet” means the transport sessions exist; do not imply that ARGOS has acknowledged membership, that sensors are healthy, that a mission is running, or that motors are armed. Show “Motors disarmed” or the actual armed state as a quiet secondary status. Keep an Emergency Stop action available whenever hardware is connected, without making debugging controls the main Home content.

### Rover section

Show the remembered or connected rover's real display name. If unavailable, use “Your rover”; do not show a placeholder UUID or fabricate a name. Publish the selected/remembered name from the controller using current discovery data, persisted alongside the identifier after confirmation. Forget clears both.

Represent real lifecycle states: searching, pairing offered, pairing/authenticating, authenticated/configuring, connected, paused by user, Bluetooth off, permission required, and recoverable or terminal failure. Introduce a small typed presentation model derived from coordinator and radio facts; do not infer states by matching human-readable error strings in view code.

Each state shows one useful primary action:

- Searching: “Keep your rover nearby”; include concise first-time USR pairing instructions.
- Pairing offered: retain the app-wide Connect / Not now consent popup; a Home indication can explain that a rover was found.
- Connecting/configuring: progress indicator and readable status; no duplicate Connect action.
- Paused/error: Retry when appropriate. Permission denial offers Open Settings. Bluetooth off explains the needed system action.
- Configuration/fault issue: “Review rover setup” opens the relevant setup screen.
- Connected: connection details opens a sheet containing Disconnect and Forget rover. Forget requires a native confirmation because it intentionally changes remembered selection; it does not erase the rover's stored ownership/bond.

Discovery, deadlines, retry backoff, authentication, configuration synchronization, and generation guards remain controller responsibilities. Navigation and opening sheets must not restart scanning, switch hardware mode, or stop healthy connections.

### Fleet dashboard section

Show distinct session status: waiting for rover, setup required, connecting, connected, paused by explicit disconnect, invalid configuration, and retrying/failure.

- Missing settings: “Set up fleet connection” opens the focused Fleet connection editor directly from Home.
- Valid saved settings: connection starts automatically after rover authentication/configuration availability.
- Connecting: progress indication; prevent overlapping attempts.
- Failure: useful concise explanation with Retry or Edit connection as appropriate.
- Connected: show the saved endpoint in details rather than making a TCP address the primary heading. Details exposes explicit Disconnect and Edit connection.

Use the existing saved keys dashboardRouterEndpoint, dashboardTopicPrefix, and dashboardRoverID. Editing is one explicit Save action with validation, not reconnect-on-every-keystroke. Saving applies the settings together, invalidates obsolete attempts, and allows automatic connection. Explicit fleet Retry clears retry suppression without resetting the Bluetooth connection. Explicit fleet Disconnect keeps its existing current-session suppression semantics.

Home should still show that fleet transport is connected in Bluetooth manual mode. Per the user’s follow-up, phone tracking starts after ARGOS connects and stays on in every autonomy mode, including Manual/Teleop. It stops when the fleet session ends or the rover disconnects. Camera/sensor failures expose a tracking-unavailable state with explicit retry while retaining transport sessions. Tracking initialization and healthy tracking are separate from fleet connectivity. Feedback eligibility follows actual layout compatibility/readiness; tracking never arms motors. Preserve the existing Rust controller, router session, accepted autonomy, and recording identity when starting sensors. ARGOS velocity intent takes priority over idle phone input.

## Focused screens and implementation boundaries

Keep root lifecycle ownership in ContentView and PhoneController. Extract Home, connection sheets, Settings, Fleet settings, and Debug screens into focused SwiftUI components. Register new source files in the Xcode project and project-generation script.

Replace ControllerDetailView's destination-anchor navigation with targeted content. Reuse existing DriveControlPanel, ActuatorLayoutView, OccupancyMapView, and simulator APIs rather than duplicating their behavior.

Provide typed connection presentation through a Foundation-only policy/helper that can be tested without SwiftUI or a radio. Preserve raw diagnostic text as secondary details; expose radio power/authorization and in-flight attempts explicitly. Add a dedicated fleet-retry method instead of using hardware Retry for fleet failures.

Moving between Home/details/settings preserves the running transport and sensor controller. Leaving a debug driving surface still zeros joystick/servo intent and disarms. Background stops/disconnects; foreground restores automatic connection as previously implemented. Inactivity revokes motion without canceling system pairing prompts. Pairing consent stays at the root and is not obscured by a splash; remove the timed splash from normal launch so connection state is immediately visible.

## Accessibility and layout

Use native iOS controls and SF Symbols, minimum 44-point actions, text labels alongside status icons, and Dynamic Type layouts that stack when needed. Do not communicate connection state through color alone. Support VoiceOver, reduced motion, light/dark appearance, and long rover names or endpoints. No joystick gesture surface or map should consume Home space. The two connection sections and their next actions should be visible on a typical iPhone without traversing other menus; allow scrolling at accessibility text sizes.

## Verification

- Pure presentation tests: disconnected/searching/pairing/configuring/connected, permission denial, Bluetooth off, invalid layout, missing/invalid fleet settings, retries, explicit suppression, and both-session summary. An authenticated but configuration-unready rover must not be presented as fully ready.
- Controller integration checks: fleet Retry does not disconnect BLE; Save validates and applies all settings together; Forget clears identifier/name; navigation does not stop transports or create a duplicate attempt.
- Build both unsigned iPhone and simulator targets; retain existing connection-policy and joystick-safety checks.
- Inspect rendered Home and focused settings/debug screens for disconnected, connecting, connected, and attention states in the simulator with deterministic preview data. Check large text and dark appearance. Simulator previews must not claim physical Bluetooth acceptance.
- Physical pairing, connection recovery, and fleet membership remain hardware acceptance tasks; this UI refactor does not redeploy the rover service.

## Scope

This overhaul changes information architecture, connection presentation, and settings interactions. It does not introduce a new fleet protocol, router discovery, ownership reset, background execution, automatic arming, or dashboard mission-management features. Implementation begins after written-spec review and a reviewed implementation plan.
