# Phone dashboard over Tailscale implementation plan

**Goal:** Connect one foreground TerraPhone to ARGOS through a Mac-hosted Zenoh router over Tailscale, preserving explicit hardware arming.

**Architecture:** Phone and ARGOS open outbound TCP client sessions to a router. The phone's existing MobileController consumes dashboard requests and publishes lightweight pose and status. Bluetooth remains the actuator transport. No MQTT or cloud deployment.

**Scope approved in chat:** Implement the Zenoh/Tailscale topology recommended on October 8, 2026. Execute in this session and preserve existing unrelated changes.

## Constraints and review focus
- Explicit connect/disconnect, no stored automatic connection and no command replay.
- Dashboard connection only attaches to an active phone or simulated sensor controller, never manual Bluetooth or a remote Bevy controller.
- Connecting never arms Bluetooth. Backgrounding/stopping resets the session and authority; disconnect clears remote intent and disarms hardware.
- Pose publications require fresh tracked VIO. Stalled control ticks must stop refreshing telemetry.
- First slice supports one phone per topic prefix; simulator and phone use separate prefixes. No fleet aggregation claims.
- Router binds only localhost and an explicitly supplied Tailscale IPv4 address; no public listening endpoint.
- Confirm transport loss/reconnect behavior with router integration tests; physical iPhone/Tailscale evidence remains separate from local automated tests.

## Tasks
- [x] Add router-client ControlPlane connection, freshness-bounded publication, disconnect detection, and loopback router integration tests.
- [x] Expose connect/disconnect/status in UniFFI; add phone connection controls and lifecycle guards; verify reset clears remote goals and session.
- [x] Add lightweight pose decoding/subscription to ARGOS, preserving simulator depth support; test validation and command acknowledgement.
- [x] Add Mac router launcher and Tailscale setup guide, generate bindings, build physical-device/simulator targets, and run targeted tests.

## Verification, October 8, 2026

- Terra workspace: 68 passed, 0 failed; 2 pre-existing live remote tests ignored.
- ARGOS workspace: 15 passed, 0 failed.
- Generated Swift pair smoke: real phone controller and ARGOS clients through a loopback router; pose, goal acknowledgement, cancellation and explicit reconnect passed.
- TerraPhone unsigned device and Simulator builds: succeeded. ARGOS Mac build: succeeded. Both Apple XCFrameworks rebuilt.
- Launcher: localhost-only router accepted an outbound Zenoh client; temporary test router stopped after verification.
- Independent review found no confirmed critical/important defect. Addressed test gaps for queued intent, accepted-goal loss/reconnect, and sensor preservation.
- Physical verification remains pending: Mac Tailscale is stopped and the registered iPhone is unavailable to Xcode. No physical rover motion or Tailscale latency was claimed.
