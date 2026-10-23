# Draft: Rover control and safety requirements

Status: proposed requirements for validation, recorded after the October 20,
2026 driving test. Existing timeout settings are not validated acceptance limits.

## Control quality

- Measure end-to-end input-to-actuation delay, command spacing, jitter, and
  missed commands.
- Sustained input and forward/reverse transitions must work without unexplained
  exits from driving mode.
- Define acceptable latency and interruption rates separately for local Wi-Fi
  and cellular operation.
- Display connection quality using command and status freshness, not merely
  "connected."

## Safety and timeouts

| Condition | Required behavior |
| --- | --- |
| Drive commands expire | Apply safe output and record the command gap. Require explicit resume after a safety pause. |
| Hardware status becomes stale | Stop sending motion commands; show that rover state is unconfirmed. |
| ARGOS connection is lost | Revoke remote driving authority and prevent old commands from restarting motion. |
| Bluetooth connection is lost | Rover independently applies safe output. Reconnection must not automatically resume driving. |
| Required sensor data is lost | Hold autonomous motion and identify the missing or unhealthy sensor. |
| Camera tracking is lost in L0 Manual | Manual driving remains available if control and hardware safety requirements are satisfied. |
| ARGOS loses window focus | Release keyboard driving and retain a visible explanation. |

Choose timeout limits from a stopping-distance budget, accounting for speed,
detection delay, actuator response, braking performance, surface, and slope.
Verify the resulting limits against measured network jitter.

Current implementation settings include:

- Rover drive-command watchdog: 200 ms.
- ARGOS hardware-status freshness cutoff: 500 ms.
- Default motion-sensor freshness: 100 ms.
- Default camera-pose freshness: 350 ms.

These values require validation; increasing them alone is not a fix.

## Recovery

- Reconnection must never replay a previous motion command.
- Safety-pause recovery requires fresh neutral input and explicit resume.
- Distinguish resume motion, re-enter driving control, and re-arm.
- Preserve the original stop reason after the connection recovers.

## Diagnostics

Every interruption should record its first trigger and timestamp, the component
that stopped motion, command and hardware-status ages, relevant connection and
sensor health, arm/pause/control-authority state, and recovery action and outcome.

ARGOS should retain this explanation instead of returning to a generic
"Drive this rover" prompt.

## Acceptance tests

Test sustained driving, repeated direction changes, delayed or dropped packets,
cellular/Wi-Fi transitions, Bluetooth loss, window-focus changes, and sensor loss.

For each scenario, verify:

1. Measured response delay and stopping distance satisfy the agreed limits.
2. No stale command causes movement after recovery.
3. The displayed reason matches the recorded trigger.
4. Normal operation does not produce unnecessary stops.

Open decisions: maximum test speed, permitted stopping distance, operating
surfaces, acceptable response latency, and required uninterrupted test duration.

## October 20 test observations and remaining work

- L0 manual operation continued without camera tracking.
- Phone logs captured rover watchdog pauses while armed, without a reported
  hardware fault; Bluetooth status continued arriving. The precise source of
  command-delivery gaps remains unproven.
- Another stop returned ARGOS to "Drive this rover" while live rover status
  remained armed and not paused. Focus loss and transient stale status are code
  paths that can exit driving; neither was established as this event's cause.
- A later phone disarm log recorded the dashboard/router connection disconnected
  while Bluetooth remained active, followed by re-arming. The initiating cause
  of that disconnect/disarm was not established.
- A rover service update was built and passed 91 rover tests. It bounds long
  fault/stop-reason status strings, logs full fault transitions, and supervises
  output-worker termination. It was not installed on either rover during this
  session because the target Pi became unreachable before transfer.
- Monitoring was stopped at the user's request. Private raw phone logs and
  deployment artifacts remain outside the repository.
