# Real rover controller tuning

Controller defaults are for the simulator. Tune with the phone mount calibrated
and the rover motor adapter initially disabled.

1. Measure wheel radius, track width, motor deadband, and motor response.
2. Confirm motor direction and sign with the rover restrained or wheels lifted.
3. Begin with low maximum speed, turn rate, and motor effort.
4. Test straight motion at low speed; tune proportional gain for response.
5. Add integral gain to reduce steady-state error; watch for oscillation.
6. Repeat for yaw rate and turning in place.
7. Tune feedforward against measured motor response.
8. Increase limits gradually in a clear test area.
9. Verify stop behavior, stale sensors, tracking loss, and command timeout.
10. Record final settings, test conditions, and reviewer before enabling hardware.

## Rover profiles

The phone picks a rover profile in **Settings → Rover setup → Rover**. The
profile (`crates/terra-runtime/src/rover_profile.rs`) sets the planner footprint
(length, width, clearance), turning policy, speed limits and the velocity loop
for that chassis. Feedforward is `1 / top_speed`. Feedback gains scale with it,
so a slower rover keeps the loop dynamics tuned on Terra.

`wheel_deadband` is the wheel effort below which the motors stall. The velocity
loop lifts any moving wheel command past it. Without it, a small target speed
(ARGOS teleop asks for 0.2 m/s) gives efforts that sit inside the deadband, and
the rover stalls even though the debug joystick, which sends raw effort, drives it.

To measure a chassis on its wheels, use the debug joystick:

| Field | How to measure |
| --- | --- |
| `wheel_deadband` | Smallest straight effort that starts the rover rolling, minus a little |
| `spin_effort` | Smallest turn effort that spins it in place on the test floor, plus a little |
| `top_speed` | Speed at full effort over a measured distance |

terra-mini's deadband (0.3), spin effort (0.6) and top speed (0.8 m/s) are still
estimates. Its body is 0.178 m square with a 0.152 m track.
