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
