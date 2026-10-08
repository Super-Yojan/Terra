# Tap to configure

**Planned.** [Terra #27](https://github.com/Super-Yojan/Terra/issues/27) asks for a configuration mode that shows a 3D model of the connected vehicle. Tapping a part, such as a drive motor, would highlight it and open a small popup. The first setting in that popup is invert direction. Later settings can include limits and mapping. The change would commit through the same layout path the list uses today.

That mode is not in the app. `RoverModelView` loads `rover.usdz`, orbits, zooms, and resets the camera. It has no tap handler that selects an actuator. Configuration is the list in `ActuatorLayoutView`, linked as **Discover, configure and arm rover** on a physical iPhone. See [Bluetooth pairing](bluetooth.md).

## Vehicle description format

Also planned, and still an open design question on #27. The app needs a format that says which node in the 3D model is which actuator, and where that description is loaded from: bundled in the app, stored on the vehicle, or fetched. Nothing in this repository is that format. The USDZ file is a viewing mesh. Actuator identity today is the layout JSON: numeric ids, kinds, ports, and coefficients, validated by `terra-actuators` and again on the Pi.

#27 asks for the format to line up with:

- [Terra #22](https://github.com/Super-Yojan/Terra/issues/22), easy robot import and flexible actuators.
- [Terra #23](https://github.com/Super-Yojan/Terra/issues/23), the Zorvane split, so the same description can describe the body in simulation and on the Pi.

Until that format exists, a new motor is configured by editing the list, staging the layout, and committing it while the rover is disarmed and the gate is confirmed open.
