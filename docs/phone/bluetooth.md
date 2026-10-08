# Bluetooth pairing

On a physical iPhone, TerraPhone talks to the Pi peripheral. The Simulator build does not show Discover or Connect. The protocol, GATT layout, and service flags are specified in [Bluetooth peripheral](../hardware/BLUETOOTH.md). Installation of the executable is in [Install on a Pi](../hardware/INSTALL.md).

Radio sessions, actuator timing, and physical compatibility have not been verified. The notes below describe the software path that is in the tree. They are not a record of a completed pairing on hardware. See [evidence](../hardware/evidence/README.md).

## Customer enrollment

Ship the Pi with the Fusion HAT kernel driver, Bluetooth, and `terra-rover` installed. No SSH session and no known phone address are required.

1. Turn the rover on and open TerraPhone.
2. Release the Fusion HAT USR button if it was held during boot, then hold it for three seconds. The LED blinks slowly and a 60-second pairing window opens. The advertised name is `terra-XXXXXX`.
3. Tap **Find rovers**, choose that name, and connect. Accept the iOS pairing prompt if it appears. The LED blinks quickly while the first phone enrolls.
4. Three LED flashes mean the private owner record was saved. The rover switches to its normal service. Tap **Find rovers** and reconnect. The LED stays on.
5. Configure the actuator layout and arm only when that is safe.

The first-owner window is Bluetooth Just Works: proximity plus the physical button, without numeric comparison. Keep enrollment supervised. A rover that already has an owner ignores the button. A timeout needs another hold. Missing or unreadable HAT controls do not open pairing. Pairing does not arm motors and does not require the gate file.

Automatic connection on a physical iPhone is on by default. The app waits until Bluetooth is ready, scans only for Terra's service, and reconnects to the last rover that synchronized. With no saved rover it waits two seconds and selects a candidate only when exactly one is present. Several candidates require a manual choice in Robots. It will not switch to a different rover when the saved one is absent. Disable **Connect Automatically** in Robots to stop later attempts. Connection still does not arm motors or start a mission.

## Developer provisioning

With the customer service stopped, a developer can run `--setup-owner` with `--expected-peer` set to the phone address BlueZ sees, and an explicitly open `--gate-file`. Setup uses numeric comparison: the six-digit code is confirmed on the phone and by typing `yes` on the Pi. That path is in [Bluetooth peripheral](../hardware/BLUETOOTH.md). Do not run two peripheral processes at once.

## After the link is up

The primary service UUID ends in `-4c2b-4f91-9e3a-1d8c6b2a0f10`. The prefixes are `7e5a0010` (service), `7e5a0011` (drive), `7e5a0012` (status), `7e5a0013` (priority and JSON control), and `7e5a0014` (configuration replies). Read status first so the link encrypts and the owner is admitted, then subscribe to status and replies.

The phone's configuration screen is **Discover, configure and arm rover** (`ActuatorLayoutView`). It is a list: up to sixteen actuators, ids 0–255, kinds and ports taken from the rover's capabilities. **Validate and stage draft** and **Commit acknowledged stage** talk to the Pi. Stage against the active revision. Commit names that stage's request id and revision. Commit does not arm.

Open the hardware cutoff and disarm before changing the layout. Manual drive then needs an explicit Arm and the rover's acknowledgement. Fresh drive is sent at 20 Hz. The rover watchdog expires at 200 ms. Disconnect, loss of the status subscription, and Emergency Stop disarm. A fresh connection never restores arming.

Bench-mode rovers advertise `gate_mode=bench`. Drive shows **Enable Bench Control** separately from Arm, and says there is no physical power cutoff. Stop, Disarm, and disconnect clear that session-only permission. Rovers that do not advertise bench mode still require the physical gate.

Configuration today is this list. Tapping the 3D model to edit a part is [planned](tap-to-configure.md).
