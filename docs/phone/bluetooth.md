# Bluetooth pairing

!!! tip "TL;DR"
    Physical iPhone only.
    Hold the Fusion HAT USR button for 3 seconds.
    Name looks like `terra-XXXXXX`.
    Pairing does not arm the motors.

![Four pairing steps](../assets/pairing.svg)

*LED: slow blink while waiting, fast while enrolling, three flashes when saved, then solid.*

![iPhone wireframe of Discover, configure and arm](../assets/phone-iphone.svg){ width="240" }

*Placeholder of the device section. Radio sessions are still unverified. See [evidence](../hardware/evidence/README.md).*

## Steps

1. Rover on. TerraPhone open.
2. Hold USR for three seconds. Window lasts 60 seconds.
3. **Find rovers**. Pick `terra-XXXXXX`. Accept the iOS prompt.
4. Reconnect after the three flashes.
5. Stage a layout, commit, then Arm when it is safe.

Just Works for the first owner. Keep it supervised. A rover that already has an owner ignores the button.

![GATT path after the bond](../assets/pi-stack.svg)

*Read status before writes. Fresh drive is 20 Hz. Rover watchdog is 200 ms.*

Automatic reconnect prefers the last rover that synchronized. Several candidates need a manual pick. Turn it off under **Connect Automatically**.

Developer numeric pairing is `--setup-owner` with the customer service stopped. Full bytes: [Bluetooth peripheral](../hardware/BLUETOOTH.md).

Bench mode shows **Enable Bench Control** separately from Arm. It says there is no physical cutoff.

## Configuration pages

The app requires the rover capability `actuator_pagination_v1`. Connection reads a small actuator index, then one compact drive profile at a time. Opening an actuator loads only that actuator's editable configuration. The full `layout.json` remains on the rover.

Editing begins a rover draft with `begin_layout_edit`. Each `stage_actuator` or `remove_actuator` acknowledgment advances its version. `validate_layout_edit` validates the complete draft on the rover; `commit_layout_edit` applies that exact version atomically and replies with the revision. `discard_layout_edit` abandons the draft. Presets ask for missing physical ports before uploading each actuator.

Configuration messages have a 4,080-byte bound, a two-second fragment assembly deadline and a five-second acknowledgment deadline. **Retry interrupted operation** replays the exact request ID to recover the rover's cached acknowledgment. **Retry configuration** reloads the index and drive profiles after synchronization failure. Motion still uses its separate freshness limits and watchdog. Reconnect or revision changes invalidate draft authority and drive profiles; unsent editor changes remain visible.
