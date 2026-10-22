# Direct device logging setup — 2026-10-21

## Phone

The rebuilt app was installed on the existing iPhone without removing its data
container. Its source is `phone-C2B2238A-1282-42CD-9C74-F436D1782B4E`, scoped for
writing Rover 0 and source-health events. The upload import was placed in the
private app container; the app imported the endpoint and token into its saved
settings/Keychain. No raw credential is stored in source control.

Authenticated phone-origin events reached GCP. The old run's backlog is being
uploaded with its original identities and timestamps. A journal snapshot at
22:19:43 UTC showed 5,248 acknowledged events, 8,803 pending events, and zero
uploader failures or journal drops/failures. This verifies the live phone path
and durable backlog; it is not a completed replay of every queued event.

The independent phone journal now starts when the app starts, even without a
control runtime. A bounded early queue retains startup events until the journal
opens. The sole uploader continues to service the shared local journal when
control runtimes change. Added events distinguish operator/router emergency
requests, Bluetooth queueing/failure, received hardware safety state, and
connection lifecycle reasons. A queued command is not a delivery acknowledgement.
Pi command-acceptance records supply that receiving-side evidence.

## Pi access, USB and offline uploading

Physical Pi 4 USB networking and SSH are verified at `pi@raspberrypi.local`.
The MasterPi Bookworm image was flashed after a verified full backup of the old
rover card. Its USB profile needed a NetworkManager `managed=1` override because
Debian's gadget udev rule otherwise leaves the interface unmanaged. The fix is
included in the prepared image and repository packaging.

`terra-log-forwarder.service` now runs independently from rover control. It
persists allowed rover and Bluetooth journal events in
`/var/lib/terra-observability-pi/events.db`, including original structured event
identities. The Pi clock was corrected from the Mac and saved with fake-hwclock.
The Pi still has no internet connection.

This Mac runs `com.terra.usb-log-relay` as a login LaunchAgent. It connects over
USB using a dedicated key restricted to queue reads and acknowledgments, uploads
the original events to GCP over HTTPS, and acknowledges the Pi queue only after
valid cloud acknowledgment. It retries after USB disconnection or cloud failure.
The private installation and connection-status file are under
`~/Library/Application Support/Terra/usb-log-relay/`. The Mac must be awake, logged
in, online and connected to USB for this route to upload; the Pi continues
recording independently when it is absent. Existing bounded retention still applies.
No internet routing or TLS relaxation was added to the Pi.

The post-reboot relay automatically reconnected and genuine Pi service events
reached the cloud collector. Pausing the Mac uploader retained 56 new events locally; resuming delivered all
56, received cloud acknowledgment, and removed them from the pending batch. These checks issue no
motion commands and are not a physical driving test.

## Rover restoration

The saved actuator layout, owner information and Bluetooth state were restored
from the pre-flash card backup. The current instrumented Python rover source is
installed under `/opt/terra-rover`, with pinned offline dependencies and the
backed-up Fusion HAT Python package. This avoids substituting the older compiled
binary, which lacks the new command-receipt logs.

The backup's matching Pi 4 kernel, modules and Fusion HAT overlay were staged
under separate boot filenames, preserving the working USB configuration and new
root partition identity. Previous boot settings are backed up on the Pi under
`/var/backups/terra-usb-restore/`.

After reboot the relay captured a genuine rover startup failure: the Fusion HAT
LED sysfs path was absent. Hardware-driver/power verification is still in progress;
this record does not claim that the rover control service is ready for driving.
No arming, motion command, safety timeout or stop policy was changed.

## Credentials and verification

All pre-existing collector permission entries were preserved exactly; only the
two Rover 0 writer credentials were added. Only the logging collector was
restarted for that change; the Zenoh router retained PID 1752.

The new USB queue tests cover bounded requests, fixed database access, original
identity preservation, selected acknowledgment, and retaining queued records on
cloud failure followed by delivery on reconnection. The installer and private
configuration contain no credentials in version control.


## Wi-Fi verification and replacement image preparation

The existing Pi joined `Super_AC` at `192.168.40.50`. A deliberately labeled
`logging.setup_check` event originating on the Pi was acknowledged by GCP over
its direct HTTPS uploader. The main observer database contained 17,804 acknowledged
events and no pending events at that check. The independent observer remained active
while the rover service was stopped for a Fusion HAT I2C probe failure.

A private backup of the Pi SQLite journals and kernel/rover/Bluetooth diagnostic
history was saved before preparing a replacement image. It contains no copied
Wi-Fi passwords or SSH credentials.

The intended replacement base is the published `terra-image-v0.1.0-rc3` release
from `Super-Yojan/terra-image-gen`. Both compressed and uncompressed published
hashes matched. Its Linux image checks passed, including bundle execution and
Fusion HAT modules built for both `6.12.109+rpt-rpi-v8` and `6.12.109+rpt-rpi-2712`.
The user approved hotspot mode at every boot plus independent Pi 4 USB SSH.
Connectivity changes are being prepared in that repository; this record does
not yet assert a successful physical boot of the customized image.

The observer unit now starts after journald without waiting for network-online.
It persists early events and retries uploads whether hotspot mode has internet
or not. This avoids delaying independent logging while waiting for an uplink.

The replacement image now includes the instrumented rover, independent observer,
existing rover layout/phone bond, and the privately supplied hotspot configuration.
Image connectivity source is committed as `terra-image-gen` commit `3b0764d`.
All 13 observability/queue tests passed on the Pi; Linux network and image-check
fixtures passed. The logger also captures kernel, hotspot, network and systemd
unit-failure diagnostics with secret redaction. The current Pi observer was updated
and is active. The customized image has not yet been flashed or physically booted;
SSH authentication provisioning and post-flash USB/hotspot/HAT verification remain.

## Fusion image physical acceptance

The Pi booted the intended Fusion image as `terra-bb75b9`, kernel
`6.12.109+rpt-rpi-v8`. USB DHCP assigned the Mac `192.168.77.2`; SSH to
`pi@192.168.77.1` works with the separately approved Mac administration key.
The WPA2 hotspot is `Terra-bb75b9`, with SSH at `192.168.78.1` after joining it.
The hotspot was discovered and association tested; the USB path was used for
subsequent setup and verification.

Offline repair restored root ownership on 45 system paths inherited incorrectly
from the image overlay, including sudo configuration. A metadata recovery image
and 6,936 event records were saved first. The ext4 journal recovered and the final
filesystem check passed. The private pre-repair image must not be reused without
correcting ownership. This was a targeted SD repair, not a reflash.

The HAT initially failed I2C communication at `0x17`. After the operator connected
the HAT, its controller answered. A live driver retry produced kernel warnings;
those logs were saved, the rover was stopped, and the Pi was rebooted. On the next
boot the driver probed successfully with the hardware interface present. Rover,
independent observer and hotspot all ran with zero service restarts. Startup
observations explicitly reported `previous_armed=false`; no arm or drive command
was sent. Driver version is 1.1.0 and MCU firmware is 1.0.2. This confirms startup,
not a physical driving test.

The existing restricted Mac relay key was installed with queue-only access and
pinned to the verified new Pi host identity. Its target is now `pi@192.168.77.1`.
A labeled synthetic `logging.setup_check` event remained pending while the Mac
relay was stopped and received GCP acknowledgment after restart. All 6,936 saved
pre-repair events were also cloud-acknowledged through a paced archive upload,
with original identities and timestamps preserved. The original archive remains
unchanged. The Pi's initially unset wall clock is inaccurate for those first-boot
events; compare monotonic times within each recorded boot.

Local event capture continues without USB or internet. With the Mac awake,
logged in, online and attached by USB, its LaunchAgent forwards the Pi backlog.
Hotspot mode alone does not give the Pi an internet uplink. Direct HTTPS upload
is also configured for when the Pi has an uplink. Existing retention and byte
limits still apply.
