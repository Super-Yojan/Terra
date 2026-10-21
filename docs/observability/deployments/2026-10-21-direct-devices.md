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

## Pi access and USB image

The rover is a Pi 4 without internet. No SSH/local device connection has been
established, so direct Pi logging is not yet installed or verified. Scoped
`pi-rover0` credentials and an offline observer bundle are prepared privately
under the ignored `observability/device-setup-20261021/pi-offline/` directory.
That bundle does not contain a rebuilt compiled rover daemon.

A separate copy of the existing `MasterPi20250328.img` was prepared at
`observability/releases/MasterPi20250328-terra-usb.img`, with adjacent instructions
and checksum. This is a copy of the downloaded MasterPi baseline, not a backup
of the current rover SD card. Neither the original download nor a physical SD
card was changed. Use a spare card for the first boot validation.

The Bookworm image enables Pi 4 `dwc2` peripheral mode, loads `g_ether`, and adds
a USB-only NetworkManager profile with link-local addresses and no default route.
Existing SSH, Avahi, login credentials, and other network profiles are retained.
After boot and USB enumeration, the intended connection is
`ssh pi@raspberrypi.local`. USB networking alone does not supply internet.
The original image's ext4 journal was recovered in the copy before modification;
final ext4 and FAT checks passed. Physical boot/USB enumeration is unverified.

## Verification and remaining work

The iOS build, Swift observation checks, six Pi observation tests and 91 Pi
regression tests passed. Review found two issues (early events without a sink,
and stale router failure attribution); both were corrected and the focused
review passed. No safety timeout, stop policy or arming policy changed.

All pre-existing collector permission entries were preserved exactly; only the
two Rover 0 writer credentials were added. Only the logging collector was
restarted for that change; the Zenoh router retained PID 1752.

Remaining: boot-test USB on the physical Pi or access its current SD installation,
install the independent forwarder and instrumented source, and confirm genuine
Pi safety/connection events reach GCP after an offline interval. Do not treat
dashboard observations or the prepared image as evidence that this is complete.
