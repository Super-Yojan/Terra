# Local USB access on Pi 4

This Bookworm configuration provides local SSH access over the Pi 4 USB-C port.
It uses the existing `dwc2`/`g_ether` kernel modules and NetworkManager; no package
download or internet connection is required. Raspberry Pi documents the hardware
and overlay in its [OTG guide](https://pip-assets.raspberrypi.com/categories/685-app-notes-guides-whitepapers/documents/RP-009276-WP-1-Using%20OTG%20mode%20on%20Raspberry%20Pi%20SBCs).

Preserve the current SD card before changing a boot image. A downloaded vendor
image is not a backup of the rover's current pairing, layout, logs or software.
The prepared 2026-10-21 image is a separate copy of that vendor baseline and
needs validation on a spare card. It has not been boot-tested on a physical Pi.

For an existing Bookworm installation with terminal access, install
`packaging/rover/terra-usb-modules.conf` as
`/etc/modules-load.d/terra-usb.conf` (root:root, 0644), and
`packaging/rover/terra-usb-local.nmconnection` as
`/etc/NetworkManager/system-connections/terra-usb-local.nmconnection`
(root:root, 0600). Ensure SSH and Avahi are already enabled and preserve their
existing credentials/settings.

Back up `/boot/firmware/config.txt`, then append once:

```ini
[pi4]
dtoverlay=dwc2,dr_mode=peripheral
[all]
```

Do not replace `cmdline.txt`, change the root partition identifier, or alter
existing Wi-Fi profiles. Reboot while stationary, use a data-capable cable to
the USB-C port, and maintain a suitable Pi power supply. USB-A remains available
for host peripherals. After the Mac receives a self-assigned address, connect
with `ssh <existing-user>@<existing-hostname>.local` using the existing password
or SSH key. The prepared vendor image uses `pi@raspberrypi.local`.

The profile uses IPv4/IPv6 link-local addresses and sets no default route.
It provides local access only; it does not automatically share the Mac's internet.
Pi logs can remain queued offline. Configure an internet path separately if
continuous GCP uploads are wanted.

To undo this configuration, remove the two installed files, restore the saved
boot configuration, and reboot. Keep the source journals and credentials intact.
