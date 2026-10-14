# Building TerraPhone on Linux

!!! tip "TL;DR"
    xtool packs the existing Swift sources into an iOS app.
    You still need Swift 6.4, an Apple ID, and an Xcode `.xip`.
    The Xcode project is unchanged. Open `TerraPhone.xcodeproj` on a Mac.

[xtool](https://xtool.sh/documentation/xtool/installation-linux/) builds a Swift package for `arm64-apple-ios` on Linux. TerraPhone's package is `mobile/ios/Package.swift`, next to the Xcode project. `xtool.yml` sets the bundle id to `org.terra.robotics.phone`, the same id as the Xcode target.

`xtool-Info.plist` only adds usage strings and the display name. It does not contain `$(EXECUTABLE_NAME)` or `$(PRODUCT_BUNDLE_IDENTIFIER)`. xtool generates those itself. The Xcode `Info.plist` is untouched.

## Install

Ubuntu 24.04 packages Swift's toolchain expects, plus `usbmuxd` for a plugged-in iPhone:

```sh
sudo apt-get update
sudo apt-get install -y \
  binutils git gnupg2 libc6-dev libcurl4-openssl-dev libedit2 \
  libgcc-13-dev libncurses-dev libpython3-dev libsqlite3-0 \
  libstdc++-13-dev libxml2-dev libz3-dev pkg-config \
  usbmuxd libimobiledevice-utils zip unzip zlib1g-dev clang
```

Swift 6.4 is what xtool's Linux toolset requires. The published Ubuntu 24.04 tarball is 6.4.1 (the `swift-6.4-RELEASE` URL returns 404):

```sh
curl -fL \
  "https://download.swift.org/swift-6.4.1-release/ubuntu2404/swift-6.4.1-RELEASE/swift-6.4.1-RELEASE-ubuntu24.04.tar.gz" \
  -o /tmp/swift.tar.gz
sudo mkdir -p /opt/swift
sudo tar -xzf /tmp/swift.tar.gz -C /opt/swift --strip-components=1
export PATH="/opt/swift/usr/bin:$PATH"
swift --version
```

That URL is the x86_64 toolchain. On aarch64 Linux, use `ubuntu2404-aarch64` and `swift-6.4.1-RELEASE-ubuntu24.04-aarch64.tar.gz` in the same release directory.

xtool 1.22.2, as an AppImage. Extract it when FUSE is not available:

```sh
curl -fL \
  "https://github.com/xtool-org/xtool/releases/download/1.22.2/xtool-$(uname -m).AppImage" \
  -o /tmp/xtool.AppImage
chmod +x /tmp/xtool.AppImage
( cd /tmp && ./xtool.AppImage --appimage-extract )
sudo mv /tmp/squashfs-root /opt/xtool
sudo ln -sf /opt/xtool/AppRun /usr/local/bin/xtool
xtool --help
```

A newer AppImage from [the latest release](https://github.com/xtool-org/xtool/releases/latest) is fine if it still reads `xtool.yml` version 1.

## Xcode.xip

xtool's iOS SDK is extracted from Xcode, not downloaded by this repo.

1. Sign in at [developer.apple.com/download/all/?q=Xcode](https://developer.apple.com/download/all/?q=Xcode) and download **Xcode 27** as `Xcode.xip`. The site asks for an Apple ID. `curl` without that login does not get the archive.
2. Install the SDK. This is large and does not need the paid developer program:

```sh
xtool sdk install /absolute/path/to/Xcode_27.xip
xtool sdk status
swift sdk list
```

`swift sdk list` should include `darwin`. `xtool sdk install --slim` uses less disk and cannot be updated in place.

## Build

From the Terra checkout, with Rust on `PATH` (`rustup`, stable):

```sh
export XCODE_XIP=/absolute/path/to/Xcode_27.xip   # optional if `xtool sdk status` already says installed
./scripts/build-ios-linux.sh
```

The script:

1. Checks `mobile/ios/xtool.yml` still matches the Xcode bundle id.
2. Stops with exit code 2 if the Darwin SDK is missing. It does not sit on the `xtool dev build` prompt.
3. Generates `TerraCore.swift` with the host `libterra_mobile` and UniFFI.
4. Builds `libterra_mobile` for `aarch64-apple-ios`, linking with the SDK's `ld64.lld`. Without that linker, the host `ld` stops on `unrecognised emulation mode: llvm`.
5. Runs `xtool dev build` in `mobile/ios`.

The unsigned app is `mobile/ios/xtool/TerraPhone.app`. That directory is gitignored. So is the generated `mobile/ios/TerraPhone/TerraCore.swift` copy the package compiles. Xcode keeps compiling `mobile/ios/Generated/TerraCore.swift` and `TerraCore.xcframework` from `./scripts/build-ios.sh` on a Mac.

`./scripts/check-xtool-ios.sh` is the same config check, without Swift or the SDK. CI runs it on every pull request.

## Deploy to a device

Signing and install need an Apple ID. A paid developer account can use an App Store Connect API key. Any Apple ID can use the password login. Credentials go to Apple through xtool's login, not to this repo.

```sh
xtool auth
```

`xtool setup` is auth plus the SDK prompt. If the SDK is already installed, `xtool auth` is enough.

Plug the iPhone in over USB, unlock it, and tap Trust. `usbmuxd` has to be running (`sudo apt-get install usbmuxd`).

```sh
ideviceinfo
xtool devices
cd mobile/ios
xtool dev --udid <UDID>
```

`xtool dev` with no udid asks which connected device to install onto. `xtool dev build` only writes the `.app` and does not need a login. `xtool dev build --sign` and `xtool dev` do.

Developer Mode has to be on, and the phone has to trust the signing certificate the first time the app opens. xtool's installer registers the device with the Apple ID you logged in with.
