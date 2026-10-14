#!/bin/sh
# Confirm the xtool package still matches the Xcode app, and that the
# Xcode project was not retargeted at the Linux package.
set -eu
repo_root=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
ios="$repo_root/mobile/ios"
fail() { echo "check-xtool-ios: $*" >&2; exit 1; }

[ -f "$ios/Package.swift" ] || fail "missing Package.swift"
[ -f "$ios/xtool.yml" ] || fail "missing xtool.yml"
[ -f "$ios/xtool-Info.plist" ] || fail "missing xtool-Info.plist"
[ -f "$ios/LinuxSupport/module.modulemap" ] || fail "missing LinuxSupport module map"
[ -f "$ios/TerraPhone/Info.plist" ] || fail "missing Xcode Info.plist"
[ -f "$ios/TerraPhone.xcodeproj/project.pbxproj" ] || fail "missing Xcode project"

grep -q 'name: "TerraPhone"' "$ios/Package.swift" || fail "Package.swift product name drifted"
grep -q '.iOS(.v17)' "$ios/Package.swift" || fail "Package.swift must stay on iOS 17"
grep -q 'swiftLanguageMode(.v5)' "$ios/Package.swift" || fail "Package.swift must keep Swift 5 mode"
grep -q 'path: "TerraPhone"' "$ios/Package.swift" || fail "Package.swift must compile the existing TerraPhone sources"

grep -q '^version: 1$' "$ios/xtool.yml" || fail "xtool.yml version must be 1"
grep -q '^bundleID: org.terra.robotics.phone$' "$ios/xtool.yml" || fail "xtool.yml bundle ID drifted"
grep -q '^product: TerraPhone$' "$ios/xtool.yml" || fail "xtool.yml product drifted"
grep -q '^infoPath: xtool-Info.plist$' "$ios/xtool.yml" || fail "xtool.yml must use the overlay plist"
grep -q 'iconPath: TerraPhone/Assets.xcassets/AppIcon.appiconset/icon.png' "$ios/xtool.yml" || fail "xtool.yml icon path drifted"
grep -q 'TerraPhone/Models' "$ios/xtool.yml" || fail "xtool.yml must copy Models into the app bundle"
[ -f "$ios/TerraPhone/Assets.xcassets/AppIcon.appiconset/icon.png" ] || fail "app icon png missing"
[ -d "$ios/TerraPhone/Models" ] || fail "Models directory missing"

# The overlay is merged on top of xtool's generated Info.plist. Xcode
# substitutions here would overwrite the real bundle id and executable name.
if grep -q '\$(' "$ios/xtool-Info.plist"; then
    fail "xtool-Info.plist must not contain Xcode \$(...) substitutions"
fi
grep -q 'NSBluetoothAlwaysUsageDescription' "$ios/xtool-Info.plist" || fail "overlay plist missing Bluetooth usage string"
grep -q 'Terra Brain' "$ios/xtool-Info.plist" || fail "overlay plist display name drifted"

# The Xcode project keeps its own Info.plist and bundle id.
grep -q 'PRODUCT_BUNDLE_IDENTIFIER = org.terra.robotics.phone;' "$ios/TerraPhone.xcodeproj/project.pbxproj" || fail "Xcode bundle id changed"
grep -q 'INFOPLIST_FILE = TerraPhone/Info.plist;' "$ios/TerraPhone.xcodeproj/project.pbxproj" || fail "Xcode Info.plist path changed"
grep -q 'IPHONEOS_DEPLOYMENT_TARGET = 17.0;' "$ios/TerraPhone.xcodeproj/project.pbxproj" || fail "Xcode deployment target changed"
grep -q '$(PRODUCT_BUNDLE_IDENTIFIER)' "$ios/TerraPhone/Info.plist" || fail "Xcode Info.plist lost its bundle id substitution"
grep -q '$(EXECUTABLE_NAME)' "$ios/TerraPhone/Info.plist" || fail "Xcode Info.plist lost its executable substitution"

echo "xtool iOS config matches the Xcode project"
