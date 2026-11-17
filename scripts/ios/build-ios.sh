#!/bin/sh
# Regenerate TerraPhone.xcodeproj from the folders under apps/ios/TerraPhone.
# terra-av has no Swift bindings yet; TerraAV.swift is a stub that returns
# stop until they exist.
set -eu
repo_root=$(CDPATH= cd -- "$(dirname -- "$0")/../.." && pwd)
cd "$repo_root"
cargo build -p terra-av
ruby scripts/ios/create-ios-project.rb
echo "Open apps/ios/TerraPhone.xcodeproj in Xcode."
