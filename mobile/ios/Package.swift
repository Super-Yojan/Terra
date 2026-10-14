// swift-tools-version: 6.0
// Linux/xtool package for TerraPhone. Xcode keeps using TerraPhone.xcodeproj;
// open that project, not this directory, when building on a Mac.

import Foundation
import PackageDescription

let packageRoot = URL(fileURLWithPath: #filePath)
    .deletingLastPathComponent()
    .path

let package = Package(
    name: "TerraPhone",
    platforms: [
        .iOS(.v17),
    ],
    products: [
        .library(name: "TerraPhone", targets: ["TerraPhone"]),
    ],
    targets: [
        .systemLibrary(
            name: "TerraCoreFFI",
            path: "LinuxSupport"
        ),
        .target(
            name: "TerraPhone",
            dependencies: ["TerraCoreFFI"],
            path: "TerraPhone",
            exclude: [
                "Assets.xcassets",
                "Info.plist",
                "Models",
            ],
            swiftSettings: [
                .swiftLanguageMode(.v5),
            ],
            linkerSettings: [
                .unsafeFlags([
                    "-L", "\(packageRoot)/LinuxSupport/lib",
                    "-lterra_mobile",
                ]),
            ]
        ),
    ]
)
