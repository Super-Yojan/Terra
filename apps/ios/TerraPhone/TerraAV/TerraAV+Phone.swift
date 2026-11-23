import Foundation
import simd

// The Swift ↔ Rust boundary is generated from crates/terra-av/src/ffi.rs into
// apps/ios/Generated/TerraAV.swift by scripts/ios/build-ios.sh:
//
//     SensorFrame, VioPose, GpsFix, Vec3, Goal, Twist, DriveCommand, RoverStatus, TerraAv
//
// This file only adds phone-side conveniences.

extension Vec3 {
    init(_ v: SIMD3<Double>) { self.init(x: v.x, y: v.y, z: v.z) }
    init(_ v: SIMD3<Float>) { self.init(x: Double(v.x), y: Double(v.y), z: Double(v.z)) }
}

extension VioPose {
    /// From ARKit's `camera.transform` (column-major); rotation is sent row-major.
    init(arkit m: simd_float4x4, tracked: Bool) {
        let c = [m.columns.0, m.columns.1, m.columns.2]
        let rotation = (0..<3).flatMap { row in (0..<3).map { col in Double(c[col][row]) } }
        self.init(position: Vec3(x: Double(m.columns.3.x), y: Double(m.columns.3.y), z: Double(m.columns.3.z)),
                  rotation: rotation, tracked: tracked)
    }
}

extension RoverStatus {
    /// One line for the screen, e.g. "Tracking · Base connected · no task".
    var line: String {
        var parts = [tracked ? "Tracking" : "No pose", baseConnected ? "Base connected" : "No base"]
        if estopped { parts.append("E-STOP") }
        parts.append(task)
        return parts.joined(separator: " · ")
    }
}
