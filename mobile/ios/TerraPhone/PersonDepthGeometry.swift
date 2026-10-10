import Foundation
import simd

/// Raw camera pixels use a top-left origin; Vision rectangles use a bottom-left origin.
enum PersonImageOrientation: Int { case up, right, down, left }
struct PersonDepthGeometry {
    static func rawPoint(x: Float, y: Float, orientation: PersonImageOrientation) -> SIMD2<Float> {
        switch orientation {
        case .up: return SIMD2(x, 1-y)
        case .right: return SIMD2(1-y, 1-x)
        case .down: return SIMD2(1-x, y)
        case .left: return SIMD2(y, x)
        }
    }
    static func worldPoint(pixel: SIMD2<Float>, depth: Float, intrinsics: simd_float3x3,
                           cameraTransform: simd_float4x4, worldFromAR: simd_float3x3) -> SIMD3<Float>? {
        guard depth.isFinite, depth > 0, depth <= 20, pixel.x.isFinite, pixel.y.isFinite,
              intrinsics.columns.0.x > 0, intrinsics.columns.1.y > 0 else { return nil }
        let camera = SIMD4((pixel.x-intrinsics.columns.2.x)*depth/intrinsics.columns.0.x,
                           -(pixel.y-intrinsics.columns.2.y)*depth/intrinsics.columns.1.y, -depth, 1)
        let ar = cameraTransform * camera
        let world = worldFromAR * SIMD3(ar.x, ar.y, ar.z)
        return world.x.isFinite && world.y.isFinite && world.z.isFinite ? world : nil
    }
    static func measuredDepth(_ values: [Float]) -> Float? {
        let valid = values.filter { $0.isFinite && $0 > 0 && $0 <= 20 }.sorted()
        guard valid.count >= 5 else { return nil }
        let median = valid[valid.count/2]
        let agreeing = valid.filter { abs($0-median) <= max(0.15, median*0.1) }
        guard agreeing.count >= 5, agreeing.count * 2 >= valid.count else { return nil }
        return agreeing[agreeing.count/2]
    }
}

struct PersonDetectorFrameGate {
    private(set) var inFlight = false
    private var lastTimestamp = -Double.infinity
    private var sequence: UInt64 = 0
    mutating func begin(timestamp: Double) -> UInt64? {
        guard timestamp.isFinite, timestamp >= 0, !inFlight, timestamp-lastTimestamp >= 0.2 else { return nil }
        inFlight = true; lastTimestamp = timestamp; sequence &+= 1
        return sequence
    }
    mutating func finish() { inFlight = false }
}
