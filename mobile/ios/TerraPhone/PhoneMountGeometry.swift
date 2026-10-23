import simd
import Foundation
import CoreLocation

/// Upright portrait mount: top edge up, rear camera toward rover forward.
/// Phone +X is right, +Y is top, +Z points out of the screen (rover backward).
/// Rover +X is forward, +Y is left, +Z is up.
enum PhoneMountGeometry {
    static let phoneToBody = simd_quatf(simd_float3x3(columns: (
        SIMD3(0, -1, 0), SIMD3(0, 0, 1), SIMD3(-1, 0, 0)
    )))
    // ARCamera uses fixed landscape sensor axes: +X toward the phone bottom,
    // +Y toward the phone right, +Z out of the screen. Depth uses these axes too.
    static let cameraToBody = simd_quatf(simd_float3x3(columns: (
        SIMD3(0, 0, -1), SIMD3(0, -1, 0), SIMD3(-1, 0, 0)
    )))
    // Core Location landscapeLeft references phone +X (the right edge).
    // Unlike the portrait top edge, it stays horizontal in the upright mount.
    static let headingOrientation: CLDeviceOrientation = .landscapeLeft

    /// Convert the configured compass reference axis to calibrated rover forward.
    static func roverTrueHeading(compass: Double, roverYaw: Double, referenceYaw: Double) -> Double {
        guard compass.isFinite, compass >= 0, compass < 360, roverYaw.isFinite, referenceYaw.isFinite else { return -1 }
        let corrected = (compass - (roverYaw - referenceYaw) * 180 / .pi).truncatingRemainder(dividingBy: 360)
        return (corrected + 360).truncatingRemainder(dividingBy: 360)
    }
    static func headingReferenceYaw(worldFromCamera: simd_float3x3) -> Double? {
        // AR camera +Y is phone +X: pair exactly with headingOrientation above.
        let reference = worldFromCamera.columns.1
        guard reference.x.isFinite, reference.y.isFinite, reference.z.isFinite,
              hypot(reference.x, reference.y) >= 0.3 else { return nil }
        return atan2(Double(reference.y), Double(reference.x))
    }
}
