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
    /// A Core Location heading orientation paired with the AR camera axis of the
    /// phone edge it references.
    struct HeadingReference: Equatable {
        let orientation: CLDeviceOrientation
        let cameraAxis: SIMD3<Float>
    }
    // Core Location landscapeLeft references phone +X (the right edge, AR +Y);
    // portrait references phone +Y (the top edge, AR -X).
    private static let headingCandidates: [(phoneAxis: SIMD3<Float>, reference: HeadingReference)] = [
        (SIMD3(1, 0, 0), HeadingReference(orientation: .landscapeLeft, cameraAxis: SIMD3(0, 1, 0))),
        (SIMD3(0, 1, 0), HeadingReference(orientation: .portrait, cameraAxis: SIMD3(-1, 0, 0))),
    ]
    /// The upright mount keeps its right edge horizontal while the top edge is vertical.
    static let uprightHeadingReference = headingCandidates[0].reference

    /// Reference the phone edge that lies most horizontal on the rover, so a rolled
    /// (for example landscape) mount keeps a valid compass reference.
    static func headingReference(deviceToBody: simd_quatf) -> HeadingReference {
        var best = headingCandidates[0].reference
        var bestHorizontal: Float = -1
        for candidate in headingCandidates {
            let edge = deviceToBody.act(candidate.phoneAxis)
            let horizontal = hypot(edge.x, edge.y)
            if horizontal > bestHorizontal + 0.001 {
                best = candidate.reference
                bestHorizontal = horizontal
            }
        }
        return best
    }

    /// Convert the configured compass reference axis to calibrated rover forward.
    static func roverTrueHeading(compass: Double, roverYaw: Double, referenceYaw: Double) -> Double {
        guard compass.isFinite, compass >= 0, compass < 360, roverYaw.isFinite, referenceYaw.isFinite else { return -1 }
        let corrected = (compass - (roverYaw - referenceYaw) * 180 / .pi).truncatingRemainder(dividingBy: 360)
        return (corrected + 360).truncatingRemainder(dividingBy: 360)
    }
    static func headingReferenceYaw(worldFromCamera: simd_float3x3, using edge: HeadingReference = uprightHeadingReference) -> Double? {
        // Pair exactly with the orientation given to Core Location.
        let reference = worldFromCamera * edge.cameraAxis
        guard reference.x.isFinite, reference.y.isFinite, reference.z.isFinite,
              hypot(reference.x, reference.y) >= 0.3 else { return nil }
        return atan2(Double(reference.y), Double(reference.x))
    }
}
