import simd

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
}
