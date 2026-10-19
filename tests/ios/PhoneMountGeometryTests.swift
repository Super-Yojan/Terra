import Foundation
import simd

@main struct PhoneMountGeometryTests {
    static func main() {
        let mount = PhoneMountGeometry.phoneToBody
        func expect(_ actual: SIMD3<Float>, _ expected: SIMD3<Float>, _ message: String) {
            precondition(simd_length(actual - expected) < 0.00001, message)
        }
        expect(mount.act(SIMD3(0, 0, -1)), SIMD3(1, 0, 0), "Rear camera forward must be rover forward")
        expect(mount.act(SIMD3(0, 1, 0)), SIMD3(0, 0, 1), "Phone top must be rover up")
        expect(mount.act(SIMD3(1, 0, 0)), SIMD3(0, -1, 0), "Phone right must be rover right")
        let cameraMount = PhoneMountGeometry.cameraToBody
        expect(cameraMount.act(SIMD3(0, 0, -1)), SIMD3(1, 0, 0), "AR camera optical forward must be rover forward")
        expect(cameraMount.act(SIMD3(-1, 0, 0)), SIMD3(0, 0, 1), "AR camera negative X is phone top and rover up")
        expect(cameraMount.act(SIMD3(0, 1, 0)), SIMD3(0, -1, 0), "AR camera Y is phone right")
        for yaw: Float in [0, .pi / 2, -.pi / 2, .pi] {
            let worldFromBody = simd_quatf(angle: yaw, axis: SIMD3(0, 0, 1))
            let worldFromPhone = worldFromBody * mount
            let recoveredBody = worldFromPhone * mount.inverse
            expect(recoveredBody.act(SIMD3(1, 0, 0)), worldFromPhone.act(SIMD3(0, 0, -1)), "Waypoint forward must match rear camera at every heading")
            expect(recoveredBody.act(SIMD3(1, 0, 0)), SIMD3(cos(yaw), sin(yaw), 0), "Heading must preserve world yaw")
            let worldFromCamera = worldFromBody * cameraMount
            let arBody = worldFromCamera * cameraMount.inverse
            expect(arBody.act(SIMD3(1, 0, 0)), worldFromCamera.act(SIMD3(0, 0, -1)), "AR navigation must point along the observed depth cloud")
            expect(arBody.act(SIMD3(0, 0, 1)), SIMD3(0, 0, 1), "AR body must stay upright")
        }
        // An upright mount must have a valid compass reference even though its
        // top edge is vertical. Check all cardinal body headings and mild tilt.
        var failures = 0
        for yaw: Float in [0, .pi / 2, -.pi / 2, .pi] {
            for pitch: Float in [0, 0.2, -0.2] {
                let worldFromBody = simd_quatf(angle: yaw, axis: SIMD3(0, 0, 1))
                    * simd_quatf(angle: pitch, axis: SIMD3(0, 1, 0))
                let worldFromCamera = simd_float3x3(worldFromBody * cameraMount)
                guard let actual = PhoneMountGeometry.headingReferenceYaw(worldFromCamera: worldFromCamera) else {
                    print("FAIL: upright phone reference missing at yaw \(yaw), pitch \(pitch)")
                    failures += 1; continue
                }
                // landscapeLeft references phone +X (right), which is rover -Y.
                let expected = Double(yaw) - .pi / 2
                if abs(atan2(sin(actual - expected), cos(actual - expected))) > 0.00001 {
                    print("FAIL: reference yaw does not match the physical phone right edge")
                    failures += 1
                }
            }
        }
        let pointingUp = simd_float3x3(simd_quatf(angle: .pi / 2, axis: SIMD3(1, 0, 0)))
        if PhoneMountGeometry.headingReferenceYaw(worldFromCamera: pointingUp) != nil {
            print("FAIL: reject a vertical compass reference instead of inventing its yaw"); failures += 1
        }
        var invalid = matrix_identity_float3x3; invalid.columns.1.x = .nan
        if PhoneMountGeometry.headingReferenceYaw(worldFromCamera: invalid) != nil {
            print("FAIL: reject invalid reference orientation"); failures += 1
        }
        if failures > 0 { exit(1) }
        print("Phone mount: forward, up, IMU axes and upright compass alignment passed")
    }
}
