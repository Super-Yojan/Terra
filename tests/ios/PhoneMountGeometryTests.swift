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
        if PhoneMountGeometry.headingReference(deviceToBody: mount) != PhoneMountGeometry.uprightHeadingReference {
            print("FAIL: upright mount must keep the right-edge compass reference"); failures += 1
        }
        // Landscape mount, rear camera forward: roll about rover forward moves the
        // top edge horizontal (-90 deg: toward rover left) and the right edge vertical.
        for (roll, topEdgeYaw): (Float, Double) in [(-.pi / 2, .pi / 2), (.pi / 2, -.pi / 2)] {
            let correction = simd_quatf(angle: roll, axis: SIMD3(1, 0, 0))
            let edge = PhoneMountGeometry.headingReference(deviceToBody: correction * mount)
            if edge.orientation != .portrait {
                print("FAIL: landscape mount at roll \(roll) must reference the horizontal top edge"); failures += 1
            }
            for yaw: Float in [0, .pi / 2, -.pi / 2, .pi] {
                for pitch: Float in [0, 0.2, -0.2] {
                    let worldFromBody = simd_quatf(angle: yaw, axis: SIMD3(0, 0, 1))
                        * simd_quatf(angle: pitch, axis: SIMD3(0, 1, 0))
                    let worldFromCamera = simd_float3x3(worldFromBody * correction * cameraMount)
                    if PhoneMountGeometry.headingReferenceYaw(worldFromCamera: worldFromCamera) != nil {
                        print("FAIL: the vertical right edge must not serve as a landscape reference"); failures += 1
                    }
                    guard let actual = PhoneMountGeometry.headingReferenceYaw(worldFromCamera: worldFromCamera, using: edge) else {
                        print("FAIL: landscape reference missing at roll \(roll), yaw \(yaw), pitch \(pitch)")
                        failures += 1; continue
                    }
                    let expected = Double(yaw) + topEdgeYaw
                    if abs(atan2(sin(actual - expected), cos(actual - expected))) > 0.00001 {
                        print("FAIL: landscape reference yaw does not match the physical top edge"); failures += 1
                    }
                    // Compass reads the top edge's true heading; the rover faces 90 deg away.
                    let compass = (200 - topEdgeYaw * 180 / .pi + 360).truncatingRemainder(dividingBy: 360)
                    let rover = PhoneMountGeometry.roverTrueHeading(compass: compass, roverYaw: Double(yaw), referenceYaw: actual)
                    if abs(atan2(sin((rover - 200) * .pi / 180), cos((rover - 200) * .pi / 180))) > 0.0001 {
                        print("FAIL: landscape compass must convert to rover true heading"); failures += 1
                    }
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
        for offset in [0.0, 0.4, -.pi / 2, .pi] {
            let actual = PhoneMountGeometry.roverTrueHeading(compass: 100, roverYaw: 1.2 + offset, referenceYaw: 1.2)
            let expected = (100 - offset * 180 / .pi + 720).truncatingRemainder(dividingBy: 360)
            precondition(abs(actual - expected) < 0.000001)
        }
        precondition(PhoneMountGeometry.roverTrueHeading(compass: -1, roverYaw: 0, referenceYaw: 0) == -1)
        if failures > 0 { exit(1) }
        print("Phone mount: forward, up, IMU axes, upright and landscape compass alignment passed")
    }
}
