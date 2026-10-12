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
        print("Phone mount: forward, up, IMU axes and waypoint headings passed")
    }
}
