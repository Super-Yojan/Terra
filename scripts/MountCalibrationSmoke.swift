import Foundation
import simd

@main struct MountCalibrationSmoke {
    static func main() {
        var mount = MountCalibration.uncalibrated
        mount.cameraHeight = 0.4
        mount.cameraOffsetForward = 0.2
        mount.cameraOffsetLeft = -0.1
        precondition(mount.isPlausible)
        let nominal = mount.arCameraToBody.act(SIMD3<Float>(0, 0, -1))
        precondition(simd_distance(nominal, SIMD3<Float>(1, 0, 0)) < 1e-5)
        mount.cameraYawDegrees = 90
        let rotated = mount.arCameraToBody.act(SIMD3<Float>(0, 0, -1))
        precondition(simd_distance(rotated, SIMD3<Float>(0, 1, 0)) < 1e-5)
        for direction in [-1.0, 1.0] {
            let pivot = SIMD2<Double>(3, -2)
            let samples = (0..<61).map { index -> RotationOffsetSample in
                let angle = 0.4 + direction * Double(index) * .pi / 90
                let camera = pivot + SIMD2(cos(angle) * 0.2 + sin(angle) * 0.1, sin(angle) * 0.2 - cos(angle) * 0.1)
                return RotationOffsetSample(timestamp: Double(index) / 20, cameraPositionWorld: camera, roverHeadingWorld: angle)
            }
            guard let fit = RotationOffsetFitter.fit(samples) else { fatalError("known turn rejected") }
            precondition(abs(fit.forward - 0.2) < 1e-8 && abs(fit.left + 0.1) < 1e-8)
            precondition(fit.turnDirection == Int(direction))
            for sample in samples {
                let q = simd_quatd(angle: sample.roverHeadingWorld, axis: SIMD3(0,0,1))
                let corrected = SIMD3(sample.cameraPositionWorld.x, sample.cameraPositionWorld.y, 0.4) - q.act(SIMD3(0.2,-0.1,0.4))
                precondition(simd_distance(corrected, SIMD3(pivot.x,pivot.y,0)) < 1e-8)
            }
            precondition(RotationOffsetFitter.fit(Array(samples.prefix(10))) == nil)
        }
        print("Mount calibration axes, two-way fit, lever arm and rejection checks passed")
    }
}
