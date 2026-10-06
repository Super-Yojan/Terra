import Foundation
import simd

/// Phone rear-camera pose relative to the rover reference point.
/// Angles are degrees; offsets and height are metres.
struct MountCalibration: Codable {
    var cameraOffsetForward: Double
    var cameraOffsetLeft: Double
    var cameraHeight: Double
    var cameraRollDegrees: Double
    var cameraPitchDegrees: Double
    var cameraYawDegrees: Double
    var reviewed: Bool

    static let uncalibrated = MountCalibration(
        cameraOffsetForward: 0,
        cameraOffsetLeft: 0,
        cameraHeight: 0,
        cameraRollDegrees: 0,
        cameraPitchDegrees: 0,
        cameraYawDegrees: 0,
        reviewed: false
    )

    private static let storageKey = "terra.mountCalibration.v1"

    static func load() -> MountCalibration {
        guard let data = UserDefaults.standard.data(forKey: storageKey),
              let value = try? JSONDecoder().decode(MountCalibration.self, from: data) else {
            return .uncalibrated
        }
        return value
    }

    func save() throws {
        let data = try JSONEncoder().encode(self)
        UserDefaults.standard.set(data, forKey: Self.storageKey)
    }

    var isPlausible: Bool {
        let values = [cameraOffsetForward, cameraOffsetLeft, cameraHeight,
                      cameraRollDegrees, cameraPitchDegrees, cameraYawDegrees]
        return values.allSatisfy(\.isFinite)
            && abs(cameraOffsetForward) <= 2
            && abs(cameraOffsetLeft) <= 2
            && cameraHeight > 0 && cameraHeight <= 2
            && abs(cameraRollDegrees) <= 180
            && abs(cameraPitchDegrees) <= 180
            && abs(cameraYawDegrees) <= 180
    }

    private var correction: simd_quatf {
        let radians = Float.pi / 180
        let roll = simd_quatf(angle: Float(cameraRollDegrees) * radians, axis: SIMD3(1, 0, 0))
        let pitch = simd_quatf(angle: Float(cameraPitchDegrees) * radians, axis: SIMD3(0, 1, 0))
        let yaw = simd_quatf(angle: Float(cameraYawDegrees) * radians, axis: SIMD3(0, 0, 1))
        return yaw * pitch * roll
    }

    /// Nominal upright, rear-camera-forward phone axes, plus measured mount correction.
    var deviceToBody: simd_quatf {
        let nominal = simd_quatf(simd_float3x3(columns: (
            SIMD3(0, -1, 0), SIMD3(0, 0, 1), SIMD3(-1, 0, 0)
        )))
        return correction * nominal
    }

    /// Optical camera axes are right/down/forward; device axes are right/up/toward screen.
    var opticalCameraToBody: simd_quatf {
        let opticalToDevice = simd_quatf(angle: .pi, axis: SIMD3(1, 0, 0))
        return deviceToBody * opticalToDevice
    }
}
