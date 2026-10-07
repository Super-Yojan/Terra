import Foundation

/// Screen up is forward; screen left is positive yaw in the rover body frame.
struct DriveJoystickCommand: Equatable {
    let forward: Double
    let yaw: Double
    static let zero = DriveJoystickCommand(forward: 0, yaw: 0)
    static func from(x: Double, y: Double, radius: Double, contactActive: Bool, enabled: Bool) -> Self {
        guard contactActive, enabled, x.isFinite, y.isFinite, radius.isFinite, radius > 0 else { return .zero }
        let distance = hypot(x, y)
        guard distance.isFinite else { return .zero }
        let amount = min(distance / radius, 1)
        let deadZone = 0.08
        guard amount > deadZone else { return .zero }
        let magnitude = (amount - deadZone) / (1 - deadZone)
        return Self(forward: -y / distance * magnitude, yaw: -x / distance * magnitude)
    }
}

enum DriveJoystickSafety {
    static func canArm(ready: Bool, manual: Bool, stopped: Bool, foreground: Bool) -> Bool {
        ready && manual && !stopped && foreground
    }
}
