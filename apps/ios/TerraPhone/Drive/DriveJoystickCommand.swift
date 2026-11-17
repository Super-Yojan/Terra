import Foundation

/// Keep the first cause through subsequent stop events and reconnects.
struct TerraStopDiagnostic {
    private(set) var message: String?
    private(set) var timestamp: TimeInterval?
    mutating func record(_ reason: String, at time: TimeInterval) {
        guard message == nil, !reason.isEmpty else { return }
        message = reason
        timestamp = time
    }
    mutating func clear() { message = nil; timestamp = nil }
    static func autonomyReason(_ json: String) -> String {
        let root = (try? JSONSerialization.jsonObject(with: Data(json.utf8))) as? [String: Any]
        return (root?["status"] as? [String: Any])?["reason"] as? String ?? "unavailable"
    }

}

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
    static func canDrive(ready: Bool, manual: Bool, stopped: Bool, foreground: Bool) -> Bool {
        ready && manual && !stopped && foreground
    }
}

/// Reads of the Rust autonomy status JSON for presentation.
enum TerraFleetManualDrive {
    /// L3, and L5 tasks that drive on the L3 route, use phone VIO motion estimates,
    /// independently of the actuator routing format.
    static func requiresVioControl(_ json: String) -> Bool {
        guard let root = (try? JSONSerialization.jsonObject(with: Data(json.utf8))) as? [String: Any],
              let status = root["status"] as? [String: Any] else { return false }
        return ["waypoint", "task"].contains(status["requested_level"] as? String ?? "") || ["waypoint", "task"].contains(status["effective_level"] as? String ?? "")
    }
    static func vioReady(_ json: String) -> Bool {
        guard let root = (try? JSONSerialization.jsonObject(with: Data(json.utf8))) as? [String: Any],
              let quality = root["pose_quality"] as? [String: Any], quality["degraded"] as? Bool == false,
              let age = quality["pose_age_s"] as? Double, age.isFinite, (0...0.35).contains(age) else { return false }
        return true
    }

}

/// Search evidence belongs to the current rover run and has its own telemetry lease.
enum TerraTargetSearchPresentation {
    static func currentSearch(_ object:[String:Any])->[String:Any]? {
        let status=object["status"] as? [String:Any] ?? object
        guard let run=status["run_id"] as? String,let search=object["search"] as? [String:Any],search["run_id"] as? String==run else {return nil}
        return search
    }
}
