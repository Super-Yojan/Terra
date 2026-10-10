import Foundation

/// Screen up is forward; screen left is positive yaw in the rover body frame.
struct DriveJoystickCommand: Equatable {
    let forward: Double
    let yaw: Double
    var wheelEfforts: (left: Double, right: Double) {
        guard forward.isFinite, yaw.isFinite else { return (0, 0) }
        let left = forward - yaw, right = forward + yaw
        let scale = max(1, max(abs(left), abs(right)))
        return (left / scale, right / scale)
    }
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
    static func trackingLossRequiresDisarm(feedback: Bool, fullyManual: Bool) -> Bool {
        feedback && !fullyManual
    }
    static func hardwareArmAllowed(feedback: Bool, fullyManual: Bool, trackingHealthy: Bool, compatible: Bool) -> Bool {
        !feedback || fullyManual || (trackingHealthy && compatible)
    }
    static func canArm(ready: Bool, manual: Bool, stopped: Bool, foreground: Bool) -> Bool {
        ready && manual && !stopped && foreground
    }
}

/// Route only the arbiter-selected manual intent, never an unvalidated network payload.
enum TerraFleetManualDrive {
    static func localWaypointActive(_ json: String) -> Bool {
        guard let root = (try? JSONSerialization.jsonObject(with: Data(json.utf8))) as? [String: Any],
              let status = root["status"] as? [String: Any], let goal = root["goal"] as? [String: Any] else { return false }
        return status["requested_level"] as? String == "waypoint" && goal["state"] as? String == "active"
    }
    static func isFullyManual(_ json: String) -> Bool {
        guard let root = (try? JSONSerialization.jsonObject(with: Data(json.utf8))) as? [String: Any],
              let status = root["status"] as? [String: Any] else { return false }
        // An idle/held L0 has no effective level. Mode selection must use
        // the requested level; command() separately enforces live authority.
        return (status["requested_level"] as? String ?? status["effective_level"] as? String) == "teleop"
    }
    static func command(_ json: String) -> DriveJoystickCommand {
        guard let root = (try? JSONSerialization.jsonObject(with: Data(json.utf8))) as? [String: Any],
              let status = root["status"] as? [String: Any],
              ((status["effective_level"] as? String == "teleop" && status["active_source"] as? String == "operator")
               || (status["effective_level"] as? String == "assisted_teleop" && status["active_source"] as? String == "assisted_operator")),
              ["teleop", "assisted_teleop"].contains(status["effective_level"] as? String ?? ""),
              ["clear", "active"].contains(status["safety"] as? String ?? ""),
              let twist = root["twist"] as? [String: Any],
              let linear = twist["linear"] as? Double, let angular = twist["angular"] as? Double,
              linear.isFinite, angular.isFinite, abs(linear) <= 0.5, abs(angular) <= 1 else { return .zero }
        print("I am here")
        return DriveJoystickCommand(forward: linear / 0.5, yaw: angular)
    }
}

/// Search evidence belongs to the current rover run and has its own telemetry lease.
enum TerraTargetSearchPresentation {
    static func currentSearch(_ object:[String:Any],remote:Bool)->[String:Any]? {
        let status=object["status"] as? [String:Any] ?? object
        guard let run=status["run_id"] as? String,let search=object["search"] as? [String:Any],search["run_id"] as? String==run else {return nil}
        if remote {guard let age=object["search_age"] as? Double,age.isFinite,age>=0,age<0.5 else{return nil}}
        return search
    }
}
