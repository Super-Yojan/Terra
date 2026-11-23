import Foundation

/// Wheel efforts in [-1, 1], ready for the motor driver.
struct WheelCommand: Equatable {
    var left: Double
    var right: Double
    static let stop = WheelCommand(left: 0, right: 0)
}

/// Execution, platform half: the proportional velocity loop (roadmap 3).
/// Runs on its own fixed-rate timer, independent of the terra-av tick, so a
/// slow planner tick never delays the motors.
///
///     effort = feedforward(target) + kp · (target − measured)
///
/// then mixed onto the two sides of a skid-steer base.
struct VelocityController {
    /// Terra (CrabTrapBot). top_speed is the measured ground speed at full
    /// effort (~1.0 m/s, thesis Ch7), not the old assumed 3.
    var trackWidth = 0.5 // m
    var topSpeed = 1.0 // m/s at effort 1
    var skidFactor = 2.4 // yaw effort multiplier for wheel scrub
    // NOTE: The gains aren't really tuned
    var kpLinear = 0.1 // effort per m/s of error
    var kpYaw = 0.1 // effort per rad/s of error
    /// Stop if the planner's target is older than this.
    var targetTimeout = 0.3 // s

    func command(target: Twist, measured: Twist?, targetAge: TimeInterval) -> WheelCommand {
        guard targetAge.isFinite, targetAge < targetTimeout else { return .stop }
        if target.v == 0 && target.w == 0 {
            return .stop
        }

        var forward = target.v / topSpeed
        var yaw = skidFactor * trackWidth / 2 * target.w / topSpeed
        // No feedback until estimation has a tracked pose: feedforward only.
        if let measured {
            forward += kpLinear * (target.v - measured.v)
            yaw += kpYaw * (target.w - measured.w)
        }
        // NOTE: Setting to to be 0.3 when testing on the terra rover,
        // this needs to be configurable depending on the motor
        let clamp = { (x: Double) in min(0.3, max(-0.3, x)) }
        return WheelCommand(left: clamp(forward - yaw), right: clamp(forward + yaw))
    }
}
