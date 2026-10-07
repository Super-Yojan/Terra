import Foundation
@main struct DriveJoystickTests {
    static func main() {
        func near(_ actual: Double, _ expected: Double) {
            precondition(abs(actual - expected) < 0.000001, "Expected \(expected), got \(actual)")
        }
        func command(_ x: Double, _ y: Double, contact: Bool = true, enabled: Bool = true) -> DriveJoystickCommand {
            DriveJoystickCommand.from(x: x, y: y, radius: 100, contactActive: contact, enabled: enabled)
        }
        near(command(0, -100).forward, 1) // Up drives forward.
        near(command(0, 100).forward, -1)
        near(command(-100, 0).yaw, 1) // Left is positive yaw in the rover frame.
        near(command(100, 0).yaw, -1)
        near(command(0, 0).forward, 0)
        near(command(3, -3).forward, 0) // Small finger tremor stays in the dead zone.
        let diagonal = command(1000, -1000)
        near(diagonal.forward, 1 / sqrt(2)); near(diagonal.yaw, -1 / sqrt(2))
        near(command(0, -100, contact: false).forward, 0) // Release or gesture cancellation.
        near(command(0, -100, enabled: false).forward, 0) // Disarm, stop or tracking loss.
        near(command(.nan, 0).yaw, 0)
        near(command(0, .infinity).forward, 0)
        near(DriveJoystickCommand.from(x: 0, y: -100, radius: 0, contactActive: true, enabled: true).forward, 0)
        precondition(DriveJoystickSafety.canArm(ready: true, manual: true, stopped: false, foreground: true))
        precondition(!DriveJoystickSafety.canArm(ready: false, manual: true, stopped: false, foreground: true))
        precondition(!DriveJoystickSafety.canArm(ready: true, manual: false, stopped: false, foreground: true))
        precondition(!DriveJoystickSafety.canArm(ready: true, manual: true, stopped: true, foreground: true))
        precondition(!DriveJoystickSafety.canArm(ready: true, manual: true, stopped: false, foreground: false))
        print("Joystick direction, bounds, dead zone, release and arming safeguards passed")
    }
}
