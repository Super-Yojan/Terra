import Foundation
@main struct FleetManualTests {
 static func main() {
  let live = "{\"status\":{\"active_source\":\"operator\",\"effective_level\":\"teleop\",\"safety\":\"clear\"},\"twist\":{\"linear\":0.25,\"angular\":-0.5}}"
  precondition(TerraFleetManualDrive.command(live) == DriveJoystickCommand(forward: 0.5, yaw: -0.5))
  precondition(TerraFleetManualDrive.command(live.replacingOccurrences(of: "operator", with: "none")) == .zero)
  precondition(TerraFleetManualDrive.command(live.replacingOccurrences(of: "clear", with: "hold")) == .zero)
  precondition(TerraFleetManualDrive.command(live.replacingOccurrences(of: "teleop", with: "waypoint")) == .zero)
  let assisted = live.replacingOccurrences(of: "operator", with: "assisted_operator").replacingOccurrences(of: "teleop", with: "assisted_teleop")
  precondition(TerraFleetManualDrive.command(assisted) == DriveJoystickCommand(forward: 0.5, yaw: -0.5))
  precondition(TerraFleetManualDrive.command("{}") == .zero)
  precondition(TerraFleetManualDrive.isFullyManual(live))
  let idle = "{\"status\":{\"requested_level\":\"teleop\",\"effective_level\":null,\"active_source\":\"none\",\"safety\":\"hold\"},\"twist\":{\"linear\":0,\"angular\":0}}"
  precondition(TerraFleetManualDrive.isFullyManual(idle))
  precondition(TerraFleetManualDrive.command(idle) == .zero)
  precondition(!TerraFleetManualDrive.isFullyManual(assisted))
  let wheels = DriveJoystickCommand(forward: 0.5, yaw: -0.5).wheelEfforts
  precondition(wheels.left == 1 && wheels.right == 0)
  let stopped = DriveJoystickCommand.zero.wheelEfforts
  precondition(stopped.left == 0 && stopped.right == 0)
  precondition(DriveJoystickSafety.hardwareArmAllowed(feedback: true, fullyManual: true, trackingHealthy: false, compatible: true))
  precondition(!DriveJoystickSafety.hardwareArmAllowed(feedback: true, fullyManual: false, trackingHealthy: false, compatible: true))
  precondition(DriveJoystickSafety.hardwareArmAllowed(feedback: true, fullyManual: false, trackingHealthy: true, compatible: true))
  precondition(TerraFleetManualDrive.localWaypointActive("{\"status\":{\"requested_level\":\"waypoint_direct\"},\"goal\":{\"state\":\"active\"}}"))
  print("Fleet manual routing: accepted operator twist, stopped authority, autonomy and missing payload passed")
 }
}
