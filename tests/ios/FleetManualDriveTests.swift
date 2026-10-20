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
  precondition(TerraFleetManualDrive.requiresVioControl("{\"status\":{\"requested_level\":\"waypoint\",\"effective_level\":\"none\"}}"))
  precondition(TerraFleetManualDrive.requiresVioControl("{\"status\":{\"effective_level\":\"waypoint\"}}"))
  precondition(!TerraFleetManualDrive.requiresVioControl("{\"status\":{\"requested_level\":\"waypoint_direct\"}}"))
  precondition(!TerraFleetManualDrive.requiresVioControl("{\"status\":{\"requested_level\":\"teleop\"}}"))
  precondition(TerraFleetManualDrive.command("{\"status\":{\"requested_level\":\"waypoint\",\"effective_level\":\"waypoint\",\"active_source\":\"waypoint\",\"safety\":\"clear\"},\"twist\":{\"linear\":1,\"angular\":0.2}}") == .zero)
  let vio = "{\"pose_quality\":{\"degraded\":false,\"pose_age_s\":0.1}}"
  precondition(TerraFleetManualDrive.vioReady(vio))
  precondition(!TerraFleetManualDrive.vioReady(vio.replacingOccurrences(of:"0.1",with:"0.4")))
  precondition(!TerraFleetManualDrive.vioReady("{}"))
  print("Fleet manual routing: accepted operator twist, stopped authority, autonomy and missing payload passed")
 }
}
