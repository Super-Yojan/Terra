import Foundation
@main struct FleetManualTests {
 static func main() {
  precondition(TerraFleetManualDrive.requiresVioControl("{\"status\":{\"requested_level\":\"waypoint\",\"effective_level\":\"none\"}}"))
  precondition(TerraFleetManualDrive.requiresVioControl("{\"status\":{\"effective_level\":\"waypoint\"}}"))
  precondition(!TerraFleetManualDrive.requiresVioControl("{\"status\":{\"requested_level\":\"waypoint_direct\"}}"))
  precondition(!TerraFleetManualDrive.requiresVioControl("{\"status\":{\"requested_level\":\"teleop\"}}"))
  let vio = "{\"pose_quality\":{\"degraded\":false,\"pose_age_s\":0.1}}"
  precondition(TerraFleetManualDrive.vioReady(vio))
  precondition(!TerraFleetManualDrive.vioReady(vio.replacingOccurrences(of:"0.1",with:"0.4")))
  precondition(!TerraFleetManualDrive.vioReady("{}"))
  print("Fleet status reads: VIO-controlled levels and pose readiness passed")
 }
}
