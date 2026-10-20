import Foundation
@main struct FleetTurningTests {
 static func main() {
  func input(_ origin:String,_ spin:Bool)->String { """
  {"status":{"effective_level":"teleop","active_source":"operator","safety":"clear"},"twist":{"linear":0.1,"angular":0.5},"command_origin":"\(origin)","navigation":{"parameters":{"turning":{"allow_in_place":\(spin)}}}}
  """ }
  let fleet=TerraFleetManualDrive.command(input("fleet_teleop",false)).wheelEfforts
  precondition(fleet.left>=0 && fleet.right>=0,"fleet must not counterrotate")
  let spin=TerraFleetManualDrive.command(input("fleet_teleop",true)).wheelEfforts
  precondition(spin.left<0 && spin.right>0)
  let local=TerraFleetManualDrive.command(input("local_manual",false)).wheelEfforts
  precondition(local.left<0 && local.right>0,"local manual stays exempt")
  precondition(!DriveJoystickSafety.trackingLossRequiresDisarm(feedback:false,fullyManual:true))
  print("Fleet wheel constraints and direct manual exemption passed")
 }
}
