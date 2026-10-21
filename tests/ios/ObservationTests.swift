import Foundation
@main struct ObservationTests {
 static func main() {
  let pending = TerraObservation()
  pending.setContext(roverID: "5", sessionID: "latest")
  var pendingRover = ""
  pending.attach(roverID: "4", sessionID: "run", sink: { raw in pendingRover = (try! JSONSerialization.jsonObject(with: Data(raw.utf8)) as! [String: Any])["rover_id"] as! String; return true })
  pending.record(component: "phone", kind: "start", attributes: [:])
  precondition(pendingRover == "5", "late initialization must not undo the selected rover")
  let observer = TerraObservation()
  var records: [[String: Any]] = []
  observer.attach(roverID: "4", sessionID: "run", sink: { raw in records.append(try! JSONSerialization.jsonObject(with: Data(raw.utf8)) as! [String: Any]); return true })
  observer.recordChanged(component: "bluetooth", kind: "hardware.state", attributes: ["armed": true])
  observer.recordChanged(component: "bluetooth", kind: "hardware.state", attributes: ["armed": true])
  observer.setContext(roverID: "4", sessionID: "run")
  observer.recordChanged(component: "bluetooth", kind: "hardware.state", attributes: ["armed": true])
  observer.setContext(roverID: "5", sessionID: "next")
  observer.record(component: "control", kind: "motion.pause", attributes: ["reason": "heading"])
  precondition(records.count == 2)
  precondition(records[0]["rover_id"] as? String == "4")
  precondition(records[1]["rover_id"] as? String == "5")
  precondition((records[1]["attributes"] as! [String: Any])["reason"] as? String == "heading")
 }
}
