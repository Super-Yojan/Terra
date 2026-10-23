import Foundation
@main struct ObservationTests {
 static func main() {
  let pending = TerraObservation()
  pending.setContext(roverID: "5", sessionID: "latest")
  pending.recordSafetyRequest(reset: false, source: "operator", route: "bluetooth")
  pending.record(component: "bluetooth", kind: "connection.closed", attributes: ["reason": "startup_failure"])
  var early: [[String: Any]] = []
  pending.attach(roverID: "4", sessionID: "run", sink: { raw in early.append(try! JSONSerialization.jsonObject(with: Data(raw.utf8)) as! [String: Any]); return true })
  precondition(early.count == 2 && early[0]["session_id"] as? String == "latest")
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
  observer.recordSafetyRequest(reset: false, source: "operator", route: "bluetooth")
  observer.recordSafetyRequest(reset: true, source: "operator", route: "bluetooth")
  let stop = records[2]["attributes"] as! [String: Any]
  precondition(records[2]["kind"] as? String == "safety.requested")
  precondition(stop["action"] as? String == "emergency_stop" && stop["source"] as? String == "operator")
  precondition((records[3]["attributes"] as! [String: Any])["action"] as? String == "reset_emergency_stop")
  let status: [String: Any] = ["session": 7, "active_revision": 2, "last_sequence": 14,
      "emergency_stop": true, "stop_reason": "emergency_stop", "armed": false,
      "owner": "must-not-log", "token": "must-not-log"]
  observer.recordHardwareStatus(status)
  observer.recordHardwareStatus(status)
  precondition(records.count == 5, "unchanged hardware state should not flood the journal")
  let hardware = records[4]["attributes"] as! [String: Any]
  precondition(hardware["emergency_stop"] as? Bool == true && hardware["last_sequence"] as? Int == 14)
  precondition(hardware["owner"] == nil && hardware["token"] == nil)
 }
}
