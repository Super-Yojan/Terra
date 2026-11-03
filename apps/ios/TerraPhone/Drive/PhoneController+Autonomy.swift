import OSLog
import ARKit
import AVFoundation
import Combine
import CoreMotion
import CoreLocation
import Foundation
import QuartzCore
import simd

// Waypoints, target search, autonomy levels, emergency stop and the authority/status refresh.
extension PhoneController {
    func engageWaypoint(originLatitude: Double, originLongitude: Double, latitude: Double, longitude: Double, token: String) {
        let token = token.trimmingCharacters(in: .whitespacesAndNewlines)
        controlQueue.async {
            let vioWaypoint = TerraFleetManualDrive.requiresVioControl(self.controller?.autonomyBrief() ?? "{}")
            let vioLayout = (try? ActuatorDriveProfileSet.decode(json: self.bleLayout).supportsVioControl) == true
            guard self.mode != .stopped, !self.bleActive || self.bleFeedback || (vioWaypoint && self.mode == .phone && vioLayout) else {
                DispatchQueue.main.async {
                    #if targetEnvironment(simulator)
                    self.waypointStatus = "Start the simulated rover, then go"
                    #else
                    self.waypointStatus = "Start a rover or connect over Bluetooth, then go"
                    #endif
                }
                return
            }
            do {
                let now=self.mode == .simulation ? self.simulatedTime:CACurrentMediaTime()
                try self.controller?.autonomyOrigin(latitude:originLatitude,longitude:originLongitude)
                let values:[String:Any]=["frame":"wgs84","latitude":latitude,"longitude":longitude,"token":token.isEmpty ? UUID().uuidString:token]
                let payload=String(decoding:try JSONSerialization.data(withJSONObject:values),as:UTF8.self)
                try self.controller?.autonomyRequest(kind:"goal",payload:payload,timestamp:now)
                DispatchQueue.main.async {
                    self.waypointActive = true
                    self.waypointDistance = 0
                    self.waypointStatus = "Goal sent · waiting for a pose"
                }
            } catch {
                DispatchQueue.main.async { self.waypointActive = false; self.waypointStatus = error.localizedDescription }
            }
        }
    }
    func cancelWaypoint() {
        controlQueue.async {
            let now=self.mode == .simulation ? self.simulatedTime:CACurrentMediaTime()
            try? self.controller?.autonomyRequest(kind:"goal",payload:"{\"cancel\":true}",timestamp:now)
            DispatchQueue.main.async {
                self.waypointActive = false
                self.waypointDistance = 0
                self.waypointStatus = "Teleop"
            }
        }
    }
    func startTargetSearch(targetClass:String,minX:Double,minY:Double,maxX:Double,maxY:Double,budget:Double) {
        controlQueue.async {
            do {
                let raw=self.controller?.autonomyStatus()
                let object=(try? JSONSerialization.jsonObject(with:Data((raw ?? "{}").utf8))) as? [String:Any] ?? [:]
                let status=object["status"] as? [String:Any] ?? object
                guard let run=status["run_id"] as? String,status["requested_level"] as? String=="target_search",[minX,minY,maxX,maxY,budget].allSatisfy({$0.isFinite}),minX<maxX,minY<maxY,maxX-minX<=100,maxY-minY<=100,budget>0,budget<=3600 else {throw NSError(domain:"TerraSearch",code:1,userInfo:[NSLocalizedDescriptionKey:"Select target search and enter valid local bounds and budget"])}
                let payload=String(decoding:try JSONSerialization.data(withJSONObject:["version":1,"run_id":run,"search_id":UUID().uuidString,"token":UUID().uuidString,"target_class":targetClass,"bounds":["min_x":minX,"min_y":minY,"max_x":maxX,"max_y":maxY],"time_budget_s":budget]),as:UTF8.self)
                try self.controller?.autonomyRequest(kind:"search",payload:payload,timestamp:self.mode == .simulation ? self.simulatedTime:CACurrentMediaTime())
            }catch{DispatchQueue.main.async{self.autonomyReason=error.localizedDescription}}
        }
    }
    func targetSearchAction(_ action:String) {
        controlQueue.async {
            do {
                let raw=self.controller?.autonomyStatus()
                let object=(try? JSONSerialization.jsonObject(with:Data((raw ?? "{}").utf8))) as? [String:Any] ?? [:]
                guard let search=TerraTargetSearchPresentation.currentSearch(object),let run=search["run_id"] as? String,let id=search["search_id"] as? String else {return}
                let payload=String(decoding:try JSONSerialization.data(withJSONObject:["version":1,"run_id":run,"search_id":id,"token":UUID().uuidString,"action":action]),as:UTF8.self)
                try self.controller?.autonomyRequest(kind:"search/action",payload:payload,timestamp:self.mode == .simulation ? self.simulatedTime:CACurrentMediaTime())
            }catch{DispatchQueue.main.async{self.autonomyReason=error.localizedDescription}}
        }
    }
    func setAutonomy(_ level:String, budgetSeconds: Double? = nil) {
        controlQueue.async {
            if self.bleActive && level != "teleop" {
                let vioLayout = (try? ActuatorDriveProfileSet.decode(json: self.bleLayout).supportsVioControl) == true
                guard self.bleFeedback || (level == "waypoint" && self.mode == .phone && vioLayout) else { return }
            }
            self.target=(0,0)
            do {
                var body: [String: Any] = ["level": level, "token": UUID().uuidString]
                if level == "explore" {
                    let budget = budgetSeconds ?? 120
                    if budget.isFinite, budget > 0 { body["budget_seconds"] = budget }
                }
                let payload=String(decoding:try JSONSerialization.data(withJSONObject:body),as:UTF8.self)
                try self.controller?.autonomyRequest(kind:"autonomy",payload:payload,timestamp:self.mode == .simulation ? self.simulatedTime:CACurrentMediaTime())
            }catch{DispatchQueue.main.async{self.autonomyReason=error.localizedDescription}}
        }
    }
    func emergencyStop(reset:Bool=false) {
        controlQueue.async {
            self.observation.recordSafetyRequest(reset: reset, source: "operator", route: self.bleActive ? "bluetooth" : "local_authority")
            self.target = (0, 0); self.resetServoTargets()
            self.dashboardEmergencyStop = !reset
            if self.bleActive || self.bluetooth.connectedIdentifier != nil { self.bluetooth.stopOutput() }
            let payload = "{\"action\":\"\(reset ? "reset" : "stop")\",\"token\":\"\(UUID().uuidString)\"}"
            try? self.controller?.autonomyRequest(kind: "safety", payload: payload, timestamp: self.mode == .simulation ? self.simulatedTime : CACurrentMediaTime())
        }
    }
    func exportRun() {
        controlQueue.async {do {try self.controller?.endRecording()}catch{DispatchQueue.main.async{self.autonomyReason=error.localizedDescription}}}
    }
    func decideProposal(_ id:UInt64,approve:Bool) {
        let run=proposalRun
        controlQueue.async {do {let payload=String(decoding:try JSONSerialization.data(withJSONObject:["run_id":run,"proposal_id":id,"decision":approve ? "approve":"reject","token":UUID().uuidString]),as:UTF8.self);try self.controller?.autonomyRequest(kind:"goal/decision",payload:payload,timestamp:self.mode == .simulation ? self.simulatedTime:CACurrentMediaTime())}catch{DispatchQueue.main.async{self.autonomyReason=error.localizedDescription}}}
    }
    func updateAuthority() {
        if let run = controller?.observationRunId(), !run.isEmpty {
            observation.setContext(roverID: UserDefaults.standard.string(forKey: "dashboardRoverID") ?? "_producer", sessionID: run)
        }
        observation.recordChanged(component: "router", kind: "connection.state", attributes: ["status": controller?.dashboardStatus() ?? "unavailable"])
        if let controller {
            let vioReady = mode == .phone && TerraFleetManualDrive.vioReady(controller.autonomyBrief())
            DispatchQueue.main.async {
                let remote = (try? JSONSerialization.jsonObject(with: Data(self.bluetooth.statusJSON.utf8))) as? [String: Any] ?? [:]
                var state: [String: Any] = ["schema_version": 1, "ready": self.hardwareReady,
                    "connected": self.bluetooth.connectedIdentifier != nil, "control_mode": "streamed_relay", "output_active": self.hardwareOutputActive, "motion_feedback_source": "phone_vio", "motion_feedback_ready": vioReady,
                    "reason": self.hardwareStatus, "phone_mounted": self.phoneMounted]
                state["stop_diagnostic"] = self.lastStopDiagnostic.message
                state["stop_reason"] = remote["stop_reason"]
                if let fault = self.hardwareFault { state["fault"] = fault }
                if let payload = try? JSONSerialization.data(withJSONObject: state) {
                    try? controller.publishHardwareStatus(payload: String(decoding: payload, as: UTF8.self))
                }
            }
        }

        let raw=controller?.autonomyStatus()
        guard let raw,let data=raw.data(using:.utf8),let object=(try? JSONSerialization.jsonObject(with:data)) as? [String:Any] else{return}
        #if DEBUG
        if tick % 50 == 0 {
            let snapshot: [String: Any] = ["time": ISO8601DateFormatter().string(from: Date()), "mode": String(describing: mode), "feedback": bleFeedback, "feedbackTrackingHealthy": feedbackTrackingHealthy, "router": controller?.dashboardStatus() ?? "none", "autonomy": object]
            let file = FileManager.default.urls(for: .documentDirectory, in: .userDomainMask)[0].appendingPathComponent("ControlDiagnostics.json")
            if let bytes = try? JSONSerialization.data(withJSONObject: snapshot) { try? bytes.write(to: file, options: .atomic) }
        }
        #endif
        let status=(object["status"] as? [String:Any]) ?? object
        let level=status["requested_level"] as? String ?? "teleop"
        let outdoor = object["outdoor"] as? [String: Any]
        let reason: String
        if outdoor?["source"] as? String == "outdoor_gps" {
            let phase = outdoor?["hold_reason"] as? String ?? outdoor?["phase"] as? String ?? "idle"
            let accuracy = outdoor?["gps_accuracy_m"] as? Double
            reason = "Outdoor GPS · " + phase.replacingOccurrences(of: "_", with: " ") + (accuracy.map { String(format: " · ±%.1f m", $0) } ?? "")
        } else { reason = status["reason"] as? String ?? "Unknown" }
        let goal=object["goal"] as? [String:Any]
        let proposal=object["proposal"] as? [String:Any]
        let reports=(object["pending_reports"] as? [[String:Any]]) ?? []
        for report in reports {
            guard let run=report["run_id"] as? String,let search=report["search_id"] as? String,let id=report["report_id"] as? String else {continue}
            let identity="\(run):\(search):\(id)"
            let now=CACurrentMediaTime()
            if searchReportReceipts[identity].map({now-$0<1}) == true {continue}
            if searchReportReceipts[identity]==nil {
                let line=String(format:"%@ · %.2f, %.2f m · %d frames",report["target_class"] as? String ?? "Target",report["world_x"] as? Double ?? 0,report["world_y"] as? Double ?? 0,(report["frame_ids"] as? [Any])?.count ?? 0)
                DispatchQueue.main.async {self.searchReportHistory=Array((self.searchReportHistory+[line]).suffix(128))}
            }
            do {
                let payload=String(decoding:try JSONSerialization.data(withJSONObject:["version":1,"run_id":run,"search_id":search,"report_id":id,"token":UUID().uuidString]),as:UTF8.self)
                try self.controller?.autonomyRequest(kind:"search/report/ack",payload:payload,timestamp:self.mode == .simulation ? self.simulatedTime:now)
                searchReportReceipts[identity]=now
            }catch{DispatchQueue.main.async{self.autonomyReason="Report acknowledgement failed: \(error.localizedDescription)"}}
        }
        searchReportReceipts=searchReportReceipts.filter{CACurrentMediaTime()-$0.value<5}
        let search=TerraTargetSearchPresentation.currentSearch(object)
        let searchPhase=search?["phase"] as? String ?? ""
        let report=search?["report"] as? [String:Any]
        let available=(status["supported_levels"] as? [String] ?? []).contains("target_search")
        DispatchQueue.main.async {
            self.searchAvailable=available
            self.searchPhase=searchPhase
            self.searchTerminal=searchPhase.isEmpty || ["completed","cancelled","timed_out","exhausted"].contains(searchPhase)
            self.searchReportText=report.map{String(format:"Target confirmed at %.2f, %.2f m · %d frames",$0["world_x"] as? Double ?? 0,$0["world_y"] as? Double ?? 0,($0["frame_ids"] as? [Any])?.count ?? 0)} ?? ""
        }
        DispatchQueue.main.async {self.autonomyLevel=level;self.autonomyReason=reason;self.proposalRun=proposal?["run_id"] as? String ?? "";self.proposedGoal=(proposal?["proposal_id"] as? NSNumber)?.uint64Value;self.proposalText=proposal.map{String(format:"Search target %.1f, %.1f m",$0["x"] as? Double ?? 0,$0["y"] as? Double ?? 0)} ?? "";if let goal{self.waypointActive=(goal["state"] as? String)=="active";self.waypointDistance=goal["distance"] as? Double ?? 0;self.waypointStatus=goal["state"] as? String ?? "Unknown"}}
        let exploration=(object["exploration"] as? [String:Any]) ?? (status["exploration"] as? [String:Any])
        let summary: String = {
            guard let exploration, let phase=exploration["phase"] as? String, phase != "idle" else { return "" }
            let remaining=exploration["remaining_seconds"] as? Double ?? 0
            let coverage=exploration["coverage_m2"] as? Double ?? 0
            if let end=exploration["end_reason"] as? String {
                return String(format:"Explore ended · %@ · %.1f m² observed", end.replacingOccurrences(of:"_", with:" "), coverage)
            }
            if let target=exploration["target"] as? [String:Any] {
                let x=target["x"] as? Double ?? 0, y=target["y"] as? Double ?? 0
                return String(format:"Exploring · %.0f s left · %.1f m² · target %.1f, %.1f m", remaining, coverage, x, y)
            }
            return String(format:"Exploring · %.0f s left · %.1f m²", remaining, coverage)
        }()
        DispatchQueue.main.async {self.explorationSummary=summary}
    }
    /// Runs on `controlQueue`. The outlet sits over the phone, so the dock aims
    /// the camera's forward offset at the bucket once it is measured.
    func applyDock(to controller: MobileController) {
        let ahead = mountCalibration.isPlausible && mountCalibration.cameraOffsetForward >= 0.05 ? mountCalibration.cameraOffsetForward : nil
        try? controller.setDockOutletAhead(metres: ahead)
        do { try controller.setDumpGate(gate: dumpGate?.rust) }
        catch { DispatchQueue.main.async { self.dumpGateStatus = "Gate setting rejected: \(error)" } }
    }
}
