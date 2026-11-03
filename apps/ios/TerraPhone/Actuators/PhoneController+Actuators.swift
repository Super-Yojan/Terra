import OSLog
import ARKit
import AVFoundation
import Combine
import CoreMotion
import CoreLocation
import Foundation
import QuartzCore
import simd

// Actuator layouts, servos, dump gate, rover profiles and the actuator editor.
extension PhoneController {
    /// The bucket dock's dump-gate position from Rust. Swift only routes it.
    func applyGateServo(_ gate: GateServo?) {
        guard let gate, servoTargets[String(gate.id)] != gate.position else { return }
        servoTargets[String(gate.id)] = gate.position
        DispatchQueue.main.async { self.servoPositions[gate.id] = gate.position }
    }
    func setServoTarget(id: UInt8, position: Double) {
        guard position.isFinite, hardwareReady else { return }
        controlQueue.async {
            guard let layout = try? ActuatorDriveProfileSet.decode(json: self.bleLayout),
                  let actuator = layout.actuators.first(where: { $0.id == Int(id) && $0.kind == "positional_servo" }) else { return }
            let value = min(actuator.limits.max, max(actuator.limits.min, position))
            self.servoTargets[String(id)] = value
            try? self.controller?.setServoTarget(id: id, value: value)
            DispatchQueue.main.async { self.servoPositions[id] = value }
        }
    }
    func resetServoTargets() {
        servoTargets = [:]
        rustLayoutSync = nil // Reloading the layout in Rust also resets its servo targets.
        if let layout = try? ActuatorDriveProfileSet.decode(json: bleLayout) {
            for a in layout.actuators where a.kind == "positional_servo" {
                servoTargets[String(a.id)] = min(a.limits.max, max(a.limits.min, a.safe.value ?? 0))
            }
        }
        let values = servoTargets.reduce(into: [UInt8: Double]()) { result, entry in if let id = UInt8(entry.key) { result[id] = entry.value } }
        DispatchQueue.main.async { self.servoPositions = values }
    }
    /// Save the dump gate servo and positions (nil clears them) and hand them to Rust.
    func setDumpGate(_ value: DumpGateSetting?) {
        DumpGateSetting.save(value)
        dumpGate = value
        controlQueue.async {
            guard let controller = self.controller else { return }
            do {
                try controller.setDumpGate(gate: value?.rust)
                DispatchQueue.main.async { self.dumpGateStatus = value == nil ? "Gate cleared" : "Gate saved" }
            } catch { DispatchQueue.main.async { self.dumpGateStatus = "Gate setting rejected: \(error)" } }
        }
    }
    /// Move the gate to its open or closed position to check it, or end the test (nil).
    /// Rust refuses while a dock runs; with no controller the servo is set directly.
    func testDumpGate(open: Bool?) {
        guard let gate = dumpGate, let id = UInt8(exactly: gate.servoId) else { return }
        controlQueue.async {
            do {
                if let controller = self.controller {
                    try controller.setDumpGate(gate: gate.rust)
                    try controller.testDumpGate(open: open)
                }
                if let open {
                    DispatchQueue.main.async { self.setServoTarget(id: id, position: open ? gate.openPosition : gate.closedPosition) }
                }
                DispatchQueue.main.async {
                    self.dumpGateStatus = !self.hardwareReady ? "Connect the rover to move the gate"
                        : open.map { $0 ? "Gate open" : "Gate closed" } ?? "Test ended"
                }
            } catch { DispatchQueue.main.async { self.dumpGateStatus = "Gate test refused: \(error)" } }
        }
    }
    func selectRoverProfile(_ name: String) {
        guard let json = RoverProfiles.json(named: name) else { return }
        UserDefaults.standard.set(name, forKey: RoverProfiles.key)
        roverProfile = name
        controlQueue.async {
            do { try self.controller?.setRoverProfile(profileJson: json) }
            catch { DispatchQueue.main.async { self.configurationStatus = "Rover profile rejected: \(error)" } }
        }
    }
    /// Shadow mode: Rust routes the same efforts so its frame can be compared with Swift's.
    func syncRustLayout(_ controller: MobileController) {
        let id = ObjectIdentifier(controller)
        if let sync = rustLayoutSync, sync.controller == id, sync.layout == bleLayout { return }
        rustLayoutSync = (id, bleLayout)
        // A pending ("{}") or invalid layout leaves Rust without one, as Swift's decode fails too.
        try? controller.setActuatorLayout(profilesJson: bleLayout == "{}" ? "" : bleLayout)
        for (key, value) in servoTargets { if let id = UInt8(key) { try? controller.setServoTarget(id: id, value: value) } }
    }
    func compareRustFrame(_ rustFrame: String?, with routed: [[String: Any]], now: TimeInterval) {
        let rust = rustFrame.flatMap { try? JSONSerialization.jsonObject(with: Data($0.utf8)) as? [[String: Any]] }
        func pairs(_ frame: [[String: Any]]) -> [(Int, Double)] {
            frame.compactMap { entry in (entry["id"] as? NSNumber).flatMap { id in (entry["value"] as? NSNumber).map { (id.intValue, $0.doubleValue) } } }
        }
        let expected = pairs(routed)
        let matches = rust.map { frame in
            let actual = pairs(frame)
            return actual.count == expected.count && zip(actual, expected).allSatisfy { $0.0 == $1.0 && abs($0.1 - $1.1) < 1e-3 }
        } ?? false
        guard !matches, now - lastShadowMismatchLog >= 1 else { return }
        lastShadowMismatchLog = now
        TerraLog.control.error("Rust actuator frame differs; swift=\(String(describing: expected), privacy: .public) rust=\(rustFrame ?? "nil", privacy: .public)")
    }
    var configurationAllowed: Bool {
        configurationBlockingReason == nil
    }
    var configurationBlockingReason: String? {
        let status = (try? JSONSerialization.jsonObject(with: Data(bluetooth.statusJSON.utf8))) as? [String: Any] ?? [:]
        return ActuatorConfigurationPolicy.blockingReason(ready: hardwareConfigurationReady, status: status)
    }
    @discardableResult private func sendConfiguration(_ operation: String, payload: [String: Any]) -> UInt32? {
        guard nextRequest < 0x7fffffff else { return nil }
        nextRequest += 1
        UserDefaults.standard.set(Int(nextRequest), forKey: "hardwareConfigurationRequestID")
        guard let data = try? JSONSerialization.data(withJSONObject: ["schema_version": 1, "request_id": nextRequest, "operation": operation, "payload": payload]) else { return nil }
        bluetooth.request(envelope: data, generation: bluetooth.connectionGeneration); return nextRequest
    }
    func resetHardwareFault() {
        guard configurationAllowed else { configurationStatus = configurationBlockingReason ?? "Configuration unavailable"; return }
        resetFaultRequest = sendConfiguration("reset_fault", payload: [:])
        configurationStatus = "Waiting for fault reset acknowledgement"
    }
    var hasStagedLayout: Bool { actuatorEditor.canCommit }
    var configurationBusy: Bool { editorRequest != nil || retryableEditorRequest != nil }
    func editActuator(_ actuator: ActuatorDraft) {
        guard !configurationBusy else { return }
        if presetUpload != nil { presetUpload?.updateCurrent(actuator) }
        actuatorEditor.change(actuator)
    }
    func discardLocalActuatorChanges() { guard !configurationBusy else { return }; presetUpload = nil; actuatorEditor.discardLocal() }
    func retryActuatorConfiguration() { guard !configurationBusy else { return }; bluetooth.refreshConfigurationIndex() }
    func retryFailedActuatorRequest() {
        guard editorRequest == nil, let request = retryableEditorRequest,
              request.generation == configurationGeneration else { return }
        retryableEditorRequest = nil; hasRetryableActuatorRequest = false
        editorRequest = request; actuatorPageLoading = request.operation == "read_actuator"
        configurationStatus = "Retrying \(request.operation.replacingOccurrences(of: "_", with: " "))"
        bluetooth.retryConfiguration(requestID: request.id, generation: request.generation)
    }
    func selectActuator(_ id: Int) {
        guard !configurationBusy, actuatorEditor.select(id) else { return }
        if actuatorEditor.selected != nil { return }
        actuatorPageLoading = true
        editorSend("read_actuator", payload: ["expected_revision": actuatorEditor.revision, "actuator_id": id], actuatorID: id)
    }
    private func editorSend(_ operation: String, payload: [String: Any], actuator: ActuatorDraft? = nil, actuatorID: Int? = nil) {
        guard !configurationBusy, let id = sendConfiguration(operation, payload: payload) else { return }
        editorRequest = (id, operation, configurationGeneration, actuator, actuatorID)
        configurationStatus = "Waiting for \(operation.replacingOccurrences(of: "_", with: " ")) acknowledgement"
    }
    private func beginEditor(replacement: Bool = false) {
        beginReplacement = replacement
        editorSend("begin_layout_edit", payload: ["expected_revision": actuatorEditor.revision, "mode": replacement ? "replace" : "existing"])
    }
    func saveSelectedActuator() {
        guard configurationAllowed, !configurationBusy, let actuator = actuatorEditor.selected else { return }
        if actuatorEditor.token == nil { beginEditor(); return }
        stageOne(actuator)
    }
    private func stageOne(_ actuator: ActuatorDraft) {
        guard let token = actuatorEditor.token,
              let data = try? JSONEncoder().encode(actuator),
              let object = try? JSONSerialization.jsonObject(with: data) else { return }
        actuatorEditor.change(actuator)
        editorSend("stage_actuator", payload: ["edit_token": token, "edit_version": actuatorEditor.version, "actuator": object], actuator: actuator)
    }
    func addActuator() {
        guard !configurationBusy, !actuatorEditor.dirty, actuatorEditor.entries.count < 16,
              let id = (0...255).first(where: { id in !actuatorEditor.entries.contains(where: { $0.id == id }) }) else { return }
        actuatorEditor.change(ActuatorDraft(id: id, name: "Actuator \(id)"))
    }
    func removeSelectedActuator() {
        guard configurationAllowed, !configurationBusy, let id = actuatorEditor.selectedID else { return }
        presetUpload = nil
        guard let token = actuatorEditor.token else { removeAfterBegin = id; beginEditor(); return }
        editorSend("remove_actuator", payload: ["edit_token": token, "edit_version": actuatorEditor.version, "actuator_id": id], actuatorID: id)
    }
    func uploadPreset(_ actuators: [ActuatorDraft]) {
        guard configurationAllowed, !configurationBusy, !actuatorEditor.dirty else { return }
        presetUpload = ActuatorPresetUpload(actuators: actuators); beginEditor(replacement: true)
    }
    private func presentNextPresetActuator() {
        guard let upload = presetUpload, let actuator = upload.current else { presetUpload = nil; configurationStatus = "Preset saved to draft · validate before applying"; return }
        if upload.needsPortSelection {
            actuatorEditor.change(actuator)
            configurationStatus = "Choose a physical port for \(actuator.name), then save to continue the preset"
        } else { stageOne(actuator) }
    }
    func validateActuatorDraft() {
        guard configurationAllowed, !configurationBusy, !actuatorEditor.dirty, let token = actuatorEditor.token else { return }
        editorSend("validate_layout_edit", payload: ["edit_token": token, "edit_version": actuatorEditor.version])
    }
    func discardActuatorDraft() {
        guard editorRequest == nil, let token = actuatorEditor.token else { return }
        retryableEditorRequest = nil; hasRetryableActuatorRequest = false; presetUpload = nil
        editorSend("discard_layout_edit", payload: ["edit_token": token])
    }
    func commitActuatorLayout() {
        guard configurationAllowed, !configurationBusy, actuatorEditor.canCommit, let token = actuatorEditor.token else { return }
        editorSend("commit_layout_edit", payload: ["edit_token": token, "edit_version": actuatorEditor.version, "base_revision": actuatorEditor.revision])
    }
    func receiveConfiguration(_ text: String) {
        guard let object = (try? JSONSerialization.jsonObject(with: Data(text.utf8))) as? [String: Any], let id = object["request_id"] as? UInt32 else { return }
        TerraLog.configuration.info("Reply request=\(id) result=\(String(describing: object["result"]), privacy: .public)")
        if id == resetFaultRequest {
            resetFaultRequest = nil
            configurationStatus = object["result"] as? String == "ok" ? "Fault reset acknowledged · output is neutral" : "Fault reset rejected: \(object["errors"] ?? [])"
            return
        }
        guard let request = editorRequest, request.id == id, request.generation == configurationGeneration else { return }
        editorRequest = nil; actuatorPageLoading = false
        guard object["result"] as? String == "ok" else {
            let errors = object["errors"] as? [[String: Any]] ?? []
            if errors.contains(where: { ["configuration_timeout", "configuration_transfer"].contains($0["code"] as? String ?? "") }) {
                retryableEditorRequest = request; hasRetryableActuatorRequest = true
                configurationStatus = "Acknowledgement unavailable · retry this operation to recover its exact result"
                return
            }
            retryableEditorRequest = nil; hasRetryableActuatorRequest = false
            if request.operation != "stage_actuator" { presetUpload = nil }
            removeAfterBegin = nil; actuatorEditor.validatedVersion = nil
            configurationStatus = "Rejected · local changes retained: \(object["errors"] ?? [])"; return
        }
        retryableEditorRequest = nil; hasRetryableActuatorRequest = false
        let payload = object["payload"] as? [String: Any] ?? [:]
        switch request.operation {
        case "read_actuator":
            if let revision = payload["revision"] as? UInt32, let object = payload["actuator"],
               let data = try? JSONSerialization.data(withJSONObject: object), let actuator = try? JSONDecoder().decode(ActuatorDraft.self, from: data) {
                _ = actuatorEditor.acceptPage(actuator, revision: revision, generation: request.generation)
            }
        case "begin_layout_edit":
            guard let token = payload["edit_token"] as? String, let base = payload["base_revision"] as? UInt32,
                  base == actuatorEditor.revision, let version = payload["edit_version"] as? UInt32 else { return }
            actuatorEditor.begin(token: token, base: base, version: version, replacement: beginReplacement)
            if let id = removeAfterBegin {
                removeAfterBegin = nil
                editorSend("remove_actuator", payload: ["edit_token": token, "edit_version": version, "actuator_id": id], actuatorID: id)
            } else if presetUpload != nil { presentNextPresetActuator(); return }
            else if let actuator = actuatorEditor.selected, actuatorEditor.dirty { stageOne(actuator) }
        case "stage_actuator", "remove_actuator", "validate_layout_edit":
            guard payload["edit_token"] as? String == actuatorEditor.token,
                  let version = payload["edit_version"] as? UInt32 else { return }
            if request.operation == "validate_layout_edit" {
                guard version == actuatorEditor.version else { return }; actuatorEditor.validatedVersion = version
            } else {
                guard version == actuatorEditor.version + 1 else { return }
                if let actuator = request.actuator { actuatorEditor.acknowledge(actuator, version: version) }
                if request.operation == "remove_actuator", let id = request.actuatorID { actuatorEditor.removed(id, version: version) }
                if request.operation == "stage_actuator", let actuator = request.actuator, presetUpload != nil {
                    guard presetUpload?.acknowledge(id: actuator.id) == true else { return }
                    presentNextPresetActuator(); return
                }
            }
        case "commit_layout_edit":
            guard let revision = payload["revision"] as? UInt32 else { return }
            actuatorEditor = ActuatorPaginationState()
            stopHardware(); bluetooth.refreshConfigurationIndex()
        case "discard_layout_edit":
            presetUpload = nil
            actuatorEditor = ActuatorPaginationState(); bluetooth.refreshConfigurationIndex()
        default: break
        }
        configurationStatus = "\(request.operation.replacingOccurrences(of: "_", with: " ")) acknowledged"

    }
}
