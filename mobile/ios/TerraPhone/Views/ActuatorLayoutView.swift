import SwiftUI

struct ActuatorLayoutView: View {
    @ObservedObject var brain: PhoneController
    @State private var pendingSelection: Int?
    @State private var showUnsent = false
    private var caps: ActuatorCapabilities? { try? JSONDecoder().decode(ActuatorCapabilities.self, from: Data(brain.capabilitiesJSON.utf8)) }
    var body: some View {
        Form {
            Section("Rover configuration") {
                Text(brain.roverDisplayName).font(.headline)
                Text(brain.hardwareConfigurationReady ? "Configuration available" : "Connect the rover from Home to read its configuration.")
                    .font(.subheadline).foregroundStyle(.secondary)
            }
            if !brain.hardwareConfigurationReady {
                Section("Configuration connection") {
                    Button("Retry configuration synchronization") { brain.retryActuatorConfiguration() }.disabled(brain.configurationBusy)
                }
            }
            if brain.hardwareBenchMode {
                Section("Bench Mode") {
                    Text("Software permission only; no physical power cutoff. Starts disabled on every connection.").font(.footnote)
                    Button(brain.hardwareBenchEnabled ? "Disable bench control" : "Enable bench control") { brain.setBenchEnabled(!brain.hardwareBenchEnabled) }
                        .disabled(!brain.hardwareReady || brain.hardwareArmed || brain.hardwareArming)
                }
            }
            Section("Hardware safety") {
                LabeledContent("Output", value: brain.hardwareArmed ? "Armed" : brain.hardwareArming ? "Arming at safe output" : "Disarmed")
                Button("Disarm", role: .destructive) { brain.disarmHardware() }
                Button("Emergency stop", role: .destructive) { brain.emergencyStop() }
                if let fault = brain.hardwareFault { Text("Fault: \(fault)").textSelection(.enabled) }
                Button("Reset fault") { brain.resetHardwareFault() }.disabled(!brain.configurationAllowed || brain.hardwareFault == nil)
                Button("Reset emergency stop") { brain.emergencyStop(reset: true) }
                Text("Reset does not arm. Apply configuration while disarmed. The battery switch is the external power cutoff for rover1; no switch signal to the Pi is required.").font(.footnote)
            }
            if let profiles = try? ActuatorDriveProfileSet.decode(json: brain.committedLayoutJSON) {
                Section("Active servo positions") {
                    ForEach(profiles.actuators.filter { $0.kind == "positional_servo" }, id: \.id) { actuator in
                        if let id = UInt8(exactly: actuator.id), actuator.limits.min < actuator.limits.max {
                            Slider(value: Binding(get: { brain.servoPositions[id] ?? actuator.safe.value ?? 0 }, set: { brain.setServoTarget(id: id, position: $0) }), in: actuator.limits.min...actuator.limits.max).disabled(!brain.hardwareArmed)
                        }
                    }
                }
            }
            Section("Actuators · revision \(brain.actuatorEditor.revision)") {
                ForEach(brain.actuatorEditor.entries) { entry in
                    Button("\(entry.name) · \(entry.kind)") {
                        if brain.actuatorEditor.dirty { pendingSelection = entry.id; showUnsent = true }
                        else { brain.selectActuator(entry.id) }
                    }.disabled(brain.configurationBusy)
                }
                Button("Add actuator") { brain.addActuator() }.disabled(brain.configurationBusy || brain.actuatorEditor.dirty || brain.actuatorEditor.entries.count >= 16)
                Menu("Replacement presets") {
                    Button("Rover1 · two ESCs on P0 and P1") { brain.uploadPreset(ActuatorLayoutDraft.rover1().actuators) }
                    Button("Terra Mini · four DC motors") { preset("mini") }
                    Button("Two bidirectional ESCs") { preset("esc") }
                    Button("Motor and positional servo") { preset("mixed") }
                }.disabled(brain.configurationBusy || brain.actuatorEditor.dirty || !brain.configurationAllowed)
                Text("Presets upload one actuator at a time. Choose physical ports before validation.").font(.footnote)
            }.disabled(brain.hardwareArmed || brain.hardwareArming)
            if brain.actuatorPageLoading { ProgressView("Loading selected actuator") }
            if let actuator = brain.actuatorEditor.selected {
                Section("Selected actuator") {
                    ActuatorEntryView(actuator: Binding(get: { brain.actuatorEditor.selected ?? actuator }, set: { brain.editActuator($0) }), capabilities: caps)
                    Button("Save actuator to draft") { brain.saveSelectedActuator() }.disabled(!brain.configurationAllowed || brain.configurationBusy)
                    Button("Discard unsent changes") { brain.discardLocalActuatorChanges() }.disabled(!brain.actuatorEditor.dirty)
                    Button("Remove actuator", role: .destructive) { brain.removeSelectedActuator() }.disabled(!brain.configurationAllowed || brain.configurationBusy)
                }.disabled(brain.hardwareArmed || brain.hardwareArming || brain.configurationBusy)
            } else if let id = brain.actuatorEditor.selectedID, !brain.actuatorPageLoading {
                Button("Retry loading actuator") { brain.selectActuator(id) }
            }
            Section("Apply configuration") {
                Text(brain.configurationStatus).textSelection(.enabled)
                if brain.hasRetryableActuatorRequest {
                    Button("Retry interrupted operation") { brain.retryFailedActuatorRequest() }
                }
                if let reason = brain.configurationBlockingReason { Text(reason).font(.footnote).foregroundStyle(.orange) }
                Button("Validate draft") { brain.validateActuatorDraft() }.disabled(!brain.configurationAllowed || brain.configurationBusy || brain.actuatorEditor.dirty || brain.actuatorEditor.token == nil)
                Button("Discard rover draft", role: .destructive) { brain.discardActuatorDraft() }.disabled((brain.configurationBusy && !brain.hasRetryableActuatorRequest) || brain.actuatorEditor.token == nil)
                Button("Commit acknowledged stage") { brain.commitActuatorLayout() }.disabled(!brain.hasStagedLayout || !brain.configurationAllowed || brain.configurationBusy)
                Text("Save updates one actuator in the rover draft. Validate checks all port assignments. Commit applies the validated version and remains disarmed.").font(.footnote)
            }
        }
        .navigationTitle("Actuator layouts")
        .onChange(of: brain.actuatorEditor.dirty) { _, dirty in
            if !dirty, let id = pendingSelection, !brain.configurationBusy {
                pendingSelection = nil; brain.selectActuator(id)
            }
        }
        .confirmationDialog("Unsent actuator changes", isPresented: $showUnsent) {
            Button("Save to draft") { brain.saveSelectedActuator() }
            Button("Discard and switch", role: .destructive) {
                brain.discardLocalActuatorChanges()
                if let id = pendingSelection { brain.selectActuator(id) }; pendingSelection = nil
            }
            Button("Keep editing", role: .cancel) { pendingSelection = nil }
        }
    }
    private func preset(_ kind: String) {
        var draft = ActuatorLayoutDraft(revision: brain.activeLayoutRevision)
        let count = kind == "mini" ? 4 : 2
        for id in 0..<count {
            var a = ActuatorDraft(id: id, name: "Actuator \(id)")
            a.changeKind(kind == "esc" ? "bidirectional_esc" : kind == "mixed" && id == 1 ? "positional_servo" : "dc_motor")
            if a.kind != "positional_servo" { a.route = ActuatorRoute(type: id < count / 2 ? "left_effort" : "right_effort") }
            draft.actuators.append(a)
        }
        brain.uploadPreset(draft.actuators)
    }
}
