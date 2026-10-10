import SwiftUI

struct ActuatorLayoutView: View {
    @ObservedObject var brain: PhoneController
    @State private var draft = ActuatorLayoutDraft.rover1()
    @State private var dirty = false
    private var caps: ActuatorCapabilities? { try? JSONDecoder().decode(ActuatorCapabilities.self, from: Data(brain.capabilitiesJSON.utf8)) }
    private var active: ActuatorLayoutDraft? { try? JSONDecoder().decode(ActuatorLayoutDraft.self, from: Data(brain.committedLayoutJSON.utf8)) }
    var body: some View {
        Form {
            Section("Rover configuration") {
                Text(brain.roverDisplayName).font(.headline)
                Text(brain.hardwareConfigurationReady ? "Configuration available" : "Connect the rover from Home to read its configuration.")
                    .font(.subheadline).foregroundStyle(.secondary)
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
            if let active {
                Section("Active layout · revision \(active.revision)") {
                    ForEach(active.actuators.filter { $0.kind == "positional_servo" }, id: \.id) { actuator in
                        if let id = UInt8(exactly: actuator.id), actuator.limits.min < actuator.limits.max {
                            LabeledContent(actuator.name, value: String(format: "Position %.2f", brain.servoPositions[id] ?? min(actuator.limits.max, max(actuator.limits.min, actuator.safe.value ?? 0))))
                            Slider(value: Binding(get: { brain.servoPositions[id] ?? min(actuator.limits.max, max(actuator.limits.min, actuator.safe.value ?? 0)) }, set: { brain.setServoTarget(id: id, position: $0) }), in: actuator.limits.min...actuator.limits.max).disabled(!brain.hardwareArmed)
                        }
                    }
                    Text("Servo position is normalized −1 to +1 and independent of propulsion effort.").font(.footnote)
                }
            }
            Section("Draft configuration") {
                Button("Rover1 · two ESCs on P0 and P1") { draft = .rover1(revision: brain.activeLayoutRevision) }
                if let caps { Text("\(caps.board) · \(caps.library) \(caps.library_version)") }
                Text("Active revision \(brain.activeLayoutRevision); draft base revision \(draft.revision)")
                Button("Load active layout") { if let active { draft = active; dirty = false } }.disabled(active == nil)
                Button("Use current revision for draft") { draft.revision = brain.activeLayoutRevision }.disabled(!brain.configurationAllowed)
                Menu("Editable presets") {
                    Button("Terra Mini · four DC motors") { preset("mini") }
                    Button("Two bidirectional ESCs") { preset("esc") }
                    Button("Motor and positional servo") { preset("mixed") }
                }
                Button("Add actuator") { addActuator() }.disabled(draft.actuators.count >= 16 || caps == nil)
                Text("Up to 16 actuators with unique IDs from 0 to 255. Presets require choosing each physical port from the rover capabilities.").font(.footnote)
            }.disabled(brain.hardwareArmed || brain.hardwareArming)
            ForEach(draft.actuators.indices, id: \.self) { index in
                Section("Actuator \(draft.actuators[index].name)") {
                    ActuatorEntryView(actuator: $draft.actuators[index], capabilities: caps)
                    Button("Remove actuator", role: .destructive) { draft.actuators.remove(at: index) }
                }.disabled(brain.hardwareArmed || brain.hardwareArming)
            }
            Section("Apply configuration") {
                Text(brain.configurationStatus).textSelection(.enabled)
                if let reason = brain.configurationBlockingReason { Text(reason).font(.footnote).foregroundStyle(.orange) }
                Button("Validate and stage draft") {
                    if let data = try? JSONEncoder().encode(draft) { brain.stageActuatorLayout(json: String(decoding: data, as: UTF8.self)) }
                }.disabled(!brain.configurationAllowed)
                Button("Commit acknowledged stage") { brain.commitActuatorLayout() }.disabled(!brain.hasStagedLayout || !brain.configurationAllowed)
                Text("Stage validates without applying. Commit applies the exact acknowledged stage. Rejections retain this draft; active revision changes only after rover acknowledgement and refresh.").font(.footnote)
            }
        }
        .navigationTitle("Actuator layouts")
        .onAppear { if !dirty { draft = active ?? .rover1(revision: brain.activeLayoutRevision) } }
        .onChange(of: brain.activeLayoutRevision) { _, revision in if !dirty { draft.revision = revision } }
        .onChange(of: brain.acknowledgedCommitRevision) { _, revision in
            if let revision { draft.revision = revision }
        }
        .onChange(of: brain.committedLayoutJSON) { _, _ in if !dirty, let active { draft = active } }
        .onChange(of: draft) { _, _ in dirty = true; brain.invalidateStagedLayout() }
    }
    private func addActuator() {
        let used = Set(draft.actuators.map(\.id))
        guard let id = (0...255).first(where: { !used.contains($0) }), let kind = caps?.supported_kinds.first else { return }
        var a = ActuatorDraft(id: id, name: "Actuator \(id)"); a.changeKind(kind); draft.actuators.append(a)
    }
    private func preset(_ kind: String) {
        draft = ActuatorLayoutDraft(revision: brain.activeLayoutRevision)
        let count = kind == "mini" ? 4 : 2
        for id in 0..<count {
            var a = ActuatorDraft(id: id, name: "Actuator \(id)")
            a.changeKind(kind == "esc" ? "bidirectional_esc" : kind == "mixed" && id == 1 ? "positional_servo" : "dc_motor")
            if a.kind != "positional_servo" { a.route = ActuatorRoute(type: id < count / 2 ? "left_effort" : "right_effort") }
            draft.actuators.append(a)
        }
    }
}
