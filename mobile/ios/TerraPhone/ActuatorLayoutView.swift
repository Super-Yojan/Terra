import SwiftUI

struct ActuatorLayoutView: View {
    @ObservedObject var brain: PhoneController
    @State private var draft = ActuatorLayoutDraft.rover1()
    @State private var dirty = false
    @State private var feedback = false
    @State private var selected: String = ""
    @AppStorage("autoConnectHardware") private var autoConnectHardware = true
    private var caps: ActuatorCapabilities? { try? JSONDecoder().decode(ActuatorCapabilities.self, from: Data(brain.capabilitiesJSON.utf8)) }
    private var active: ActuatorLayoutDraft? { try? JSONDecoder().decode(ActuatorLayoutDraft.self, from: Data(brain.committedLayoutJSON.utf8)) }
    var body: some View {
        Form {
            Section("Bluetooth rover") {
                Toggle("Connect Automatically", isOn: $autoConnectHardware)
                    .onChange(of: autoConnectHardware) { _, enabled in
                        if enabled { brain.autoConnectHardware() } else { brain.cancelAutoConnection() }
                    }
                Text("Reconnect to your last rover when the app opens. Motors remain disarmed until you arm them.").font(.footnote).foregroundStyle(.secondary)
                Text("First setup: hold the rover’s USR button for 3 seconds until its LED blinks. Find your terra- rover below and accept pairing on your phone.").font(.footnote)
                Button("Find rovers") { brain.scanBluetooth() }
                Picker("Rover", selection: $selected) {
                    Text("Select discovered rover").tag("")
                    ForEach(brain.discoveredRovers) { rover in Text("\(rover.name) · \(rover.identifier)").tag(rover.identifier) }
                }
                Toggle("Phone feedback", isOn: $feedback)
                Button("Connect selected rover") { if let id = UUID(uuidString: selected) { brain.startBluetooth(identifier: id, feedback: feedback) } }.disabled(UUID(uuidString: selected) == nil)
                Text(brain.hardwareStatus)
                Text(brain.hardwareReady ? "Layout and capabilities synchronized" : brain.hardwareConfigurationReady ? "Configuration available · motion requires a valid layout and cleared fault" : "Waiting for bonded owner access and capabilities")
                Text("Manual mode commands normalized effort. Feedback requires a compatible left/right layout and healthy phone tracking.").font(.footnote)
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
                Button("Arm hardware") { brain.armHardware() }.disabled(!brain.hardwareReady || brain.hardwareArmed || brain.hardwareArming)
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
        .onChange(of: selected) { _, _ in if brain.hardwareActive { brain.stop() } }
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

private struct ActuatorEntryView: View {
    @Binding var actuator: ActuatorDraft
    let capabilities: ActuatorCapabilities?
    private var ports: [String] { capabilities?.ports.keys.filter { capabilities?.ports[$0]?.kinds.contains(actuator.kind) == true }.sorted() ?? [] }
    var body: some View {
        Stepper("ID: \(actuator.id)", value: $actuator.id, in: 0...255)
        TextField("Name", text: $actuator.name)
        Picker("Output kind", selection: Binding(get: { actuator.kind }, set: { actuator.changeKind($0); actuator.port = "" })) {
            ForEach(capabilities?.supported_kinds ?? [], id: \.self) { Text($0.replacingOccurrences(of: "_", with: " ")).tag($0) }
        }
        Picker("Physical port", selection: $actuator.port) {
            Text("Choose capability port").tag("")
            ForEach(ports, id: \.self) { Text($0).tag($0) }
        }
        if let port = capabilities?.ports[actuator.port] { Text("\(port.frequency_hz) Hz · resources \(port.resources.joined(separator: ", "))").font(.caption) }
        Toggle("Invert output", isOn: $actuator.inverted).disabled(actuator.kind == "unidirectional_esc")
        if actuator.kind == "unidirectional_esc" { Text("Unidirectional ESCs cannot invert output; zero always means stop.").font(.footnote) }
        number("Minimum normalized command", value: $actuator.limits.min)
        number("Maximum normalized command", value: $actuator.limits.max)
        if actuator.kind == "positional_servo" {
            pulse("Minimum pulse µs", value: $actuator.calibration.min_us)
            pulse("Center pulse µs", value: $actuator.calibration.center_us)
            pulse("Maximum pulse µs", value: $actuator.calibration.max_us)
            Picker("Safe output", selection: Binding(get: { actuator.safe.type }, set: { actuator.safe = ActuatorSafe(type: $0, value: $0 == "position" ? 0 : nil) })) { Text("Configured position").tag("position"); Text("Disable PWM").tag("disabled") }
            if actuator.safe.type == "position" { optionalNumber("Safe position", value: $actuator.safe.value) }
            Text("Disabled PWM releases the servo while disarmed. Its control starts at center bounded by command limits on rearm.").font(.footnote)
        } else {
            Picker("Route", selection: Binding(get: { actuator.route.type }, set: { actuator.route = $0 == "manual" ? ActuatorRoute(type: $0, forward_coefficient: 1, turn_coefficient: 0) : ActuatorRoute(type: $0) })) { Text("Left effort").tag("left_effort"); Text("Right effort").tag("right_effort"); Text("Manual coefficients").tag("manual") }
            if actuator.route.type == "manual" { optionalNumber("Forward coefficient", value: $actuator.route.forward_coefficient); optionalNumber("Turn coefficient", value: $actuator.route.turn_coefficient) }
            if actuator.kind == "dc_motor" { optionalNumber("Maximum power fraction", value: $actuator.calibration.max_power_fraction) }
            if actuator.kind == "bidirectional_esc" { pulse("Reverse pulse µs", value: $actuator.calibration.reverse_us); pulse("Neutral pulse µs", value: $actuator.calibration.neutral_us); pulse("Forward pulse µs", value: $actuator.calibration.forward_us) }
            if actuator.kind == "unidirectional_esc" { pulse("Stop pulse µs", value: $actuator.calibration.stop_us); pulse("Full power pulse µs", value: $actuator.calibration.full_power_us) }
            if actuator.kind.contains("esc") { pulse("Arming duration ms", value: $actuator.calibration.arming_duration_ms) }
            Text("Safe propulsion is zero effort; ESCs maintain their neutral or stop pulse.").font(.footnote)
        }
    }
    private func number(_ title: String, value: Binding<Double>) -> some View { HStack { Text(title); TextField(title, value: value, format: .number).keyboardType(.numbersAndPunctuation).multilineTextAlignment(.trailing) } }
    private func optionalNumber(_ title: String, value: Binding<Double?>) -> some View { number(title, value: Binding(get: { value.wrappedValue ?? 0 }, set: { value.wrappedValue = $0 })) }
    private func pulse(_ title: String, value: Binding<Int?>) -> some View { HStack { Text(title); TextField(title, value: Binding(get: { value.wrappedValue ?? 0 }, set: { value.wrappedValue = $0 }), format: .number).keyboardType(.numberPad).multilineTextAlignment(.trailing) } }
}
