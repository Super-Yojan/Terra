import SwiftUI

/// Initial configuration on one page: connect the rover, say what the vehicle
/// is, tap each wheel to choose its port, set the size, apply. Applying writes
/// the actuator layout to the rover-pi over Bluetooth and the vehicle geometry
/// to terra-av.
struct SetupView: View {
    @ObservedObject var loop: RoverLoop
    @ObservedObject var motors: BluetoothLink
    @Environment(\.dismiss) private var dismiss
    @State private var setup: VehicleSetup
    @State private var editing: Output?
    @State private var testing = false
    @State private var view3D = RoverViewMode.ports

    init(loop: RoverLoop, motors: BluetoothLink) {
        self.loop = loop
        self.motors = motors
        _setup = State(initialValue: motors.vehicleSetup ?? VehicleSetup())
    }

    var body: some View {
        NavigationStack {
            ScrollView {
                VStack(alignment: .leading, spacing: 14) {
                    SectionTitle("Connection")
                    RoverCard(motors: motors)

                    SectionTitle("Base station")
                    BaseStationCard(loop: loop, motors: motors, setup: $setup)

                    SectionTitle("Vehicle")
                    VehicleCard(setup: $setup, ports: motors.ports)

                    SectionTitle("Rover")
                    Picker("View", selection: $view3D) {
                        ForEach(RoverViewMode.allCases) { Text($0.title).tag($0) }
                    }
                    .pickerStyle(.segmented)
                    RoverModelView(setup: setup, mode: view3D,
                                   onTapOutput: motors.ports.isEmpty ? nil : { editing = $0 })
                        .frame(height: 320)
                        .background(Theme.card, in: RoundedRectangle(cornerRadius: 12))
                        .clipShape(RoundedRectangle(cornerRadius: 12))
                    ModelControls(mode: view3D, setup: $setup, ports: motors.ports, occupied: motors.occupied)

                    SectionTitle("Driving")
                    SizeCard(setup: $setup)

                    ApplyStatus(state: motors.layoutApply, problems: setup.problems, connected: motors.connected != nil)
                }
                .padding(.horizontal, 20)
                .padding(.vertical, 12)
            }
            .background(Color.black)
            .navigationTitle("Set up rover")
            .navigationBarTitleDisplayMode(.inline)
            .toolbar {
                ToolbarItem(placement: .topBarTrailing) {
                    Button { dismiss() } label: { Image(systemName: "xmark") }
                        .accessibilityLabel("Close setup")
                }
            }
            .safeAreaInset(edge: .bottom) { bottomBar }
            .sheet(item: $editing) { output in
                PortPicker(output: output, setup: $setup, ports: motors.ports, occupied: motors.occupied)
            }
            .sheet(isPresented: $testing) { TestDriveView(loop: loop, motors: motors) }
            .toolbar {
                ToolbarItemGroup(placement: .keyboard) {
                    Spacer()
                    Button("Done") {
                        UIApplication.shared.sendAction(#selector(UIResponder.resignFirstResponder), to: nil, from: nil, for: nil)
                    }
                }
            }
        }
        .preferredColorScheme(.dark)
        .tint(Theme.accent)
        .onChange(of: motors.vehicleSetup) { _, loaded in
            if let loaded, loaded != setup { setup = loaded }
        }
        .onChange(of: setup) { _, value in
            var normalized = value
            normalized.normalize()
            if normalized != value { setup = normalized; return }
            if saved { motors.resetLayoutApply() } // edited after saving: apply again
        }
        .onAppear {
            if let loaded = motors.vehicleSetup { setup = loaded }
            if !working { motors.resetLayoutApply() }
        }
    }

    // MARK: Bottom bar

    @ViewBuilder private var bottomBar: some View {
        Group {
            if saved {
                HStack(spacing: 12) {
                    Button { testing = true } label: {
                        Text("Test drive").font(.headline).foregroundStyle(.white)
                            .frame(maxWidth: .infinity, minHeight: 52)
                            .background(Theme.card, in: RoundedRectangle(cornerRadius: 10))
                    }
                    Button { dismiss() } label: {
                        Text("Done").font(.headline).foregroundStyle(.white)
                            .frame(maxWidth: .infinity, minHeight: 52)
                            .background(Theme.accent, in: RoundedRectangle(cornerRadius: 10))
                    }
                }
            } else {
                Button(action: apply) {
                    Text(working ? "Applying…" : "Apply to rover").font(.headline).foregroundStyle(.white)
                        .frame(maxWidth: .infinity, minHeight: 52)
                        .background(canApply ? Theme.accent : Theme.line, in: RoundedRectangle(cornerRadius: 10))
                }
                .disabled(!canApply)
            }
        }
        .padding(.horizontal, 20)
        .padding(.top, 8)
        .padding(.bottom, 8)
        .background(Color.black)
    }

    private var working: Bool { if case .working = motors.layoutApply { true } else { false } }
    private var saved: Bool { if case .done = motors.layoutApply { true } else { false } }
    private var canApply: Bool { setup.problems.isEmpty && motors.connected != nil && !working }

    private func apply() {
        loop.setVehicle(setup)
        motors.applyLayout(setup.actuators(), vehicleSetup: setup)
    }
}

// MARK: Cards

private struct RoverCard: View {
    @ObservedObject var motors: BluetoothLink

    var body: some View {
        Card {
            HStack {
                Circle().fill(motors.connected != nil ? Color.green : Theme.warning).frame(width: 9, height: 9)
                Text(motors.status)
                Spacer()
                if motors.connected != nil { Button("Disconnect") { motors.disconnect() } }
                else { Button("Scan") { motors.scan() } }
            }
            if motors.connected == nil {
                if motors.rovers.isEmpty {
                    Text("Turn on the rover. Hold USR on the Pi to pair a new one.")
                        .font(.footnote).foregroundStyle(Theme.secondary)
                }
                ForEach(motors.rovers) { rover in
                    Divider().overlay(Theme.line)
                    Button { motors.connect(rover.id) } label: {
                        HStack {
                            Image(systemName: "antenna.radiowaves.left.and.right")
                            Text(rover.pairing ? "\(rover.name) · pair" : rover.name).foregroundStyle(.white)
                            Spacer()
                            Image(systemName: "chevron.right").foregroundStyle(Theme.secondary)
                        }
                    }
                }
            }
        }
    }
}

private struct BaseStationCard: View {
    @ObservedObject var loop: RoverLoop
    @ObservedObject var motors: BluetoothLink
    @Binding var setup: VehicleSetup

    @State private var draft: BaseStationConfig
    @State private var editingPassword = false
    @State private var password = ""
    @State private var hasPassword = RouterPassword.load() != nil
    @State private var savedBanner = false

    init(loop: RoverLoop, motors: BluetoothLink, setup: Binding<VehicleSetup>) {
        self.loop = loop
        self.motors = motors
        _setup = setup
        _draft = State(initialValue: loop.baseStation)
    }

    private var roverIdBinding: Binding<Int> {
        Binding(
            get: { motors.connected != nil ? setup.roverId : draft.roverId },
            set: { newValue in
                draft.roverId = newValue
                if motors.connected != nil {
                    setup.roverId = newValue
                }
            }
        )
    }

    private var hasUnsavedChanges: Bool {
        draft != loop.baseStation
    }

    var body: some View {
        Card {
            Toggle("Connect to base station", isOn: $draft.enabled)

            if draft.enabled {
                Divider().overlay(Theme.line)
                VStack(alignment: .leading, spacing: 4) {
                    Text("Endpoint").font(.footnote).foregroundStyle(Theme.secondary)
                    TextField("tcp/192.168.1.165:7447", text: $draft.endpoint)
                        .keyboardType(.URL)
                        .textInputAutocapitalization(.never)
                        .autocorrectionDisabled()
                        .font(.system(.body, design: .monospaced))
                }

                Divider().overlay(Theme.line)
                Stepper(value: roverIdBinding, in: 0...99) {
                    HStack {
                        Text("Rover ID")
                        Spacer()
                        Text("\(roverIdBinding.wrappedValue)")
                            .foregroundStyle(Theme.secondary)
                            .monospacedDigit()
                    }
                }
                Text(motors.connected != nil
                     ? "Saved on the Raspberry Pi when you tap Apply to rover."
                     : "Local fallback while no rover is connected.")
                    .font(.footnote).foregroundStyle(Theme.secondary)

                Divider().overlay(Theme.line)
                HStack {
                    Text("Prefix")
                    Spacer()
                    TextField("terra", text: $draft.prefix)
                        .multilineTextAlignment(.trailing)
                        .textInputAutocapitalization(.never)
                        .autocorrectionDisabled()
                        .foregroundStyle(Theme.secondary)
                }

                Divider().overlay(Theme.line)
                HStack {
                    Text("User")
                    Spacer()
                    TextField("Optional", text: $draft.user)
                        .multilineTextAlignment(.trailing)
                        .textInputAutocapitalization(.never)
                        .autocorrectionDisabled()
                        .foregroundStyle(Theme.secondary)
                }

                Divider().overlay(Theme.line)
                Button { password = ""; editingPassword = true } label: {
                    Row(title: "Router password", value: hasPassword ? "Saved" : "Not set")
                }
            }

            Divider().overlay(Theme.line)
            HStack {
                if savedBanner && !hasUnsavedChanges {
                    Text("Reconnected").font(.footnote).foregroundStyle(.green)
                } else if hasUnsavedChanges {
                    Text("Unsaved changes").font(.footnote).foregroundStyle(Theme.warning)
                } else {
                    Text("Keys: \(draft.prefix.isEmpty ? "terra" : draft.prefix)/\(roverIdBinding.wrappedValue)/…")
                        .font(.footnote).foregroundStyle(Theme.secondary)
                }
                Spacer()
                Button("Save & reconnect") {
                    loop.setBaseStation(draft)
                    if motors.connected != nil {
                        loop.setVehicle(setup)
                    }
                    savedBanner = true
                }
                .font(.subheadline.weight(.semibold))
                .disabled(!hasUnsavedChanges && motors.connected == nil)
            }
        }
        .onChange(of: loop.baseStation) { _, updated in
            if updated != draft { draft = updated }
        }
        .onChange(of: draft) { _, _ in
            savedBanner = false
        }
        .alert("Fleet router password", isPresented: $editingPassword) {
            SecureField("Password", text: $password)
            Button("Save") {
                loop.setRouterPassword(password)
                hasPassword = RouterPassword.load() != nil
            }
            Button("Cancel", role: .cancel) {}
        } message: {
            Text("Stored in the Keychain. Save empty to clear.")
        }
    }
}

private struct VehicleCard: View {
    @Binding var setup: VehicleSetup
    let ports: [RoverPort]

    var body: some View {
        Card {
            Picker("Steering", selection: $setup.steering) {
                ForEach(Steering.allCases) { Text($0.title).tag($0) }
            }
            .pickerStyle(.segmented)
            Text(setup.steering.detail).font(.footnote).foregroundStyle(Theme.secondary)

            if setup.steering.wheelCounts.count > 1 {
                Divider().overlay(Theme.line)
                HStack {
                    Text("Wheels")
                    Spacer()
                    Picker("Wheels", selection: $setup.wheelCount) {
                        ForEach(setup.steering.wheelCounts, id: \.self) { Text("\($0)").tag($0) }
                    }
                    .pickerStyle(.segmented)
                    .frame(width: 170)
                }
            }

            Divider().overlay(Theme.line)
            HStack {
                Text("Motors")
                Spacer()
                Picker("Motors", selection: $setup.driveKind) {
                    ForEach(DriveKind.allCases) { Text($0.title).tag($0) }
                }
                .pickerStyle(.menu)
            }
            Text(setup.driveKind.detail).font(.footnote).foregroundStyle(Theme.secondary)

            if !(setup.steering == .differential && setup.wheelCount == 2) {
                let fits = setup.perWheelFits(ports: ports)
                Divider().overlay(Theme.line)
                HStack {
                    Text("Outputs")
                    Spacer()
                    Picker("Outputs", selection: $setup.grouping) {
                        Text(setup.steering == .differential ? "Per side" : "Shared").tag(Grouping.shared)
                        Text(setup.steering == .differential ? "Per wheel" : "Per wheel").tag(Grouping.perWheel)
                    }
                    .pickerStyle(.segmented)
                    .frame(width: 170)
                    .onChange(of: fits) { _, fits in if !fits { setup.grouping = .shared } }
                }
                if !fits && !ports.isEmpty {
                    Text("Not enough \(setup.driveKind.title) ports for one per wheel.")
                        .font(.footnote).foregroundStyle(Theme.warning)
                }
            }

            if setup.steering == .ackermann {
                Divider().overlay(Theme.line)
                DisclosureGroup("Steering servo pulses") {
                    VStack(spacing: 10) {
                        PulseRow(title: "Full left", value: $setup.servoMinUs)
                        PulseRow(title: "Center", value: $setup.servoCenterUs)
                        PulseRow(title: "Full right", value: $setup.servoMaxUs)
                    }
                    .padding(.top, 8)
                }
                .foregroundStyle(.white)
            }
        }
    }
}

/// What goes under the 3D rover for each view.
private struct ModelControls: View {
    let mode: RoverViewMode
    @Binding var setup: VehicleSetup
    let ports: [RoverPort]
    let occupied: Set<String>

    var body: some View {
        switch mode {
        case .ports:
            HStack {
                Text(ports.isEmpty ? "Connect the rover to choose ports." : "Tap a wheel to choose its port. Drag to turn the rover.")
                    .font(.footnote).foregroundStyle(Theme.secondary)
                Spacer()
                Button("Auto-assign") { setup.autoAssign(ports: ports, occupied: occupied) }
                    .font(.subheadline)
                    .disabled(ports.isEmpty)
            }
        case .forward:
            Card {
                DimensionField(title: "Phone ahead of centre", detail: "From the rover's centre to the phone, along the length",
                               color: DimensionColor.forward, value: $setup.phoneForward, range: -2...2)
            }
        case .height:
            Card {
                DimensionField(title: "Phone height", detail: "From the ground to the middle of the phone",
                               color: DimensionColor.height, value: $setup.phoneHeight, range: 0.01...2)
            }
        case .angle:
            Card {
                VStack(alignment: .leading, spacing: 8) {
                    HStack(spacing: 12) {
                        RoundedRectangle(cornerRadius: 2).fill(DimensionColor.view).frame(width: 4, height: 34)
                        VStack(alignment: .leading, spacing: 2) {
                            Text("Turned")
                            Text("Turn until the camera on the model is where it is on your phone")
                                .font(.caption).foregroundStyle(Theme.secondary)
                        }
                    }
                    Picker("Turned", selection: $setup.phoneTurn) {
                        ForEach([0, 90, 180, 270], id: \.self) { Text("\($0)°").tag($0) }
                    }
                    .pickerStyle(.segmented)
                }
                Divider().overlay(Theme.line)
                DimensionField(title: "Tilt", detail: "Degrees the camera looks down (negative: up)",
                               color: DimensionColor.view, unit: "°", value: $setup.phoneTilt, range: -89...89)
                Divider().overlay(Theme.line)
                DimensionField(title: "Pan", detail: "Degrees the camera looks left (negative: right)",
                               color: DimensionColor.view, unit: "°", value: $setup.phonePan, range: -180...180)
            }
            let e = setup.mountEuler
            Text(String(format: "Sent to the autonomy stack as roll %.1f° · pitch %.1f° · yaw %.1f°", e.roll, e.pitch, e.yaw))
                .font(.footnote).foregroundStyle(Theme.secondary)
        case .size:
            Card {
                DimensionField(title: "Length", detail: "Front to back of the body",
                               color: DimensionColor.size, value: $setup.bodyLength, range: 0.05...3)
                Divider().overlay(Theme.line)
                DimensionField(title: "Width", detail: "Side to side of the body, without wheels",
                               color: DimensionColor.size, value: $setup.bodyWidth, range: 0.05...3)
            }
        }
    }
}

/// A labelled number field, colour-matched to its line in the 3D view.
private struct DimensionField: View {
    let title: String
    let detail: String
    let color: Color
    var unit = "m"
    @Binding var value: Double
    let range: ClosedRange<Double>

    var body: some View {
        HStack(spacing: 12) {
            RoundedRectangle(cornerRadius: 2).fill(color).frame(width: 4, height: 34)
            VStack(alignment: .leading, spacing: 2) {
                Text(title)
                Text(detail).font(.caption).foregroundStyle(Theme.secondary)
            }
            Spacer()
            TextField("0.00", value: Binding(get: { value }, set: { value = min(range.upperBound, max(range.lowerBound, $0)) }),
                      format: .number.precision(.fractionLength(0...3)))
                .keyboardType(.numbersAndPunctuation)
                .multilineTextAlignment(.trailing)
                .font(.body.monospacedDigit())
                .frame(width: 70)
                .padding(.vertical, 6).padding(.horizontal, 8)
                .background(Color(white: 0.18), in: RoundedRectangle(cornerRadius: 8))
            Text(unit).foregroundStyle(Theme.secondary).frame(width: 14, alignment: .leading)
        }
    }
}

private struct SizeCard: View {
    @Binding var setup: VehicleSetup

    var body: some View {
        Card {
            MeasureRow(title: "Track width", detail: "Left to right wheel centres", unit: "m",
                       value: $setup.trackWidth, range: 0.1...2.0, step: 0.01)
            if setup.steering == .ackermann {
                Divider().overlay(Theme.line)
                MeasureRow(title: "Wheelbase", detail: "Front to rear axle", unit: "m",
                           value: $setup.wheelbase, range: 0.1...2.0, step: 0.01)
            }
            Divider().overlay(Theme.line)
            MeasureRow(title: "Top wheel speed", detail: "Ground speed at full power", unit: "m/s",
                       value: $setup.maxWheelSpeed, range: 0.1...5.0, step: 0.05)
        }
        if setup.steering == .ackermann {
            Text("Autonomy only has a differential-drive model so far. An Ackermann rover can be driven by hand; the autonomy settings stay as they are.")
                .font(.footnote).foregroundStyle(Theme.warning)
        }
    }
}

private struct ApplyStatus: View {
    let state: BluetoothLink.LayoutApply
    let problems: [String]
    let connected: Bool

    var body: some View {
        VStack(alignment: .leading, spacing: 6) {
            switch state {
            case .idle:
                if !connected { Text("Connect to the rover to apply.").foregroundStyle(Theme.secondary) }
                ForEach(problems, id: \.self) { Text($0).foregroundStyle(Theme.warning) }
            case .working(let what):
                HStack { ProgressView(); Text(what).foregroundStyle(Theme.secondary) }
            case .failed(let why):
                Text("Not saved: \(why)").foregroundStyle(.red)
            case .done(let revision):
                Text("Saved on the rover (layout revision \(revision)).").foregroundStyle(.green)
            }
        }
        .font(.footnote)
        .padding(.top, 4)
    }
}

/// Port list for one output, opened by tapping a wheel.
private struct PortPicker: View {
    let output: Output
    @Binding var setup: VehicleSetup
    let ports: [RoverPort]
    let occupied: Set<String>
    @Environment(\.dismiss) private var dismiss

    var body: some View {
        let current = setup.assignments[output.id]?.port
        let available = setup.availablePorts(for: output, ports: ports, occupied: occupied)
        NavigationStack {
            List {
                Section {
                    ForEach(available) { port in
                        Button {
                            setup.assignments[output.id, default: PortAssignment()].port = port.name
                            dismiss()
                        } label: {
                            HStack {
                                Text(port.name).font(.body.monospaced()).foregroundStyle(.white)
                                if port.resources != [port.name] {
                                    Text("uses \(port.resources.joined(separator: ", "))")
                                        .font(.caption).foregroundStyle(Theme.secondary)
                                }
                                Spacer()
                                if port.name == current { Image(systemName: "checkmark") }
                            }
                        }
                    }
                    if current != nil {
                        Button("Clear", role: .destructive) {
                            setup.assignments[output.id]?.port = nil
                            dismiss()
                        }
                    }
                } header: {
                    Text(output.kind == "positional_servo" ? "Servo ports" : "\(setup.driveKind.title) ports")
                } footer: {
                    Text("Ports already used, or sharing pins with a used port, aren't listed.")
                }
                if output.kind != DriveKind.unidirectionalESC.rawValue {
                    Section {
                        Toggle(output.role == .steering ? "Reverse steering" : "Spins backwards (invert)",
                               isOn: Binding(get: { setup.assignments[output.id]?.inverted ?? false },
                                             set: { setup.assignments[output.id, default: PortAssignment()].inverted = $0 }))
                    }
                }
            }
            .navigationTitle(output.name)
            .navigationBarTitleDisplayMode(.inline)
            .toolbar { ToolbarItem(placement: .topBarTrailing) { Button("Done") { dismiss() } } }
        }
        .presentationDetents([.medium, .large])
        .preferredColorScheme(.dark)
        .tint(Theme.accent)
    }
}

// MARK: Building blocks (Tesla-style: dark cards, plain rows)

private struct Card<Content: View>: View {
    @ViewBuilder var content: Content
    var body: some View {
        VStack(alignment: .leading, spacing: 12) { content }
            .padding(16)
            .frame(maxWidth: .infinity, alignment: .leading)
            .background(Theme.card, in: RoundedRectangle(cornerRadius: 12))
    }
}

private struct SectionTitle: View {
    let text: String
    init(_ text: String) { self.text = text }
    var body: some View {
        Text(text.uppercased()).font(.caption).foregroundStyle(Theme.secondary).padding(.top, 6)
    }
}

private struct Row: View {
    let title: String
    let value: String
    var body: some View {
        HStack { Text(title).foregroundStyle(.white); Spacer(); Text(value).foregroundStyle(Theme.secondary) }
    }
}

private struct MeasureRow: View {
    let title: String
    let detail: String
    let unit: String
    @Binding var value: Double
    let range: ClosedRange<Double>
    let step: Double
    var body: some View {
        Stepper(value: $value, in: range, step: step) {
            VStack(alignment: .leading, spacing: 2) {
                HStack { Text(title); Spacer(); Text(String(format: "%.2f %@", value, unit)).monospacedDigit() }
                Text(detail).font(.caption).foregroundStyle(Theme.secondary)
            }
        }
    }
}

private struct PulseRow: View {
    let title: String
    @Binding var value: Int
    var body: some View {
        Stepper(value: $value, in: 500...2500, step: 10) {
            HStack { Text(title); Spacer(); Text("\(value) µs").monospacedDigit().foregroundStyle(Theme.secondary) }
        }
    }
}
