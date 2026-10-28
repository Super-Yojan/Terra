import SwiftUI

struct RoverSetupView: View {
    @ObservedObject var brain: PhoneController
    @AppStorage("autoConnectHardware") private var automatic = true
    var body: some View {
        Form {
            Section {
                Toggle("Connect automatically", isOn: $automatic)
                    .onChange(of: automatic) { _, enabled in
                        if enabled { brain.autoConnectHardware() } else { brain.cancelAutoConnection() }
                    }
                Text("Terra remembers the last confirmed rover and reconnects when the app opens.")
                    .font(.footnote).foregroundStyle(.secondary)
            }
            Section {
                Label(brain.phoneTrackingActive ? "Phone tracking on" : "Phone tracking off", systemImage: "viewfinder")
                Text(brain.phoneTrackingStatus)
                if brain.dashboardConnected && !brain.phoneTrackingActive {
                    Button("Retry phone tracking") { brain.retryPhoneTracking() }
                }
                Text("ARGOS manages tracking automatically in every autonomy mode. Camera access and phone sensors are required. Feedback control also requires a compatible actuator layout.")
                    .font(.footnote).foregroundStyle(.secondary)
            }
            Section {
                Picker("Rover", selection: Binding(get: { brain.roverProfile }, set: { brain.selectRoverProfile($0) })) {
                    ForEach(RoverProfiles.names, id: \.self) { Text(RoverProfiles.title($0)).tag($0) }
                }
                Text("Sets the body size the planner avoids obstacles with, the speed limits, and the motor deadband the speed loop drives past.")
                    .font(.footnote).foregroundStyle(.secondary)
            }
            Section("Dump gate") {
                DumpGateEditor(brain: brain)
            }
            Section("GPS position") {
                PhoneLocalizationStatusView(report: brain.localizationReport, trackingActive: brain.phoneTrackingActive)
            }
            Section {
                NavigationLink("Actuator configuration") { ActuatorLayoutView(brain: brain) }
                Text(brain.configurationStatus).font(.footnote).foregroundStyle(.secondary)
            }
        }.navigationTitle("Rover setup")
    }
}

/// The same measured localization diagnostics appear on Home and in Rover setup.
struct PhoneLocalizationStatusView: View {
    let report: PhoneLocalizationReport?
    let trackingActive: Bool
    private var title: String {
        guard trackingActive, let report else { return "Waiting for GPS" }
        return report.mode == "gps" ? "GPS position available" : "GPS needs attention"
    }
    var body: some View {
        VStack(alignment: .leading, spacing: 8) {
            Label(title, systemImage: "location")
                .font(.subheadline.weight(.semibold))
            if trackingActive, let report {
                if report.issues.isEmpty {
                    Text(report.reason).font(.footnote).foregroundStyle(.secondary)
                } else {
                    ForEach(Array(report.issues.enumerated()), id: \.offset) { _, issue in
                        Text(issue).font(.footnote).foregroundStyle(.secondary)
                            .fixedSize(horizontal: false, vertical: true)
                    }
                }
            } else {
                Text(trackingActive ? "Waiting for a GPS fix" : "Location updates start with the phone runtime")
                    .font(.footnote).foregroundStyle(.secondary)
            }
        }
    }
}

/// Choose the gate servo and set its open and closed positions. Rust decides
/// when the gate opens; these are only where the servo goes.
struct DumpGateEditor: View {
    @ObservedObject var brain: PhoneController
    @State private var draft: DumpGateSetting?
    private var servos: [ActuatorDriveProfile] {
        ((try? ActuatorDriveProfileSet.decode(json: brain.committedLayoutJSON))?.actuators ?? [])
            .filter { $0.kind == "positional_servo" && $0.limits.min < $0.limits.max }
    }
    var body: some View {
        let gate = draft ?? brain.dumpGate
        let servo = servos.first { $0.id == gate?.servoId }
        Picker("Gate servo", selection: Binding(
            get: { gate?.servoId ?? -1 },
            set: { id in
                guard id >= 0 else { draft = nil; brain.setDumpGate(nil); return }
                let s = servos.first { $0.id == id }
                draft = DumpGateSetting(servoId: id, openPosition: gate?.openPosition ?? s?.limits.max ?? 1,
                                        closedPosition: gate?.closedPosition ?? s?.limits.min ?? -1)
            })) {
            Text("None").tag(-1)
            ForEach(servos, id: \.id) { Text("Servo \($0.id)").tag($0.id) }
        }
        if let gate, let servo {
            position("Open", value: gate.openPosition, servo: servo) { draft = DumpGateSetting(servoId: gate.servoId, openPosition: $0, closedPosition: gate.closedPosition) }
            Button("Try open") { save(); brain.testDumpGate(open: true) }.disabled(!brain.hardwareReady)
            position("Closed", value: gate.closedPosition, servo: servo) { draft = DumpGateSetting(servoId: gate.servoId, openPosition: gate.openPosition, closedPosition: $0) }
            Button("Try closed") { save(); brain.testDumpGate(open: false) }.disabled(!brain.hardwareReady)
            Button("Save gate") { save(); brain.testDumpGate(open: nil) }.disabled(draft == nil || draft == brain.dumpGate)
        }
        Text(servos.isEmpty ? "Add a positional servo in Actuator configuration first."
             : "The dock opens the gate once the outlet is over a bucket, then closes it. Try each position on the rover before a run.")
            .font(.footnote).foregroundStyle(.secondary)
        if !brain.dumpGateStatus.isEmpty { Text(brain.dumpGateStatus).font(.footnote) }
    }
    private func position(_ title: String, value: Double, servo: ActuatorDriveProfile, set: @escaping (Double) -> Void) -> some View {
        VStack(alignment: .leading) {
            LabeledContent(title, value: value.formatted(.number.precision(.fractionLength(2))))
            Slider(value: Binding(get: { value }, set: { set(($0 * 100).rounded() / 100) }), in: servo.limits.min...servo.limits.max)
        }
    }
    private func save() {
        guard let draft, draft.openPosition != draft.closedPosition else { return }
        brain.setDumpGate(draft)
        self.draft = nil
    }
}
