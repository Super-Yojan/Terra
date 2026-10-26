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
