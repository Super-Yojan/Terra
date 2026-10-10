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
                NavigationLink("Actuator configuration") { ActuatorLayoutView(brain: brain) }
                Text(brain.configurationStatus).font(.footnote).foregroundStyle(.secondary)
            }
        }.navigationTitle("Rover setup")
    }
}
