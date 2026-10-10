import SwiftUI

struct TerraSettingsView: View {
    @ObservedObject var brain: PhoneController
    var body: some View {
        List {
            Section("Setup") {
                NavigationLink { FleetConnectionSettings(brain: brain) } label: { Label("Fleet connection", systemImage: "network") }
                NavigationLink { RoverSetupView(brain: brain) } label: { Label("Rover setup", systemImage: "sensor.tag.radiowaves.forward") }
            }
            Section("Tools") {
                NavigationLink { ControllerDetailView(brain: brain, destination: .telemetry) } label: { Label("Diagnostics", systemImage: "waveform.path") }
                NavigationLink { TerraDebugTools(brain: brain) } label: { Label("Debug tools", systemImage: "wrench.and.screwdriver") }
            }
        }.navigationTitle("Settings")
    }
}
