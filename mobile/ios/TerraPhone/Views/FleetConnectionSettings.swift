import SwiftUI

struct FleetConnectionSettings: View {
    @ObservedObject var brain: PhoneController
    @Environment(\.dismiss) private var dismiss
    @State private var endpoint = UserDefaults.standard.string(forKey: "dashboardRouterEndpoint") ?? ""
    @State private var prefix = UserDefaults.standard.string(forKey: "dashboardTopicPrefix") ?? "terra/phone"
    @State private var roverID = UserDefaults.standard.string(forKey: "dashboardRoverID") ?? "0"
    @State private var error = ""
    var body: some View {
        Form {
            Section {
                VStack(alignment: .leading, spacing: 6) {
                    Text("Router address").font(.caption).foregroundStyle(.secondary)
                    TextField("tcp/<router address>:7448", text: $endpoint)
                        .textInputAutocapitalization(.never).autocorrectionDisabled()
                        .accessibilityLabel("Fleet router address")
                }
                VStack(alignment: .leading, spacing: 6) {
                    Text("Topic prefix").font(.caption).foregroundStyle(.secondary)
                    TextField("Topic prefix", text: $prefix).textInputAutocapitalization(.never).autocorrectionDisabled()
                }
                VStack(alignment: .leading, spacing: 6) {
                    Text("Rover ID").font(.caption).foregroundStyle(.secondary)
                    TextField("Rover ID", text: $roverID).keyboardType(.numberPad)
                }
            } header: { Text("Fleet router") } footer: {
                Text("Use the router’s Tailscale address when connecting across networks. Keep Tailscale enabled on this phone and the router computer.")
            }
            Section {
                Text("Terra connects automatically after your rover is ready. Saving changes replaces the current fleet session without disconnecting Bluetooth.")
                if !error.isEmpty { Text(error).foregroundStyle(.red).accessibilityLabel("Error: \(error)") }
            }
        }
        .navigationTitle("Fleet connection").navigationBarTitleDisplayMode(.inline)
        .toolbar {
            ToolbarItem(placement: .confirmationAction) {
                Button("Save") {
                    if brain.applyDashboardSettings(endpoint: endpoint, prefix: prefix, roverID: roverID) { dismiss() }
                    else { error = "Enter a TCP address with a valid port, a topic prefix, and a non-negative rover ID." }
                }.fontWeight(.semibold)
            }
            ToolbarItem(placement: .cancellationAction) { Button("Cancel") { dismiss() } }
        }
    }
}
