import SwiftUI

struct FleetConnectionSettings: View {
    @ObservedObject var brain: PhoneController
    @Environment(\.dismiss) private var dismiss
    @State private var endpoint = TerraFleetDefaults.savedEndpoint
    @State private var prefix = UserDefaults.standard.string(forKey: "dashboardTopicPrefix") ?? "terra/phone"
    @State private var roverID = UserDefaults.standard.string(forKey: "dashboardRoverID") ?? "0"
    @State private var user = UserDefaults.standard.string(forKey: "dashboardRouterUser") ?? "terra"
    @State private var password = TerraFleetDefaults.password
    @State private var stunURL = FpvIceStore.load().stunURL
    @State private var turnURL = FpvIceStore.load().turnURL
    @State private var turnUsername = FpvIceStore.load().turnUsername
    @State private var turnCredential = FpvIceStore.load().turnCredential
    @State private var videoSaved = ""
    @State private var error = ""
    var body: some View {
        Form {
            Section {
                VStack(alignment: .leading, spacing: 6) {
                    Text("Router address").font(.caption).foregroundStyle(.secondary)
                    TextField("tls/<router address>:7448", text: $endpoint)
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
                TextField("Fleet account", text: $user).textInputAutocapitalization(.never).autocorrectionDisabled()
                SecureField("Fleet password", text: $password)
            } header: { Text("Fleet router") } footer: {
                Text("The default ARGOS router is public and uses encrypted TLS. Enter your fleet credentials; the password is stored in this phone’s Keychain.")
            }
            Section {
                TextField("STUN URL", text: $stunURL).textInputAutocapitalization(.never).autocorrectionDisabled()
                TextField("TURN URL", text: $turnURL).textInputAutocapitalization(.never).autocorrectionDisabled()
                TextField("TURN username", text: $turnUsername).textInputAutocapitalization(.never).autocorrectionDisabled()
                SecureField("TURN credential", text: $turnCredential)
                Button("Save video servers") {
                    guard FpvIceStore.save(stunURL: stunURL, turnURL: turnURL, turnUsername: turnUsername, turnCredential: turnCredential) else {
                        error = "Unable to save the TURN credential in Keychain."
                        videoSaved = ""
                        return
                    }
                    error = ""
                    videoSaved = "Video servers saved. The rover stays disarmed."
                }
                if !videoSaved.isEmpty { Text(videoSaved).foregroundStyle(.secondary) }
            } header: { Text("FPV video") } footer: {
                Text("A public STUN server is used when this field is blank. TURN is optional. The credential stays in the Keychain and is not written into the app. Saving video servers does not connect, arm, or move the rover.")
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
                    guard TerraRouterSettings.parse(endpoint: endpoint, prefix: prefix, roverID: roverID) != nil else { error = "Enter a valid TCP or TLS router, prefix, and rover ID."; return }
                    guard TerraFleetDefaults.savePassword(password) else { error = "Unable to save fleet password in Keychain."; return }
                    UserDefaults.standard.set(user, forKey: "dashboardRouterUser")
                    if brain.applyDashboardSettings(endpoint: endpoint, prefix: prefix, roverID: roverID) { dismiss() }
                    else { error = "Enter a TCP address with a valid port, a topic prefix, and a non-negative rover ID." }
                }.fontWeight(.semibold)
            }
            ToolbarItem(placement: .cancellationAction) { Button("Cancel") { dismiss() } }
        }
    }
}
