import SwiftUI

struct RoverConnectionDetails: View {
    @ObservedObject var brain: PhoneController
    @Environment(\.dismiss) private var dismiss
    @State private var confirmForget = false
    var body: some View {
        Form {
            Section {
                LabeledContent("Rover", value: brain.roverDisplayName)
                Text(brain.hardwareStatus).font(.subheadline)
                LabeledContent("Output", value: brain.hardwareArmed ? "Armed" : "Disarmed")
            }
            Section {
                Button("Disconnect rover", role: .destructive) { brain.disconnectHardware(); dismiss() }
                Button("Forget rover", role: .destructive) { confirmForget = true }
            }
        }.navigationTitle("Rover connection").navigationBarTitleDisplayMode(.inline)
            .toolbar { ToolbarItem(placement: .confirmationAction) { Button("Done") { dismiss() } } }
            .confirmationDialog("Forget this rover?", isPresented: $confirmForget, titleVisibility: .visible) {
                Button("Forget rover", role: .destructive) { brain.forgetHardwareRover(); dismiss() }
            } message: { Text("Terra will stop reconnecting to this rover and look for a new pairing candidate. This does not erase the rover’s ownership record.") }
    }
}
