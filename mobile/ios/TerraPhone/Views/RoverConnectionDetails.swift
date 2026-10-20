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
                NavigationLink("Choose rover or add rover") { RoverSelectionView(brain: brain) }
                Button("Disconnect rover", role: .destructive) { brain.disconnectHardware(); dismiss() }
                Button("Forget rover", role: .destructive) { confirmForget = true }
            }
            if brain.lastStopDiagnostic.message != nil {
                Section("Last disarm reason") { TerraStopReasonView(brain: brain) }
            }
        }.navigationTitle("Rover connection").navigationBarTitleDisplayMode(.inline)
            .toolbar { ToolbarItem(placement: .confirmationAction) { Button("Done") { dismiss() } } }
            .confirmationDialog("Forget this rover?", isPresented: $confirmForget, titleVisibility: .visible) {
                Button("Forget rover", role: .destructive) { brain.forgetHardwareRover(); dismiss() }
            } message: { Text("Terra will stop reconnecting to this rover and look for a new pairing candidate. This does not erase the rover’s ownership record.") }
    }
}

struct RoverSelectionView: View {
    @ObservedObject var brain: PhoneController
    @Environment(\.dismiss) private var dismiss
    @State private var roverToForget: TerraSavedRover?
    var body: some View {
        List {
            Section {
                if brain.savedRovers.rovers.isEmpty {
                    Text("No saved rovers").foregroundStyle(.secondary)
                }
                ForEach(brain.savedRovers.rovers) { rover in
                    VStack(alignment: .leading, spacing: 8) {
                        Button {
                            brain.selectHardwareRover(rover.id); dismiss()
                        } label: {
                            HStack {
                                Text(rover.name)
                                Spacer()
                                if brain.savedRovers.selected == rover.id {
                                    Text("Selected").font(.caption)
                                }
                            }.frame(minHeight: 44).contentShape(Rectangle())
                        }.buttonStyle(.plain)
                        Button("Forget \(rover.name)") { roverToForget = rover }
                            .font(.footnote).foregroundStyle(.secondary).frame(minHeight: 44)
                    }
                }
            } header: { Text("Saved rovers") }
            footer: { Text("Selecting a rover disconnects and disarms the current rover. Connection keeps motors disarmed.") }
            Section {
                Button("Add rover") { brain.addHardwareRover(); dismiss() }
                    .foregroundStyle(.primary).frame(minHeight: 44)
            } footer: {
                Text("Hold the new rover’s USR button for 3 seconds, then accept the pairing popup within 60 seconds. Holding USR clears that rover’s previous owner. Saved rovers stay on this phone.")
            }
        }
        .tint(.primary)
        .navigationTitle("Choose rover")
        .navigationBarTitleDisplayMode(.inline)
        .toolbar { ToolbarItem(placement: .confirmationAction) { Button("Done") { dismiss() } } }
        .confirmationDialog("Forget \(roverToForget?.name ?? "rover")?", isPresented: Binding(
            get: { roverToForget != nil }, set: { if !$0 { roverToForget = nil } }
        ), titleVisibility: .visible) {
            Button("Forget rover", role: .destructive) {
                if let rover = roverToForget { brain.forgetHardwareRover(rover.id) }
                roverToForget = nil
            }
        } message: { Text("This removes the rover from this phone. Hold USR on the rover to clear its ownership record.") }
    }
}
