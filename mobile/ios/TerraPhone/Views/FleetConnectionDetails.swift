import SwiftUI

struct FleetConnectionDetails: View {
    @ObservedObject var brain: PhoneController
    @Environment(\.dismiss) private var dismiss
    var body: some View {
        Form {
            Section {
                Text(brain.dashboardStatus)
                LabeledContent("Router", value: UserDefaults.standard.string(forKey: "dashboardRouterEndpoint") ?? "Not configured")
                LabeledContent("Rover ID", value: UserDefaults.standard.string(forKey: "dashboardRoverID") ?? "0")
            }
            Section {
                NavigationLink("Edit connection") { FleetConnectionSettings(brain: brain) }
                Button("Disconnect fleet", role: .destructive) { brain.disconnectDashboard(); dismiss() }
            }
        }.navigationTitle("Fleet connection").navigationBarTitleDisplayMode(.inline)
            .toolbar { ToolbarItem(placement: .confirmationAction) { Button("Done") { dismiss() } } }
    }
}
