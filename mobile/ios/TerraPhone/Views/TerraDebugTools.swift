import SwiftUI

struct TerraDebugTools: View {
    @ObservedObject var brain: PhoneController
    var body: some View {
        List {
            Section {
                Text("For development and troubleshooting. Normal fleet operation happens in ARGOS.")
                    .font(.subheadline).foregroundStyle(.secondary)
            }
            Section {
                NavigationLink("Manual drive") { ControllerDetailView(brain: brain, destination: .drive) }
                NavigationLink("Controller & simulation") { ControllerDetailView(brain: brain, destination: .settings) }
                NavigationLink("Map inspection") { ControllerDetailView(brain: brain, destination: .live) }
                NavigationLink("Waypoint & autonomy tools") { ControllerDetailView(brain: brain, destination: .missions) }
            }
        }.navigationTitle("Debug tools")
    }
}
