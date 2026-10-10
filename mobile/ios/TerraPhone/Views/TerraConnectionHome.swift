import SwiftUI

struct TerraConnectionHome: View {
    @ObservedObject var brain: PhoneController
    @Environment(\.openURL) private var openURL
    @Binding var sheet: TerraHomeSheet?
    @AppStorage("autoConnectHardware") private var automaticConnection = true
    private var facts: TerraConnectionFacts { brain.connectionFacts }
    var body: some View {
        ScrollView {
            VStack(alignment: .leading, spacing: 20) {
                VStack(alignment: .leading, spacing: 10) {
                    Text(facts.summary).font(.title.bold())
                    Text(homeSubtitle)
                        .font(.subheadline).foregroundStyle(.secondary)
                }
                roverSection
                Divider()
                fleetSection
                if facts.authenticated {
                    Label(brain.hardwareArmed ? "Motors armed" : "Motors disarmed", systemImage: brain.hardwareArmed ? "exclamationmark.shield" : "shield.checkered")
                        .font(.footnote).foregroundStyle(brain.hardwareArmed ? Color.red : .secondary)
                    Text(brain.phoneTrackingStatus).font(.footnote).foregroundStyle(.secondary)
                }
            }
            .padding(22).frame(maxWidth: 600, alignment: .leading).frame(maxWidth: .infinity)
        }
        .background(TerraStyle.background)
        .navigationTitle("Terra")
        .navigationBarTitleDisplayMode(.inline)
        .toolbar {
            ToolbarItem(placement: .topBarLeading) {
                TerraMark().fill(TerraStyle.forest).frame(width: 28, height: 24).accessibilityHidden(true)
            }
        }
    }
    private var homeSubtitle: String {
        #if DEBUG && targetEnvironment(simulator)
        if ProcessInfo.processInfo.environment["TERRA_UI_PREVIEW_STATE"] != nil { return "Simulator preview · sample connection state" }
        #endif
        return "Your rover, connected to the fleet."
    }
    private var roverSection: some View {
        VStack(alignment: .leading, spacing: 10) {
            Text("Rover").font(.headline).foregroundStyle(.secondary)
            statusRow(name: brain.roverDisplayName, status: roverStatus, symbol: "sensor.tag.radiowaves.forward", busy: roverBusy)
            RoverModelView().frame(height: 160)
                .clipShape(RoundedRectangle(cornerRadius: 16))
            if facts.configurationAvailable,
               let layout = try? JSONDecoder().decode(ActuatorLayoutDraft.self, from: Data(brain.committedLayoutJSON.utf8)) {
                Text(layout.actuators.isEmpty ? "No actuators configured" : layout.actuators.map { $0.name }.joined(separator: " · "))
                    .font(.footnote).foregroundStyle(.secondary)
                    .fixedSize(horizontal: false, vertical: true)
            }
            Text(roverExplanation).font(.subheadline).foregroundStyle(.secondary).fixedSize(horizontal: false, vertical: true)
            switch facts.roverState {
            case .permissionRequired:
                Button("Open iPhone Settings") {
                    if let url = URL(string: UIApplication.openSettingsURLString) { openURL(url) }
                }.buttonStyle(.borderedProminent)
            case .paused, .failed:
                Button(automaticConnection ? "Retry rover connection" : "Enable automatic connection") {
                    automaticConnection = true
                    brain.retryAutomaticConnection()
                }.buttonStyle(.borderedProminent)
            case .needsSetup:
                Button("Review rover setup") { sheet = .roverSetup }.buttonStyle(.borderedProminent)
            case .connected:
                Button("Connection details") { sheet = .rover }.buttonStyle(.bordered)
            case .configuring:
                Button("Connection details") { sheet = .rover }.buttonStyle(.bordered)
            default: EmptyView()
            }
        }
    }
    private var fleetSection: some View {
        VStack(alignment: .leading, spacing: 10) {
            Text("Fleet dashboard").font(.headline).foregroundStyle(.secondary)
            statusRow(name: "ARGOS", status: fleetStatus, symbol: "point.3.connected.trianglepath.dotted", busy: facts.fleetState == .connecting)
            Text(fleetExplanation).font(.subheadline).foregroundStyle(.secondary).fixedSize(horizontal: false, vertical: true)
            switch facts.fleetState {
            case .setupRequired:
                Button("Set up fleet connection") { sheet = .fleetSettings }.buttonStyle(.borderedProminent)
            case .connected:
                Button("Connection details") { sheet = .fleet }.buttonStyle(.bordered)
            case .paused, .retrying:
                HStack {
                    Button("Retry fleet connection") { brain.retryFleetConnection() }.buttonStyle(.borderedProminent)
                    Button("Edit") { sheet = .fleetSettings }.buttonStyle(.bordered)
                }
            default: EmptyView()
            }
        }
    }
    private func statusRow(name: String, status: String, symbol: String, busy: Bool) -> some View {
        HStack(alignment: .center, spacing: 14) {
            Image(systemName: symbol).font(.title2).foregroundStyle(TerraStyle.forest)
                .frame(width: 44, height: 44).accessibilityHidden(true)
            VStack(alignment: .leading, spacing: 4) {
                Text(name).font(.title2.weight(.semibold))
                Text(status).font(.subheadline.weight(.medium))
            }
            Spacer(minLength: 0)
            if busy { ProgressView().accessibilityLabel(status) }
            else if status == "Connected" { Image(systemName: "checkmark.circle.fill").foregroundStyle(TerraStyle.forest).accessibilityHidden(true) }
        }.accessibilityElement(children: .combine)
    }
    private var roverBusy: Bool {
        [.searching, .connecting, .configuring, .retrying].contains(facts.roverState)
    }
    private var roverStatus: String {
        switch facts.roverState {
        case .simulator: return "Physical rover unavailable"
        case .searching: return "Searching nearby"
        case .pairing: return "Ready to pair"
        case .connecting: return "Connecting"
        case .configuring: return "Preparing rover"
        case .connected: return "Connected"
        case .needsSetup: return "Needs attention"
        case .paused: return "Connection paused"
        case .bluetoothOff: return "Bluetooth is off"
        case .permissionRequired: return "Bluetooth permission needed"
        case .retrying: return "Reconnecting"
        case .failed: return "Could not connect"
        }
    }
    private var roverExplanation: String {
        switch facts.roverState {
        case .simulator: return "Bluetooth pairing requires a physical iPhone. Simulation tools are in Settings → Debug tools."
        case .searching: return "Keep your rover nearby. For first setup, hold its USR button for 3 seconds until the LED blinks, then accept the pairing popup."
        case .pairing: return "Choose Connect in the pairing popup to connect this rover."
        case .connecting: return "Establishing your authenticated Bluetooth connection."
        case .configuring: return "Reading the rover’s saved configuration. Motors remain disarmed."
        case .connected: return "The fleet connection is handled automatically."
        case .needsSetup: return "Review the actuator setup or clear the reported fault before operating."
        case .paused: return automaticConnection ? "Automatic connection is paused. Retry when you are ready." : "Automatic connection is off. Enable it to discover and reconnect your rover."
        case .bluetoothOff: return "Turn on Bluetooth in iPhone Settings or Control Center. Terra will continue when it is available."
        case .permissionRequired: return "Allow Bluetooth access for Terra in iPhone Settings."
        case .retrying: return "Keep your rover powered on and nearby. Terra is trying again automatically."
        case .failed: return "Check that the rover is nearby and ready, then retry. Details are available in Diagnostics."
        }
    }
    private var fleetStatus: String {
        switch facts.fleetState {
        case .waitingForRover: return "Waiting for rover"
        case .setupRequired: return "Setup required"
        case .connecting: return "Connecting"
        case .connected: return "Connected"
        case .paused: return "Connection paused"
        case .waiting: return "Starting connection"
        case .retrying: return "Trying again"
        }
    }
    private var fleetExplanation: String {
        switch facts.fleetState {
        case .waitingForRover: return "Terra will connect to the saved fleet router after your rover is ready."
        case .setupRequired: return "Save your fleet router address once. Terra will use it automatically."
        case .connecting: return "Opening a session with your fleet router."
        case .connected: return "The fleet router session is open. Fleet membership and missions are managed in ARGOS."
        case .paused: return "This connection was paused. Retry to rejoin the fleet router."
        case .waiting: return "Terra will start the saved connection automatically."
        case .retrying: return "The router is not available yet. Check the router and Tailscale connection; Terra is retrying."
        }
    }
}
