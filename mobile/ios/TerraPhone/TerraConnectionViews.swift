import SwiftUI

enum TerraHomeSheet: String, Identifiable {
    case rover, fleet, fleetSettings, roverSetup
    var id: String { rawValue }
}

extension View {
    func terraPairingConsent(brain: PhoneController, enabled: Bool = true) -> some View {
        alert("Connect to this rover?", isPresented: Binding(
            get: { enabled && brain.pairingCandidate != nil }, set: { _ in }
        )) {
            Button("Connect") { brain.respondToPairing(connect: true) }
            Button("Not now", role: .cancel) { brain.respondToPairing(connect: false) }
        } message: { Text("\(brain.pairingCandidate?.name ?? "Terra rover") is ready to pair.") }
    }
}

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
