import SwiftUI

struct ControllerDetailView: View {
    @ObservedObject var brain: PhoneController
    var destination: TerraDestination = .drive
    @State private var forward = 0.0
    @State private var yaw = 0.0
    #if targetEnvironment(simulator)
    @AppStorage(TerraBevySessionStore.endpointKey) private var zenohEndpoint = "tcp/127.0.0.1:7447"
    @AppStorage(TerraBevySessionStore.roverIDKey) private var zenohRoverID = "0"
    #endif
    @AppStorage("waypointOriginLat") private var originLat = "38.8297"
    @AppStorage("waypointOriginLon") private var originLon = "-77.3075"
    @AppStorage("waypointGoalLat") private var goalLat = "38.82981"
    @AppStorage("waypointGoalLon") private var goalLon = "-77.3075"
    @AppStorage("waypointToken") private var waypointToken = "gmu-north"
    @State private var entryNote = ""
    @State private var searchClass = "person"
    @State private var searchMinX = "-20"
    @State private var searchMinY = "-20"
    @State private var searchMaxX = "20"
    @State private var searchMaxY = "20"
    @State private var searchBudget = "600"
    private let waypointReach = 49.0
    var body: some View {
        Form {
                if destination == .drive {
                    Section("Manual Control") { DriveControlPanel(brain: brain) }
                        .id(TerraDestination.drive)
                }
                if destination == .settings || destination == .telemetry {
                Section("Controller") {
                    LabeledContent("Source", value: brain.source)
                    LabeledContent("Status", value: brain.status)
                    if destination == .settings {
                        Button("Start local simulation") { brain.startSimulation() }
                        Button("Start phone sensors") { brain.startPhone() }
                        Button("Stop controller", role: .destructive) { forward = 0; yaw = 0; brain.stop() }
                    }
                }
                }
                if destination == .settings {
                // Compiled only for the iOS Simulator. A device build has no
                // Bevy controls and does not read the stored endpoint.
                #if targetEnvironment(simulator)
                Section("Bevy simulator · Zenoh") {
                    TextField("Zenoh endpoint", text: $zenohEndpoint)
                        .textInputAutocapitalization(.never).autocorrectionDisabled()
                        .accessibilityLabel("Zenoh TCP endpoint")
                    TextField("Rover ID", text: $zenohRoverID).keyboardType(.numberPad)
                        .accessibilityLabel("Simulator rover ID")
                    LabeledContent("Connection", value: brain.zenohStatus)
                    Button(brain.zenohConnecting ? "Connecting…" : "Connect to Bevy rover") {
                        forward = 0; yaw = 0
                        brain.startBevy(endpoint: zenohEndpoint, roverID: zenohRoverID)
                    }.disabled(brain.zenohConnecting)
                    Text("Use localhost (tcp/127.0.0.1:7447) on the same Mac as Bevy. Connect starts at zero and subscribes to that rover’s depth camera. The occupancy map below fills in from simulator depth and the exposure pose published with each frame. Stop or leaving the app disconnects.")
                        .font(.footnote).foregroundStyle(.secondary)
                }
                #endif
                }
                if destination == .live || destination == .missions {
                Section("Local occupancy map") {
                    OccupancyMapView(grid: brain.occupancy, rover: brain.mapPose, goal: selectedGoal) { north, west in
                        guard !brain.waypointActive,
                              let originLatitude = Double(originLat), let originLongitude = Double(originLon),
                              let geo = try? geographicPosition(originLatitude: originLatitude, originLongitude: originLongitude, north: north, west: west) else { return }
                        goalLat = String(format: "%.6f", geo.latitude)
                        goalLon = String(format: "%.6f", geo.longitude)
                    }
                        .frame(height: 280)
                    Text(brain.mapStatus).font(.footnote).foregroundStyle(.secondary)
                    HStack {
                        Label("Free", systemImage: "square.fill").foregroundStyle(.green)
                        Label("Occupied", systemImage: "square.fill").foregroundStyle(.primary)
                        Label("Unknown", systemImage: "square.fill").foregroundStyle(.secondary)
                        Label("Goal", systemImage: "circle.fill").foregroundStyle(.orange)
                    }.font(.caption)
                    if let grid = brain.occupancy {
                        Text(String(format: "%.0f × %.0f m · %.0f cm cells · world +X right, +Y up", Double(grid.width) * grid.resolution, Double(grid.height) * grid.resolution, grid.resolution * 100)).font(.caption)
                    }
                    Button("Clear map") { brain.clearMap() }
                    #if targetEnvironment(simulator)
                    Text("Phone mapping uses scene depth when available. Mapping waits for a stable detected floor; point the camera toward the ground during startup. Bevy Zenoh mode uses the simulator’s exposure-aligned camera pose instead, with ground at robotics Z = 0.").font(.footnote).foregroundStyle(.secondary)
                    #else
                    Text("Phone mapping uses scene depth when available. Mapping waits for a stable detected floor; point the camera toward the ground during startup.").font(.footnote).foregroundStyle(.secondary)
                    #endif
                }.id(TerraDestination.live)
                }
                if destination == .missions {
                Section("Mission autonomy") {
                    Picker("Requested level",selection:Binding(get:{brain.autonomyLevel},set:{brain.setAutonomy($0)})) {Text("Teleop").tag("teleop");Text("Assisted teleop").tag("assisted_teleop");Text("L2 · Direct waypoint").tag("waypoint_direct");Text("L3 · Obstacle-aware waypoint").tag("waypoint");Text("Supervised search").tag("supervised");if brain.searchAvailable {Text("L4 target search").tag("target_search")}}
                        .disabled(brain.hardwareActive && !brain.hardwareFeedback)
                    Text(brain.autonomyReason)
                    if let proposal=brain.proposedGoal {Text(brain.proposalText);HStack {Button("Approve search target") {brain.decideProposal(proposal,approve:true)};Button("Reject") {brain.decideProposal(proposal,approve:false)}}}
                    Button("Take over") {forward=0;yaw=0;brain.setAutonomy("teleop")}
                    Button("Emergency stop",role:.destructive) {forward=0;yaw=0;brain.emergencyStop()}
                    Button("Reset stop") {brain.emergencyStop(reset:true)}
                    Button("Finish run log") {brain.exportRun()}
                    if let log=brain.runLog {ShareLink("Share run log",item:log)}
                }.id(TerraDestination.missions)
                Section("Target search") {
                    if brain.searchAvailable {
                        TextField("Target class",text:$searchClass).textInputAutocapitalization(.never).autocorrectionDisabled().accessibilityLabel("Target class")
                        Text("Bounds in rover local metres").font(.caption)
                        HStack {TextField("Min x",text:$searchMinX).accessibilityLabel("Search minimum x");TextField("Max x",text:$searchMaxX).accessibilityLabel("Search maximum x")}.keyboardType(.numbersAndPunctuation)
                        HStack {TextField("Min y",text:$searchMinY).accessibilityLabel("Search minimum y");TextField("Max y",text:$searchMaxY).accessibilityLabel("Search maximum y")}.keyboardType(.numbersAndPunctuation)
                        TextField("Time budget in seconds",text:$searchBudget).keyboardType(.numbersAndPunctuation).accessibilityLabel("Search time budget in seconds")
                        Button("Start search") {
                            guard let a=Double(searchMinX),let b=Double(searchMinY),let c=Double(searchMaxX),let d=Double(searchMaxY),let t=Double(searchBudget) else {entryNote="Enter numeric search bounds and budget";return}
                            brain.startTargetSearch(targetClass:searchClass,minX:a,minY:b,maxX:c,maxY:d,budget:t)
                        }.disabled(brain.autonomyLevel != "target_search" || !brain.searchTerminal)
                    }else {Text("L4 people search requires a LiDAR phone with live camera tracking and scene depth.").font(.caption).foregroundStyle(.secondary)}
                    Text(brain.personDetectorStatus).font(.caption).foregroundStyle(.secondary)
                    if brain.autonomyLevel == "waypoint_direct" { Text("L2 follows directly without obstacle avoidance.").font(.caption) }
                    if !brain.searchPhase.isEmpty {Text(brain.searchPhase.replacingOccurrences(of:"_",with:" ").capitalized)}
                    if !brain.searchReportText.isEmpty {Text(brain.searchReportText).textSelection(.enabled)}
                    if !brain.searchReportHistory.isEmpty {DisclosureGroup("Confirmed reports") {ForEach(Array(brain.searchReportHistory.enumerated()),id:\.offset) { _,report in Text(report).textSelection(.enabled)}}}
                    HStack {Button("Pause") {brain.targetSearchAction("pause")};Button("Resume") {brain.targetSearchAction("resume")};Button("Cancel",role:.destructive) {brain.targetSearchAction("cancel")}}.disabled(brain.searchTerminal)
                }
                Section("Waypoint") {
                    TextField("Origin latitude", text: $originLat).keyboardType(.numbersAndPunctuation)
                        .accessibilityLabel("Origin latitude")
                        .disabled(brain.waypointActive)
                    TextField("Origin longitude", text: $originLon).keyboardType(.numbersAndPunctuation)
                        .accessibilityLabel("Origin longitude")
                        .disabled(brain.waypointActive)
                    TextField("Goal latitude", text: $goalLat).keyboardType(.numbersAndPunctuation)
                        .accessibilityLabel("Goal latitude")
                        .disabled(brain.waypointActive)
                    TextField("Goal longitude", text: $goalLon).keyboardType(.numbersAndPunctuation)
                        .accessibilityLabel("Goal longitude")
                        .disabled(brain.waypointActive)
                    TextField("Token", text: $waypointToken)
                        .textInputAutocapitalization(.never).autocorrectionDisabled()
                        .accessibilityLabel("Waypoint token")
                        .disabled(brain.waypointActive)
                    Button("Johnson Center, 12 m north") {
                        originLat = "38.8297"; originLon = "-77.3075"
                        goalLat = "38.82981"; goalLon = "-77.3075"
                        waypointToken = "gmu-north"
                    }.disabled(brain.waypointActive)
                    LabeledContent("Follower", value: brain.waypointStatus)
                    if brain.waypointActive {
                        LabeledContent("Distance", value: String(format: "%.1f m", brain.waypointDistance))
                    }
                    Button("Go to waypoint") { goToWaypoint() }
                        .disabled(brain.waypointActive || (brain.hardwareActive && !brain.hardwareFeedback))
                    Button("Cancel waypoint", role: .destructive) { brain.cancelWaypoint() }
                        .disabled(!brain.waypointActive)
                    if !entryNote.isEmpty {
                        Text(entryNote).font(.footnote).foregroundStyle(.red)
                    }
                    #if targetEnvironment(simulator)
                    Text("Tap the map to drop the goal. On that grid +X is north and +Y is west, the same frame as the blue rover marker. Select Waypoint autonomy before sending a goal. The shared Rust runtime selects motion locally; a Bevy connection sends the goal to the simulator-owned arbiter.")
                        .font(.footnote).foregroundStyle(.secondary)
                    #else
                    Text("Tap the map to drop the goal. On that grid +X is north and +Y is west, the same frame as the blue rover marker. Select Waypoint autonomy before sending a goal. The shared Rust runtime selects motion locally.")
                        .font(.footnote).foregroundStyle(.secondary)
                    #endif
                }
                }
                if destination == .settings {
                Section(brain.hardwareActive && !brain.hardwareFeedback ? "Normalized manual effort" : "Velocity target") {
                    LabeledContent("Forward", value: String(format: brain.hardwareActive && !brain.hardwareFeedback ? "%.2f effort" : "%.2f m/s", forward))
                    Slider(value: $forward, in: -1...1, step: 0.05)
                        .accessibilityLabel("Forward velocity in metres per second")
                        .disabled(brain.waypointActive)
                    LabeledContent("Left turn", value: String(format: brain.hardwareActive && !brain.hardwareFeedback ? "%.2f effort" : "%.2f rad/s", yaw))
                    Slider(value: $yaw, in: -1...1, step: 0.05)
                        .accessibilityLabel("Turn rate in radians per second")
                        .disabled(brain.waypointActive)
                    Button("Zero target") { forward = 0; yaw = 0 }
                        .disabled(brain.waypointActive)
                }
                }
                if destination == .telemetry {
                Section("Feedback") {
                    LabeledContent("Measured forward", value: String(format: "%.2f m/s", brain.measuredForward))
                    LabeledContent("Measured turn", value: String(format: "%.2f rad/s", brain.measuredYaw))
                    LabeledContent("Left motor", value: String(format: "%+.3f", brain.leftEffort))
                    LabeledContent("Right motor", value: String(format: "%+.3f", brain.rightEffort))
                    Text("Motor effort is normalized from −1 to +1 (unidirectional ESCs use 0 to 1). Bluetooth hardware requires explicit arming and an independent enable gate. Zero effort is not a brake.")
                        .font(.footnote).foregroundStyle(.secondary)
                }.id(TerraDestination.telemetry)
                Section("Connection details") {
                    Text(brain.hardwareStatus)
                    Text(brain.dashboardStatus)
                    Text(brain.autonomyReason)
                }
                Section("Run recording") {
                    Button("Finish run log") { brain.exportRun() }
                    if let log = brain.runLog { ShareLink("Share run log", item: log) }
                }
                }
                if destination == .settings {
                Section("Shared Rust test") {
                    Button("Run velocity benchmark") { brain.runBenchmark() }
                    Text(brain.benchmark).font(.footnote)
                }
                Section("Phone mounting") {
                    Text("Default: phone flat, screen up, top edge toward the rover’s front. Phone and rover origins are assumed coincident. Calibrate the mount before connecting motor hardware.")
                        .font(.footnote).foregroundStyle(.secondary)
                    Text("ARKit needs a physical supported iPhone and visual features. Use Simulated rover on the iOS simulator.")
                        .font(.footnote).foregroundStyle(.secondary)
                }.id(TerraDestination.settings)
                }
        }
        .navigationTitle(destination.title)
        .onChange(of: brain.hardwareArmed) { _, _ in forward = 0; yaw = 0 }
        .onChange(of: brain.hardwareActive) { _, _ in forward = 0; yaw = 0 }
        .onChange(of: forward) { _, _ in brain.setTarget(forward: forward, yaw: yaw) }
        .onChange(of: yaw) { _, _ in brain.setTarget(forward: forward, yaw: yaw) }
        .onDisappear {
            if destination == .drive || destination == .settings {
                forward = 0; yaw = 0; brain.setTarget(forward: 0, yaw: 0); brain.disarmHardware()
            }
        }
    }
    private var selectedGoal: SIMD2<Double>? {
        guard let originLatitude = Double(originLat), let originLongitude = Double(originLon),
              let latitude = Double(goalLat), let longitude = Double(goalLon),
              let point = try? tangentMetres(originLatitude: originLatitude, originLongitude: originLongitude, latitude: latitude, longitude: longitude) else { return nil }
        return SIMD2(point.north, point.west)
    }
    private func goToWaypoint() {
        guard let originLatitude = Double(originLat), let originLongitude = Double(originLon),
              let latitude = Double(goalLat), let longitude = Double(goalLon),
              (-85...85).contains(originLatitude), (-180...180).contains(originLongitude),
              (-85...85).contains(latitude), (-180...180).contains(longitude) else {
            entryNote = "Enter latitude from −85 to 85 and longitude from −180 to 180."
            return
        }
        entryNote = ""
        brain.engageWaypoint(originLatitude: originLatitude, originLongitude: originLongitude, latitude: latitude, longitude: longitude, token: waypointToken, halfExtent: waypointReach)
    }
}
