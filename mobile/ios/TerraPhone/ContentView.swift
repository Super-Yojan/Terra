import SwiftUI

struct ContentView: View {
    @StateObject private var brain = PhoneController()
    @Environment(\.scenePhase) private var scenePhase
    @State private var forward = 0.0
    @State private var yaw = 0.0
    @AppStorage("zenohEndpoint") private var zenohEndpoint = "tcp/127.0.0.1:7447"
    @AppStorage("zenohRoverID") private var zenohRoverID = "0"
    @AppStorage("waypointOriginLat") private var originLat = "38.8297"
    @AppStorage("waypointOriginLon") private var originLon = "-77.3075"
    @AppStorage("waypointGoalLat") private var goalLat = "38.82981"
    @AppStorage("waypointGoalLon") private var goalLon = "-77.3075"
    @AppStorage("waypointToken") private var waypointToken = "gmu-north"
    @State private var entryNote = ""
    private let waypointReach = 49.0
    var body: some View {
        NavigationStack {
            Form {
                Section("Controller") {
                    LabeledContent("Source", value: brain.source)
                    LabeledContent("Status", value: brain.status)
                    HStack {
                        Button("Simulated rover") { brain.startSimulation() }
                        Spacer()
                        Button("Phone IMU + VIO") { brain.startPhone() }
                    }
                    Button("Stop controller", role: .destructive) { forward = 0; yaw = 0; brain.stop() }
                }
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
                    Text("Use localhost in iOS Simulator. On an iPhone, enter the Mac’s LAN address. Connect starts at zero and subscribes to that rover’s depth camera. The occupancy map below fills in from simulator depth and the exposure pose published with each frame. Stop or leaving the app disconnects.")
                        .font(.footnote).foregroundStyle(.secondary)
                }
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
                    Text("Phone mapping uses scene depth when available. Initial camera height is assumed 0.5 m above flat ground; calibrate before using the map for navigation. Bevy Zenoh mode uses the simulator’s exposure-aligned camera pose instead, with ground at robotics Z = 0.").font(.footnote).foregroundStyle(.secondary)
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
                        .disabled(brain.waypointActive)
                    Button("Cancel waypoint", role: .destructive) { brain.cancelWaypoint() }
                        .disabled(!brain.waypointActive)
                    if !entryNote.isEmpty {
                        Text(entryNote).font(.footnote).foregroundStyle(.red)
                    }
                    Text("Tap the map to drop the goal. On that grid +X is north and +Y is west, the same frame as the blue rover marker. Go latches the coordinate on this phone. Connect to Bevy first: the phone sends the twist as cmd_vel and does not publish a Zenoh goal. A goal already latched in the simulator ignores these twists until it is cancelled.")
                        .font(.footnote).foregroundStyle(.secondary)
                }
                Section("Velocity target") {
                    LabeledContent("Forward", value: String(format: "%.2f m/s", forward))
                    Slider(value: $forward, in: -1...1, step: 0.05)
                        .accessibilityLabel("Forward velocity in metres per second")
                        .disabled(brain.waypointActive)
                    LabeledContent("Left turn", value: String(format: "%.2f rad/s", yaw))
                    Slider(value: $yaw, in: -1...1, step: 0.05)
                        .accessibilityLabel("Turn rate in radians per second")
                        .disabled(brain.waypointActive)
                    Button("Zero target") { forward = 0; yaw = 0 }
                        .disabled(brain.waypointActive)
                }
                Section("Feedback") {
                    LabeledContent("Measured forward", value: String(format: "%.2f m/s", brain.measuredForward))
                    LabeledContent("Measured turn", value: String(format: "%.2f rad/s", brain.measuredYaw))
                    LabeledContent("Left motor", value: String(format: "%+.3f", brain.leftEffort))
                    LabeledContent("Right motor", value: String(format: "%+.3f", brain.rightEffort))
                    Text("Motor effort is signed from −1 to +1. This phone displays that effort only; no motor hardware is connected. Robot-side PWM, the enable switch, and the command watchdog are in the terra-motors adapter. Zero effort coasts and is not a brake.")
                        .font(.footnote).foregroundStyle(.secondary)
                }
                Section("Shared Rust test") {
                    Button("Run velocity benchmark") { brain.runBenchmark() }
                    Text(brain.benchmark).font(.footnote)
                }
                Section("Phone mounting") {
                    Text("Default: phone flat, screen up, top edge toward the rover’s front. Phone and rover origins are assumed coincident. Calibrate the mount before connecting motor hardware.")
                        .font(.footnote).foregroundStyle(.secondary)
                    Text("ARKit needs a physical supported iPhone and visual features. Use Simulated rover on the iOS simulator.")
                        .font(.footnote).foregroundStyle(.secondary)
                }
            }
            .navigationTitle("Terra Brain")
        }
        .onChange(of: forward) { _, _ in brain.setTarget(forward: forward, yaw: yaw) }
        .onChange(of: yaw) { _, _ in brain.setTarget(forward: forward, yaw: yaw) }
        .onChange(of: scenePhase) { _, phase in if phase != .active { brain.stop() } }
        .onDisappear { brain.stop() }
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


struct OccupancyMapView: View {
    let grid: OccupancyGrid?
    let rover: SIMD3<Double>
    var goal: SIMD2<Double>?
    var onSelect: ((Double, Double) -> Void)?
    var body: some View {
        Canvas { context, size in
            guard let grid, let layout = MapLayout(size: size, grid: grid) else { return }
            var unknown = Path(), free = Path(), occupied = Path(), uncertain = Path()
            for row in 0..<Int(grid.height) {
                for col in 0..<Int(grid.width) {
                    let rect = layout.cell(col: col, row: row)
                    let value = grid.occupancy[row * Int(grid.width) + col]
                    if value < 0 { unknown.addRect(rect) }
                    else if value < 45 { free.addRect(rect) }
                    else if value > 65 { occupied.addRect(rect) }
                    else { uncertain.addRect(rect) }
                }
            }
            context.fill(unknown, with: .color(.gray.opacity(0.18)))
            context.fill(free, with: .color(.green.opacity(0.25)))
            context.fill(uncertain, with: .color(.gray.opacity(0.4)))
            context.fill(occupied, with: .foreground)
            if let goal, let point = layout.screen(x: goal.x, y: goal.y, grid: grid) {
                context.fill(Path(ellipseIn: CGRect(x: point.x - 6, y: point.y - 6, width: 12, height: 12)), with: .color(.orange))
            }
            if let point = layout.screen(x: rover.x, y: rover.y, grid: grid) {
                context.fill(Path(ellipseIn: CGRect(x: point.x - 5, y: point.y - 5, width: 10, height: 10)), with: .color(.blue))
                var heading = Path(); heading.move(to: point)
                heading.addLine(to: CGPoint(x: point.x + 16 * cos(rover.z), y: point.y - 16 * sin(rover.z)))
                context.stroke(heading, with: .color(.blue), lineWidth: 3)
            }
        }
        .overlay {
            if grid == nil { ContentUnavailableView("No map yet", systemImage: "map", description: Text("Start a rover or connect to Bevy to collect depth.")) }
        }
        .overlay {
            GeometryReader { geo in
                Color.clear.contentShape(Rectangle())
                    .gesture(SpatialTapGesture().onEnded { value in
                        guard let grid, let layout = MapLayout(size: geo.size, grid: grid),
                              let point = layout.world(at: value.location, grid: grid) else { return }
                        onSelect?(point.x, point.y)
                    })
            }
        }
        .accessibilityElement(children: .ignore)
        .accessibilityLabel("Local occupancy map")
        .accessibilityValue(accessibilitySummary)
        .accessibilityHint("Tap to choose a waypoint")
    }
    private var accessibilitySummary: String {
        guard let grid else { return "No depth observations yet" }
        let known = grid.occupancy.filter { $0 >= 0 }.count
        let occupied = grid.occupancy.filter { $0 > 65 }.count
        return "\(known) observed cells, \(occupied) occupied cells. Rover position \(String(format: "%.1f", rover.x)), \(String(format: "%.1f", rover.y)) metres."
    }
}

private struct MapLayout {
    let scale: Double
    let left: Double
    let top: Double
    let gridWidth: Int
    let gridHeight: Int
    init?(size: CGSize, grid: OccupancyGrid) {
        guard grid.width > 0, grid.height > 0, grid.resolution > 0,
              grid.occupancy.count == Int(grid.width) * Int(grid.height) else { return nil }
        let scale = min(size.width / Double(grid.width), size.height / Double(grid.height))
        guard scale > 0 else { return nil }
        self.scale = scale
        left = (size.width - Double(grid.width) * scale) / 2
        top = (size.height - Double(grid.height) * scale) / 2
        gridWidth = Int(grid.width)
        gridHeight = Int(grid.height)
    }
    func cell(col: Int, row: Int) -> CGRect {
        CGRect(x: left + Double(col) * scale, y: top + Double(gridHeight - 1 - row) * scale, width: scale, height: scale)
    }
    func screen(x: Double, y: Double, grid: OccupancyGrid) -> CGPoint? {
        let point = CGPoint(
            x: left + (x - grid.originX) / grid.resolution * scale,
            y: top + (Double(gridHeight) - (y - grid.originY) / grid.resolution) * scale)
        let right = left + Double(gridWidth) * scale
        let bottom = top + Double(gridHeight) * scale
        guard point.x >= left, point.x <= right, point.y >= top, point.y <= bottom else { return nil }
        return point
    }
    func world(at location: CGPoint, grid: OccupancyGrid) -> SIMD2<Double>? {
        let col = (location.x - left) / scale
        let rowFromTop = (location.y - top) / scale
        guard col >= 0, col <= Double(gridWidth), rowFromTop >= 0, rowFromTop <= Double(gridHeight) else { return nil }
        return SIMD2(grid.originX + col * grid.resolution, grid.originY + (Double(gridHeight) - rowFromTop) * grid.resolution)
    }
}
