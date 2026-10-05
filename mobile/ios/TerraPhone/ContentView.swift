import SwiftUI

struct ContentView: View {
    @StateObject private var brain = PhoneController()
    @Environment(\.scenePhase) private var scenePhase
    @State private var forward = 0.0
    @State private var yaw = 0.0
    @AppStorage("zenohEndpoint") private var zenohEndpoint = "tcp/127.0.0.1:7447"
    @AppStorage("zenohRoverID") private var zenohRoverID = "0"
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
                    OccupancyMapView(grid: brain.occupancy, rover: brain.mapPose)
                        .frame(height: 280)
                    Text(brain.mapStatus).font(.footnote).foregroundStyle(.secondary)
                    HStack {
                        Label("Free", systemImage: "square.fill").foregroundStyle(.green)
                        Label("Occupied", systemImage: "square.fill").foregroundStyle(.primary)
                        Label("Unknown", systemImage: "square.fill").foregroundStyle(.secondary)
                    }.font(.caption)
                    if let grid = brain.occupancy {
                        Text(String(format: "%.0f × %.0f m · %.0f cm cells · world +X right, +Y up", Double(grid.width) * grid.resolution, Double(grid.height) * grid.resolution, grid.resolution * 100)).font(.caption)
                    }
                    Button("Clear map") { brain.clearMap() }
                    Text("Phone mapping uses scene depth when available. Initial camera height is assumed 0.5 m above flat ground; calibrate before using the map for navigation. Bevy Zenoh mode uses the simulator’s exposure-aligned camera pose instead, with ground at robotics Z = 0.").font(.footnote).foregroundStyle(.secondary)
                }
                Section("Velocity target") {
                    LabeledContent("Forward", value: String(format: "%.2f m/s", forward))
                    Slider(value: $forward, in: -1...1, step: 0.05)
                        .accessibilityLabel("Forward velocity in metres per second")
                    LabeledContent("Left turn", value: String(format: "%.2f rad/s", yaw))
                    Slider(value: $yaw, in: -1...1, step: 0.05)
                        .accessibilityLabel("Turn rate in radians per second")
                    Button("Zero target") { forward = 0; yaw = 0 }
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
}


struct OccupancyMapView: View {
    let grid: OccupancyGrid?
    let rover: SIMD3<Double>
    var body: some View {
        Canvas { context, size in
            guard let grid, grid.width > 0, grid.height > 0,
                  grid.occupancy.count == Int(grid.width) * Int(grid.height) else { return }
            let scale = min(size.width / Double(grid.width), size.height / Double(grid.height))
            let left = (size.width - Double(grid.width) * scale) / 2
            let top = (size.height - Double(grid.height) * scale) / 2
            var unknown = Path(), free = Path(), occupied = Path(), uncertain = Path()
            for row in 0..<Int(grid.height) {
                for col in 0..<Int(grid.width) {
                    let rect = CGRect(x: left + Double(col) * scale, y: top + Double(Int(grid.height) - 1 - row) * scale, width: scale, height: scale)
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
            let x = left + (rover.x - grid.originX) / grid.resolution * scale
            let y = top + (Double(grid.height) - (rover.y - grid.originY) / grid.resolution) * scale
            if x >= left && x <= left + Double(grid.width) * scale && y >= top && y <= top + Double(grid.height) * scale {
                let centre = CGPoint(x: x, y: y)
                context.fill(Path(ellipseIn: CGRect(x: x - 5, y: y - 5, width: 10, height: 10)), with: .color(.blue))
                var heading = Path(); heading.move(to: centre)
                heading.addLine(to: CGPoint(x: x + 16 * cos(rover.z), y: y - 16 * sin(rover.z)))
                context.stroke(heading, with: .color(.blue), lineWidth: 3)
            }
        }
        .overlay {
            if grid == nil { ContentUnavailableView("No map yet", systemImage: "map", description: Text("Start a rover or connect to Bevy to collect depth.")) }
        }
        .accessibilityElement(children: .ignore)
        .accessibilityLabel("Local occupancy map")
        .accessibilityValue(accessibilitySummary)
    }
    private var accessibilitySummary: String {
        guard let grid else { return "No depth observations yet" }
        let known = grid.occupancy.filter { $0 >= 0 }.count
        let occupied = grid.occupancy.filter { $0 > 65 }.count
        return "\(known) observed cells, \(occupied) occupied cells. Rover position \(String(format: "%.1f", rover.x)), \(String(format: "%.1f", rover.y)) metres."
    }
}
