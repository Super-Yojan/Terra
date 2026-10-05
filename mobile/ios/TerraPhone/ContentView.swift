import SwiftUI

struct ContentView: View {
    @StateObject private var brain = PhoneController()
    @Environment(\.scenePhase) private var scenePhase
    @State private var forward = 0.0
    @State private var yaw = 0.0
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
                    Text("Motor effort is signed from −1 to +1. This starter app displays output; no motor hardware is connected.")
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
