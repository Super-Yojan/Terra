import SwiftUI

struct DriveControlPanel: View {
    @ObservedObject var brain: PhoneController
    @Environment(\.scenePhase) private var scenePhase
    @State private var stopRequested = false
    @State private var command = DriveJoystickCommand.zero
    @State private var gestureID = UUID()
    private var hardware: Bool { brain.hardwareActive || brain.hardwareReady || brain.hardwareConfigurationReady }
    private var ready: Bool { hardware ? brain.hardwareReady : brain.source != "Stopped" }
    private var manual: Bool { brain.autonomyLevel == "teleop" && !brain.waypointActive }
    private var canDrive: Bool {
        DriveJoystickSafety.canDrive(ready: ready, manual: manual, stopped: stopRequested, foreground: scenePhase == .active)
            && !brain.dashboardConnected
    }
    var body: some View {
        VStack(spacing: 18) {
            HStack(spacing: 8) {
                Circle().fill(canDrive ? Color.green : stopRequested ? Color.red : Color.orange).frame(width: 9, height: 9)
                Text(statusText).font(.subheadline.weight(.semibold))
                Spacer()
                Image(systemName: "gamecontroller.fill").foregroundStyle(.secondary)
            }
            JoystickPad(enabled: canDrive, onCommand: send)
                .id(gestureID)
                .frame(height: 258)
            HStack(spacing: 12) {
                readout("Forward", value: command.forward, unit: hardware && !brain.hardwareFeedback ? "effort" : "m/s")
                readout("Turn", value: command.yaw, unit: hardware && !brain.hardwareFeedback ? "effort" : "rad/s")
            }
            Button {
                stopRequested = true; neutralize(); brain.emergencyStop()
            } label: {
                Label("Stop", systemImage: "stop.fill").font(.headline)
                    .frame(maxWidth: .infinity).padding(.vertical, 15)
                    .foregroundStyle(.white).background(Color.red, in: RoundedRectangle(cornerRadius: 14))
            }.buttonStyle(.plain).accessibilityLabel("Emergency stop rover")
            if stopRequested {
                Button("Reset Stop") {
                    neutralize(); brain.emergencyStop(reset: true); stopRequested = false
                }
                Text("Reset keeps the command at zero. Move the joystick to drive.").font(.caption).foregroundStyle(.secondary)
            } else if brain.dashboardConnected {
                Text("ARGOS controls movement while the fleet session is connected. Disconnect the fleet from Home to use the local debugging joystick.")
                    .font(.caption).foregroundStyle(.secondary)
            } else if !manual {
                Button("Take Over Manual Control") {
                    neutralize(); brain.cancelWaypoint(); brain.setAutonomy("teleop"); brain.stopHardware()
                }
            } else {
                Text(canDrive ? "Hold and drag to drive. Release to send zero." : "Connect or start a rover to enable manual control.")
                    .font(.caption).foregroundStyle(.secondary)
            }
        }
        .padding(.vertical, 10)
        .onChange(of: canDrive) { _, enabled in if !enabled { neutralize() } }
        .onChange(of: brain.source) { _, source in if source == "Stopped" { neutralize() } }
        .onChange(of: scenePhase) { _, phase in
            if phase != .active { neutralize(); brain.stopHardware() }
        }
        .onDisappear { neutralize(); brain.stopHardware() }
    }
    private var statusText: String {
        if stopRequested { return "Stopped · Reset Required" }
        if brain.dashboardConnected { return "ARGOS Control" }
        if canDrive { return "Manual Control Ready" }
        if !manual { return "Autonomy Active · Joystick Locked" }
        return ready ? "Connected · neutral output" : "No Rover Connected"
    }
    private func send(_ value: DriveJoystickCommand) {
        let safe = canDrive ? value : .zero
        command = safe
        brain.setTarget(forward: safe.forward, yaw: safe.yaw)
    }
    private func neutralize() {
        gestureID = UUID(); command = .zero
        brain.setTarget(forward: 0, yaw: 0)
    }
    private func readout(_ title: String, value: Double, unit: String) -> some View {
        VStack(spacing: 3) {
            Text(title).font(.caption).foregroundStyle(.secondary)
            Text(String(format: "%+.2f", value)).font(.title3.weight(.semibold)).monospacedDigit()
            Text(unit).font(.caption2).foregroundStyle(.secondary)
        }.frame(maxWidth: .infinity).padding(10)
            .background(Color(uiColor: .tertiarySystemGroupedBackground), in: RoundedRectangle(cornerRadius: 12))
            .accessibilityElement(children: .combine)
    }
}
