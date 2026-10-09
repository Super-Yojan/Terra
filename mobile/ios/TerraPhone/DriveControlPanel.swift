import SwiftUI

/// The local arming latch inhibits input immediately, before Bluetooth acknowledges
/// disarm/stop. Hardware motion still requires the rover's real armed status.
struct DriveControlPanel: View {
    @ObservedObject var brain: PhoneController
    @Environment(\.scenePhase) private var scenePhase
    @State private var locallyArmed = false
    @State private var stopRequested = false
    @State private var command = DriveJoystickCommand.zero
    @State private var gestureID = UUID()
    private let forest = Color(red: 0.09, green: 0.23, blue: 0.17)
    private var hardware: Bool { brain.hardwareActive || brain.hardwareReady || brain.hardwareConfigurationReady }
    private var ready: Bool { hardware ? brain.hardwareReady : brain.source != "Stopped" }
    private var manual: Bool { brain.autonomyLevel == "teleop" && !brain.waypointActive }
    private var canArm: Bool {
        DriveJoystickSafety.canArm(ready: ready, manual: manual, stopped: stopRequested, foreground: scenePhase == .active)
            && !brain.hardwareArming && (!hardware || !brain.hardwareBenchMode || brain.hardwareBenchEnabled)
    }
    private var canDrive: Bool { canArm && locallyArmed && (!hardware || brain.hardwareArmed) && !brain.dashboardConnected }
    private var armPending: Bool { hardware && locallyArmed && !brain.hardwareArmed }
    var body: some View {
        VStack(spacing: 18) {
            HStack(spacing: 8) {
                Circle().fill(canDrive ? Color.green : stopRequested ? Color.red : Color.orange).frame(width: 9, height: 9)
                Text(statusText).font(.subheadline.weight(.semibold))
                Spacer()
                Image(systemName: "gamecontroller.fill").foregroundStyle(.secondary)
            }
            if hardware && brain.hardwareBenchMode {
                VStack(alignment: .leading, spacing: 8) {
                    Text("Bench Mode").font(.headline)
                    Text("Software enable only. No physical power cutoff. Keep the rover supervised.").font(.caption).foregroundStyle(.secondary)
                    Button(brain.hardwareBenchEnabled ? "Disable Bench Control" : "Enable Bench Control") {
                        locallyArmed = false; neutralize()
                        brain.setBenchEnabled(!brain.hardwareBenchEnabled)
                    }.disabled(!ready || brain.hardwareArmed || brain.hardwareArming || stopRequested)
                    Text(brain.hardwareBenchEnabled ? "Enabled · arm separately to drive" : "Disabled · enable before arming").font(.caption)
                }.frame(maxWidth: .infinity, alignment: .leading)
            }
            JoystickPad(enabled: canDrive, onCommand: send)
                .id(gestureID)
                .frame(height: 258)
            HStack(spacing: 12) {
                readout("Forward", value: command.forward, unit: hardware && !brain.hardwareFeedback ? "effort" : "m/s")
                readout("Turn", value: command.yaw, unit: hardware && !brain.hardwareFeedback ? "effort" : "rad/s")
            }
            HStack(spacing: 12) {
                Button {
                    neutralize()
                    if locallyArmed || brain.hardwareArmed || brain.hardwareArming {
                        locallyArmed = false
                        brain.disarmHardware()
                    } else {
                        locallyArmed = true
                        if hardware { brain.armHardware() }
                    }
                } label: {
                    Label(locallyArmed || brain.hardwareArmed || brain.hardwareArming ? "Disarm" : "Arm", systemImage: "lock.shield.fill")
                        .font(.headline).frame(maxWidth: .infinity).padding(.vertical, 15)
                        .foregroundStyle(.white).background(forest, in: RoundedRectangle(cornerRadius: 14))
                }
                .disabled(!canArm && !locallyArmed && !brain.hardwareArmed && !brain.hardwareArming)
                .opacity(!canArm && !locallyArmed && !brain.hardwareArmed && !brain.hardwareArming ? 0.45 : 1)
                Button {
                    locallyArmed = false; stopRequested = true
                    neutralize()
                    brain.emergencyStop()
                } label: {
                    Label("Stop", systemImage: "stop.fill").font(.headline)
                        .frame(maxWidth: .infinity).padding(.vertical, 15)
                        .foregroundStyle(.white).background(Color.red, in: RoundedRectangle(cornerRadius: 14))
                }
                .accessibilityLabel("Emergency stop rover")
            }.buttonStyle(.plain)
            if stopRequested {
                Button("Reset Stop") {
                    locallyArmed = false; neutralize()
                    brain.emergencyStop(reset: true)
                    stopRequested = false
                }
                .disabled(hardware && !brain.configurationAllowed)
                Text("Reset leaves the rover disarmed. Arm again when ready.").font(.caption).foregroundStyle(.secondary)
            } else if brain.dashboardConnected {
                Text("ARGOS controls movement while the fleet session is connected. Disconnect the fleet from Home to use the local debugging joystick.")
                    .font(.caption).foregroundStyle(.secondary)
            } else if !manual {
                Button("Take Over Manual Control") {
                    locallyArmed = false; neutralize()
                    brain.cancelWaypoint(); brain.setAutonomy("teleop"); brain.disarmHardware()
                }
            } else {
                Text(canDrive ? "Hold and drag to drive. Release to send zero." : ready ? "Arm to enable the joystick." : "Connect or start a rover to enable manual control.")
                    .font(.caption).foregroundStyle(.secondary)
            }
        }
        .padding(.vertical, 10)
        .onAppear { locallyArmed = brain.hardwareArmed }
        .onChange(of: canDrive) { _, enabled in if !enabled { neutralize() } }
        .task(id: armPending) {
            guard armPending else { return }
            do { try await Task.sleep(for: .seconds(3)) } catch { return }
            if !brain.hardwareArmed && !brain.hardwareArming { locallyArmed = false; neutralize() }
        }
        .onChange(of: brain.hardwareArming) { _, arming in
            if !arming && !brain.hardwareArmed { locallyArmed = false; neutralize() }
        }
        .onChange(of: brain.hardwareArmed) { _, armed in if !armed && !brain.hardwareArming { locallyArmed = false; neutralize() } }
        .onChange(of: brain.source) { _, source in if source == "Stopped" { locallyArmed = false; neutralize() } }
        .onChange(of: scenePhase) { _, phase in
            if phase != .active { locallyArmed = false; neutralize(); brain.disarmHardware() }
        }
        .onDisappear { locallyArmed = false; neutralize(); brain.disarmHardware() }
    }
    private var statusText: String {
        if stopRequested { return "Stopped · Reset Required" }
        if brain.hardwareArming { return "Arming at Safe Output…" }
        if armPending { return "Waiting for Arm Acknowledgement…" }
        if brain.dashboardConnected { return brain.hardwareArmed ? "ARGOS Control · Hardware Armed" : "ARGOS Control · Disarmed" }
        if canDrive { return hardware ? "Hardware Armed · Manual Control" : "Manual Control Enabled" }
        if !manual { return "Autonomy Active · Joystick Locked" }
        return ready ? "Connected · Disarmed" : "No Rover Connected"
    }
    private func send(_ value: DriveJoystickCommand) {
        let safe = canDrive ? value : .zero
        command = safe
        brain.setTarget(forward: safe.forward, yaw: safe.yaw)
    }
    private func neutralize() {
        gestureID = UUID()
        command = .zero
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

private struct JoystickPad: View {
    let enabled: Bool
    let onCommand: (DriveJoystickCommand) -> Void
    @GestureState private var translation: CGSize? = nil
    private var drag: some Gesture {
        DragGesture(minimumDistance: 0)
            .updating($translation) { (value: DragGesture.Value, state: inout CGSize?, _: inout Transaction) in
                if enabled { state = value.translation }
            }
    }
    var body: some View {
        GeometryReader { geometry in
            let diameter: CGFloat = min(geometry.size.width, geometry.size.height)
            let radius: CGFloat = max(1, (diameter - 78) / 2 - 12)
            let delta: CGSize = translation ?? .zero
            let length: CGFloat = sqrt(delta.width * delta.width + delta.height * delta.height)
            let scale: CGFloat = length > radius ? radius / length : 1
            let offset = CGSize(width: delta.width * scale, height: delta.height * scale)
            JoystickSurface(diameter: diameter, offset: offset)
            .frame(maxWidth: .infinity, maxHeight: .infinity)
            .opacity(enabled ? 1 : 0.45)
            .contentShape(Circle())
            .highPriorityGesture(drag)
            .onChange(of: translation) { _, value in
                // GestureState also resets after cancellation, covering interrupted drags.
                publishTranslation(value, radius: radius)
            }
            .accessibilityElement(children: .ignore)
            .accessibilityLabel("Drive joystick")
            .accessibilityValue(enabled ? "Armed. Centered when released." : "Disarmed")
            .accessibilityHint("Hold and drag up to move forward, down to reverse, left or right to turn. Release to send zero.")
        }
    }
    private func publishTranslation(_ value: CGSize?, radius: CGFloat) {
        let x: Double = Double(value?.width ?? CGFloat.zero)
        let y: Double = Double(value?.height ?? CGFloat.zero)
        let command = DriveJoystickCommand.from(x: x, y: y, radius: Double(radius), contactActive: value != nil, enabled: enabled)
        onCommand(command)
    }
}

private struct JoystickSurface: View {
    let diameter: CGFloat
    let offset: CGSize
    var body: some View {
        ZStack {
            Circle().fill(Color(uiColor: .tertiarySystemGroupedBackground))
            guides
            arrows
            thumb
        }
        .font(.caption.weight(.semibold)).foregroundStyle(.secondary)
        .frame(width: diameter, height: diameter)
    }
    private var guides: some View {
        ZStack {
            Circle().stroke(Color.primary.opacity(0.08), lineWidth: 1)
            Circle().stroke(Color.primary.opacity(0.07), style: StrokeStyle(lineWidth: 1, dash: [3, 5])).padding(42)
            Rectangle().fill(Color.primary.opacity(0.06)).frame(width: 1).padding(.vertical, 24)
            Rectangle().fill(Color.primary.opacity(0.06)).frame(height: 1).padding(.horizontal, 24)
        }
    }
    private var arrows: some View {
        ZStack {
            VStack { Image(systemName: "chevron.up"); Spacer(); Image(systemName: "chevron.down") }.padding(15)
            HStack { Image(systemName: "chevron.left"); Spacer(); Image(systemName: "chevron.right") }.padding(15)
        }
    }
    private var thumb: some View {
        Circle().fill(Color(red: 0.09, green: 0.23, blue: 0.17))
            .frame(width: 78, height: 78)
            .overlay {
                Image(systemName: "plus").font(.title2.weight(.medium)).foregroundStyle(.white.opacity(0.8))
            }
            .shadow(color: .black.opacity(0.14), radius: 8, y: 4)
            .offset(offset)
    }
}
