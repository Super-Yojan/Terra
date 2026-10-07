import SwiftUI

// All navigation destinations share the same controller; browsing never arms a rover.
enum TerraDestination: String, Hashable {
    case drive, missions, live, telemetry, settings, robots
    var title: String {
        switch self {
        case .drive: return "Drive"
        case .missions: return "Missions"
        case .live: return "Live View"
        case .telemetry: return "Telemetry"
        case .settings: return "Configurations"
        case .robots: return "Robots"
        }
    }
}

private enum TerraStyle {
    static let forest = Color(uiColor: UIColor { traits in
        traits.userInterfaceStyle == .dark
            ? UIColor(red: 0.63, green: 0.83, blue: 0.70, alpha: 1)
            : UIColor(red: 0.09, green: 0.23, blue: 0.17, alpha: 1)
    })
    static let buttonForest = Color(red: 0.09, green: 0.23, blue: 0.17)
    static let background = Color(uiColor: .systemGroupedBackground)
    static let surface = Color(uiColor: .secondarySystemGroupedBackground)
}

struct ContentView: View {
    @StateObject private var brain = PhoneController()
    @Environment(\.scenePhase) private var scenePhase
    @Environment(\.accessibilityReduceMotion) private var reduceMotion
    @State private var showSplash = true
    var body: some View {
        ZStack {
            TabView {
                NavigationStack {
                    TerraHomeView(brain: brain)
                        .navigationDestination(for: TerraDestination.self) { destination in
                            destinationView(destination)
                        }
                }
                .tabItem { Label("Home", systemImage: "house.fill") }
                NavigationStack { destinationView(.robots) }
                    .tabItem { Label("Robots", systemImage: "cpu.fill") }
                NavigationStack { ControllerDetailView(brain: brain, destination: .missions) }
                    .tabItem { Label("Missions", systemImage: "list.bullet") }
                NavigationStack { ControllerDetailView(brain: brain, destination: .settings) }
                    .tabItem { Label("More", systemImage: "ellipsis") }
            }
            .tint(TerraStyle.forest)
            .allowsHitTesting(!showSplash)
            .accessibilityHidden(showSplash)
            if showSplash {
                TerraSplashView { dismissSplash() }
                    .transition(.opacity)
                    .zIndex(1)
            }
        }
        .task {
            brain.autoConnectHardware()
            try? await Task.sleep(for: .seconds(1.4))
            guard !Task.isCancelled else { return }
            dismissSplash()
        }
        .onChange(of: scenePhase) { _, phase in
            if phase == .background { brain.stop() }
            else if phase == .inactive { brain.setTarget(forward: 0, yaw: 0); brain.disarmHardware() }
            else if phase == .active { brain.autoConnectHardware() }
        }
        .onDisappear { brain.stop(keepBluetooth: true) }
    }
    @ViewBuilder private func destinationView(_ destination: TerraDestination) -> some View {
        if destination == .robots {
            #if targetEnvironment(simulator)
            ControllerDetailView(brain: brain, destination: .robots)
            #else
            ActuatorLayoutView(brain: brain)
            #endif
        } else {
            ControllerDetailView(brain: brain, destination: destination)
        }
    }
    private func dismissSplash() {
        withAnimation(reduceMotion ? nil : .easeOut(duration: 0.3)) { showSplash = false }
    }
}

private struct TerraSplashView: View {
    var dismiss: () -> Void
    var body: some View {
        GeometryReader { geometry in
            ZStack {
                Image("MountainBackdrop")
                    .resizable().scaledToFill()
                    .frame(width: geometry.size.width, height: geometry.size.height)
                    .clipped()
                Color.black.opacity(0.22)
                VStack(spacing: 24) {
                    Spacer()
                    TerraMark().fill(.white).frame(width: 158, height: 120)
                    VStack(spacing: 10) {
                        Text("TERRA").font(.system(size: 43, weight: .medium, design: .rounded)).tracking(10)
                        Text("MOBILE").font(.system(size: 20, weight: .medium)).tracking(9)
                    }
                    Text("EXPLORE · OPERATE · DISCOVER")
                        .font(.system(size: 10, weight: .semibold)).tracking(3)
                        .padding(.top, 2)
                    Spacer()
                    Text("PART OF ARGOS").font(.system(size: 11, weight: .medium)).tracking(5)
                        .padding(.bottom, 58)
                }
                .foregroundStyle(.white)
            }
        }
        .ignoresSafeArea()
        .onTapGesture(perform: dismiss)
        .accessibilityElement(children: .ignore)
        .accessibilityLabel("Terra Mobile. Explore, operate, discover.")
        .accessibilityAddTraits(.isButton)
        .accessibilityHint("Continue to Home")
        .accessibilityAction { dismiss() }
    }
}

// A compact mountain and winding trail, drawn at any scale for the Terra identity.
private struct TerraMark: Shape {
    func path(in rect: CGRect) -> Path {
        var path = Path()
        func point(_ x: CGFloat, _ y: CGFloat) -> CGPoint {
            CGPoint(x: rect.minX + x * rect.width, y: rect.minY + y * rect.height)
        }
        path.move(to: point(0.5, 0))
        path.addLine(to: point(0.88, 0.76))
        path.addLine(to: point(0.67, 0.7))
        path.addLine(to: point(0.52, 0.42))
        path.addLine(to: point(0.37, 0.65))
        path.addLine(to: point(0.18, 0.65))
        path.closeSubpath()
        path.move(to: point(0.14, 0.73))
        path.addCurve(to: point(0.45, 0.81), control1: point(0.49, 0.7), control2: point(0.57, 0.77))
        path.addCurve(to: point(0.0, 1.0), control1: point(0.33, 0.92), control2: point(0.17, 0.97))
        path.closeSubpath()
        path.move(to: point(0.46, 0.75))
        path.addCurve(to: point(0.4, 1), control1: point(0.75, 0.79), control2: point(0.7, 0.89))
        path.addLine(to: point(1, 1))
        path.addLine(to: point(0.9, 0.81))
        path.closeSubpath()
        return path
    }
}

private struct TerraHomeView: View {
    @ObservedObject var brain: PhoneController
    @State private var showStatus = false
    @Environment(\.dynamicTypeSize) private var typeSize
    private var connected: Bool { brain.hardwareReady || brain.zenohStatus.hasPrefix("Zenoh session open") }
    private var controllerRunning: Bool { brain.source != "Stopped" }
    private var connectionDescription: String {
        if brain.hardwareReady { return brain.hardwareArmed ? "Connected · Armed" : "Connected · Disarmed" }
        if brain.zenohStatus.hasPrefix("Zenoh session open") { return "Simulator connected" }
        if brain.zenohConnecting { return "Connecting…" }
        if controllerRunning { return brain.source }
        #if targetEnvironment(simulator)
        return "Simulator ready"
        #else
        return brain.hardwareStatus == "Disconnected" ? "Ready to connect" : brain.hardwareStatus
        #endif
    }
    var body: some View {
        ScrollView {
            VStack(spacing: 16) {
                header
                if connected || controllerRunning {
                    connectedDashboard
                } else {
                    roverCard
                    shortcuts
                    configurationCard

                }
            }
            .padding(.horizontal, 18).padding(.top, 8).padding(.bottom, 20)
            .frame(maxWidth: 600)
            .frame(maxWidth: .infinity)
        }
        .background(TerraStyle.background)
        .safeAreaInset(edge: .bottom, spacing: 0) {
            if !connected && !controllerRunning {
                NavigationLink(value: TerraDestination.missions) {
                    Label("Start Mission", systemImage: "play.circle.fill")
                        .font(.headline).frame(maxWidth: .infinity).padding(.vertical, 15)
                        .foregroundStyle(.white).background(TerraStyle.buttonForest, in: RoundedRectangle(cornerRadius: 15))
                }
                .buttonStyle(.plain).padding(.horizontal, 18).padding(.vertical, 8)
                .frame(maxWidth: 600).frame(maxWidth: .infinity)
                .background(TerraStyle.background)
            } else {
                missionControls.padding(.horizontal, 18).padding(.vertical, 8)
                    .frame(maxWidth: 600).frame(maxWidth: .infinity)
                    .background(TerraStyle.background)
            }
        }
        .toolbar(.hidden, for: .navigationBar)
        .alert("Robot Status", isPresented: $showStatus) {
            Button("Done", role: .cancel) { }
        } message: {
            Text("\(brain.status)\n\(brain.hardwareStatus)\n\(brain.autonomyReason)")
        }
    }
    private var header: some View {
        HStack(spacing: 10) {
            TerraMark().fill(TerraStyle.forest).frame(width: 46, height: 37)
                .accessibilityHidden(true)
            VStack(alignment: .leading, spacing: 0) {
                Text("TERRA").font(.system(size: 23, weight: .semibold, design: .rounded)).tracking(4)
                    .foregroundStyle(TerraStyle.forest)
                Text("Mobile").font(.caption).foregroundStyle(.secondary)
            }
            .accessibilityElement(children: .combine)
            Spacer(minLength: 8)
            Button { showStatus = true } label: {
                Image(systemName: "bell").font(.title3).frame(width: 44, height: 44)
            }.accessibilityLabel("Robot status")
            NavigationLink(value: TerraDestination.settings) {
                Image(systemName: "gearshape.fill").font(.title3).frame(width: 44, height: 44)
            }.accessibilityLabel("Configurations")
        }
        .foregroundStyle(.primary).padding(.bottom, 4)
    }
    private var roverCard: some View {
        VStack(alignment: .leading, spacing: 0) {
            RoverModelView().frame(height: 180)
            NavigationLink(value: TerraDestination.robots) {
                HStack {
                    VStack(alignment: .leading, spacing: 6) {
                        Text("Terra Rover").font(.title3.weight(.semibold))
                        HStack(spacing: 7) {
                            Circle().fill(connected ? Color.green : controllerRunning ? Color.orange : Color.secondary).frame(width: 8, height: 8)
                            Text(connectionDescription).font(.subheadline).foregroundStyle(.secondary)
                        }
                    }
                    Spacer()
                    Image(systemName: "chevron.right").font(.subheadline).foregroundStyle(.secondary)
                }.padding(16)
            }.buttonStyle(.plain)
        }
        .background(TerraStyle.surface)
        .clipShape(RoundedRectangle(cornerRadius: 18))
    }
    private var shortcuts: some View {
        LazyVGrid(columns: Array(repeating: GridItem(.flexible(), spacing: 12), count: typeSize.isAccessibilitySize ? 1 : 2), spacing: 12) {
            shortcut(.drive, icon: "gamecontroller.fill", subtitle: "Manual Control")
            shortcut(.missions, icon: "point.topleft.down.to.point.bottomright.curvepath", subtitle: "Waypoints & Tasks")
            shortcut(.live, icon: "camera.fill", subtitle: "Camera & Sensors")
            shortcut(.telemetry, icon: "chart.bar.fill", subtitle: "Status & Logs")
        }
    }
    private func shortcut(_ destination: TerraDestination, icon: String, subtitle: String) -> some View {
        NavigationLink(value: destination) {
            VStack(alignment: .leading, spacing: 4) {
                HStack {
                    Image(systemName: icon).font(.system(size: 27, weight: .semibold)).foregroundStyle(TerraStyle.forest)
                    Spacer()
                    Image(systemName: "chevron.right").font(.caption).foregroundStyle(.secondary)
                }.padding(.bottom, 9)
                Text(destination.title).font(.headline)
                Text(subtitle).font(.caption).foregroundStyle(.secondary)
            }
            .frame(maxWidth: .infinity, alignment: .leading).padding(17)
            .background(TerraStyle.surface, in: RoundedRectangle(cornerRadius: 16))
            .overlay(RoundedRectangle(cornerRadius: 16).stroke(.primary.opacity(0.045)))
        }.buttonStyle(.plain)
    }
    private var configurationCard: some View {
        NavigationLink(value: TerraDestination.settings) {
            HStack(spacing: 16) {
                Image(systemName: "gearshape.fill").font(.system(size: 30)).foregroundStyle(TerraStyle.forest)
                VStack(alignment: .leading, spacing: 3) {
                    Text("Configurations").font(.headline)
                    Text("Robot setup & tuning").font(.caption).foregroundStyle(.secondary)
                }
                Spacer()
                Image(systemName: "chevron.right").font(.caption).foregroundStyle(.secondary)
            }.padding(17).background(TerraStyle.surface, in: RoundedRectangle(cornerRadius: 16))
        }.buttonStyle(.plain)
    }
    private var connectedDashboard: some View {
        VStack(spacing: 13) {
            roverCard
            LazyVGrid(columns: Array(repeating: GridItem(.flexible(), spacing: 9), count: typeSize.isAccessibilitySize ? 2 : 4), spacing: 9) {
                metric("Speed", value: String(format: "%.2f", brain.measuredForward), unit: "m/s", icon: "speedometer")
                metric("Turn", value: String(format: "%.2f", brain.measuredYaw), unit: "rad/s", icon: "arrow.triangle.turn.up.right.diamond")
                metric("Left Motor", value: String(format: "%+.2f", brain.leftEffort), unit: "effort", icon: "bolt.fill")
                metric("Right Motor", value: String(format: "%+.2f", brain.rightEffort), unit: "effort", icon: "bolt.fill")
            }
            HStack {
                Text("Current Mission").font(.subheadline.weight(.semibold))
                Spacer()
                NavigationLink("View All", value: TerraDestination.missions).font(.subheadline).foregroundStyle(TerraStyle.forest)
            }.padding(.top, 5)
            NavigationLink(value: TerraDestination.missions) {
                HStack(spacing: 16) {
                    Image(systemName: "point.topleft.down.to.point.bottomright.curvepath").font(.system(size: 30)).foregroundStyle(TerraStyle.forest)
                    VStack(alignment: .leading, spacing: 5) {
                        Text(brain.waypointActive ? "Waypoint Navigation" : "No Active Mission").font(.subheadline.weight(.semibold))
                        Text(brain.waypointActive ? String(format: "%.1f m to goal", brain.waypointDistance) : "Choose your next waypoint")
                            .font(.caption).foregroundStyle(.secondary)
                    }
                    Spacer()
                    Text(brain.waypointActive ? "Active" : "Idle").font(.caption.weight(.semibold))
                        .foregroundStyle(TerraStyle.forest).padding(8)
                        .background(Color.green.opacity(0.1), in: RoundedRectangle(cornerRadius: 8))
                }.padding(16).background(TerraStyle.surface, in: RoundedRectangle(cornerRadius: 16))
            }.buttonStyle(.plain)
            NavigationLink(value: TerraDestination.missions) {
                HStack(spacing: 14) {
                    Image(systemName: "location.north.line.fill").font(.title2).foregroundStyle(.blue)
                        .frame(width: 40, height: 40).background(Color.blue.opacity(0.08), in: RoundedRectangle(cornerRadius: 9))
                    VStack(alignment: .leading, spacing: 3) {
                        Text("Autonomy Level").font(.subheadline.weight(.semibold))
                        Text(autonomyTitle).font(.caption).foregroundStyle(.secondary)
                    }
                    Spacer()
                    Image(systemName: "chevron.right").font(.caption)
                }.padding(14).background(TerraStyle.surface, in: RoundedRectangle(cornerRadius: 16))
            }.buttonStyle(.plain)
            shortcuts
            configurationCard
        }
    }
    private var missionControls: some View {
            HStack(spacing: 12) {
                Button {
                    brain.cancelWaypoint()
                    brain.setTarget(forward: 0, yaw: 0)
                    brain.setAutonomy("teleop")
                } label: {
                    Label("Take Over", systemImage: "pause.fill").font(.headline)
                        .frame(maxWidth: .infinity).padding(.vertical, 17)
                        .background(TerraStyle.surface, in: RoundedRectangle(cornerRadius: 16))
                }.foregroundStyle(TerraStyle.forest)
                Button {
                    brain.setTarget(forward: 0, yaw: 0)
                    brain.emergencyStop()
                } label: {
                    Label("Stop", systemImage: "stop.fill").font(.headline)
                        .frame(maxWidth: .infinity).padding(.vertical, 17)
                        .background(Color.red.opacity(0.18), in: RoundedRectangle(cornerRadius: 16))
                }.foregroundStyle(.red)
            }.buttonStyle(.plain)
    }
    private var autonomyTitle: String {
        switch brain.autonomyLevel {
        case "waypoint": return "Waypoint Navigation"
        case "assisted_teleop": return "Assisted Control"
        case "supervised": return "Supervised Search"
        default: return "Manual Control"
        }
    }
    private func metric(_ title: String, value: String, unit: String, icon: String) -> some View {
        VStack(spacing: 7) {
            Image(systemName: icon).font(.title3).foregroundStyle(TerraStyle.forest)
            Text(value).font(.subheadline.weight(.semibold)).monospacedDigit()
            Text(unit).font(.caption2).foregroundStyle(.secondary)
            Text(title).font(.caption2).foregroundStyle(.secondary)
        }.frame(maxWidth: .infinity).padding(.vertical, 12)
            .background(TerraStyle.surface, in: RoundedRectangle(cornerRadius: 13))
            .accessibilityElement(children: .ignore)
            .accessibilityLabel("\(title), \(value) \(unit)")
    }
}
