import SwiftUI
import UIKit

enum TerraDestination: String, Hashable {
    case drive, missions, live, telemetry, settings, robots
    var title: String {
        switch self {
        case .drive: return "Debug drive"
        case .missions: return "Navigation tools"
        case .live: return "Map inspection"
        case .telemetry: return "Diagnostics"
        case .settings: return "Controller tools"
        case .robots: return "Rover setup"
        }
    }
}

enum TerraStyle {
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
    @State private var connectionSheet: TerraHomeSheet?
    var body: some View {
        NavigationStack {
            rootPage
                .safeAreaInset(edge: .bottom, alignment: .leading) {
                    HStack(spacing: 12) {
                        NavigationLink { TerraSettingsView(brain: brain) } label: {
                            Image(systemName: "gearshape").font(.title2)
                                .frame(width: 44, height: 44)
                        }.accessibilityLabel("Settings").buttonStyle(.bordered)
                        if brain.connectionFacts.authenticated {
                            Button(role: .destructive) { brain.emergencyStop() } label: {
                                Label("Emergency Stop", systemImage: "stop.circle.fill")
                                    .font(.headline).frame(maxWidth: .infinity, minHeight: 44)
                            }.buttonStyle(.bordered).tint(.red)
                        }
                    }.padding(.horizontal, 22).padding(.vertical, 8)
                        .frame(maxWidth: .infinity, alignment: .leading)
                        .background(TerraStyle.background)
                }
        }
        .tint(TerraStyle.forest)
        .task { UIApplication.shared.isIdleTimerDisabled = scenePhase == .active; brain.autoConnectHardware() }
        .terraPairingConsent(brain: brain, enabled: connectionSheet == nil)
        .sheet(item: $connectionSheet) { item in
            NavigationStack {
                switch item {
                case .rover: RoverConnectionDetails(brain: brain)
                case .fleet: FleetConnectionDetails(brain: brain)
                case .fleetSettings: FleetConnectionSettings(brain: brain)
                case .roverSetup: RoverSetupView(brain: brain)
                }
            }.terraPairingConsent(brain: brain)
        }
        .onChange(of: scenePhase) { _, phase in
            UIApplication.shared.isIdleTimerDisabled = phase == .active
            if phase == .background { brain.stop() }
            else if phase == .inactive { brain.setTarget(forward: 0, yaw: 0); brain.disarmHardware() }
            else if phase == .active { brain.autoConnectHardware() }
        }
    }
    @ViewBuilder private var rootPage: some View {
        #if DEBUG && targetEnvironment(simulator)
        switch ProcessInfo.processInfo.environment["TERRA_UI_PREVIEW_PAGE"] {
        case "settings": TerraSettingsView(brain: brain)
        case "fleet": FleetConnectionSettings(brain: brain)
        case "debug": TerraDebugTools(brain: brain)
        default: TerraConnectionHome(brain: brain, sheet: $connectionSheet)
        }
        #else
        TerraConnectionHome(brain: brain, sheet: $connectionSheet)
        #endif
    }

}

struct TerraMark: Shape {
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
