import SwiftUI
import UIKit

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
