import SwiftUI

@main
struct TerraPhoneApp: App {
    @StateObject private var loop = RoverLoop()
    var body: some Scene { WindowGroup { HomeView(loop: loop, motors: loop.motors) } }
}

/// Home: a still orb and one action. "Mounted" means the phone is on the
/// rover: the screen goes black, the orb animates and the rover runs.
/// Setup (vehicle type, actuators, ports, base station) is behind the gear,
/// and opens by itself the first time.
struct HomeView: View {
    @ObservedObject var loop: RoverLoop
    @ObservedObject var motors: BluetoothLink
    @State private var showSetup = false
    @State private var mounted = false

    var body: some View {
        ZStack {
            Color.black.ignoresSafeArea()
            OrbView(animated: false).ignoresSafeArea()
            VStack(spacing: 0) {
                HStack {
                    Text("Terra").font(.title2.weight(.semibold))
                    Spacer()
                    Button { showSetup = true } label: {
                        Image(systemName: "gearshape").font(.title2)
                    }
                    .accessibilityLabel("Set up the rover")
                }
                .padding(.horizontal, 24)
                .padding(.top, 12)

                Spacer()

                Button { mounted = true } label: {
                    Text("Mounted")
                        .font(.headline)
                        .frame(maxWidth: .infinity, minHeight: 56)
                        .background(Theme.accent, in: Capsule())
                }
                .padding(.horizontal, 24)
                .padding(.bottom, 24)
            }
        }
        .foregroundStyle(.white)
        .preferredColorScheme(.dark)
        .sheet(isPresented: $showSetup) { SetupView(loop: loop, motors: motors) }
        .fullScreenCover(isPresented: $mounted) { MountedView(loop: loop) }
        .onAppear { if VehicleSetup.load() == nil { showSetup = true } }
    }
}

/// The phone is on the rover: everything black except the animated orb.
/// The rover loop runs and the screen stays on. Hold for two seconds to exit,
/// which stops the rover.
struct MountedView: View {
    @ObservedObject var loop: RoverLoop
    @Environment(\.dismiss) private var dismiss
    @State private var hint = true

    var body: some View {
        ZStack {
            Color.black.ignoresSafeArea()
            OrbView(animated: true).ignoresSafeArea()
            VStack {
                HStack {
                    if let status = loop.worldMapStatus {
                        Text(status)
                            .font(.caption)
                            .foregroundStyle(.white.opacity(0.6))
                    }
                    Spacer()
                    Button { loop.sendWorldMap() } label: {
                        Label("Send Map", systemImage: "map")
                            .font(.caption.weight(.medium))
                            .padding(.horizontal, 12)
                            .padding(.vertical, 8)
                            .background(.white.opacity(0.14), in: Capsule())
                    }
                }
                .padding(.horizontal, 20)
                .padding(.top, 16)
                Spacer()
                if hint {
                    Text("Hold anywhere to exit")
                        .font(.footnote)
                        .foregroundStyle(.white.opacity(0.4))
                        .padding(.bottom, 40)
                        .transition(.opacity)
                }
            }
        }
        .contentShape(Rectangle())
        .onLongPressGesture(minimumDuration: 2) { dismiss() }
        .statusBarHidden()
        .persistentSystemOverlays(.hidden)
        .onAppear {
            UIApplication.shared.isIdleTimerDisabled = true
            loop.start()
            DispatchQueue.main.asyncAfter(deadline: .now() + 4) {
                withAnimation(.easeOut(duration: 1)) { hint = false }
            }
        }
        .onDisappear {
            loop.stop()
            UIApplication.shared.isIdleTimerDisabled = false
        }
    }
}

enum Theme {
    static let accent = Color(red: 0.24, green: 0.42, blue: 0.88)
    static let card = Color(white: 0.12)
    static let line = Color(white: 0.22)
    static let secondary = Color(white: 0.6)
    static let warning = Color(red: 0.96, green: 0.65, blue: 0.14)
}
