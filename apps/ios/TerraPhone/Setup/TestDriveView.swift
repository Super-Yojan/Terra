import SwiftUI

/// Check the wiring after setup: drive with the joystick and watch each wheel.
/// The rover loop runs only while this screen is open.
struct TestDriveView: View {
    @ObservedObject var loop: RoverLoop
    @ObservedObject var motors: BluetoothLink
    @Environment(\.dismiss) private var dismiss

    var body: some View {
        NavigationStack {
            VStack(spacing: 20) {
                Text(motors.ready ? "Lift the wheels off the ground, then push forward: both sides should spin forward."
                                  : motors.status)
                    .foregroundStyle(Theme.secondary)
                    .multilineTextAlignment(.center)
                Text(String(format: "L %+.2f   R %+.2f", loop.command.left, loop.command.right))
                    .font(.title3.monospacedDigit())
                Joystick { forward, turn in loop.setStick(forward: forward, turn: turn) }
                    .disabled(!motors.ready || loop.estopped)
                    .opacity(motors.ready && !loop.estopped ? 1 : 0.4)
                Button(loop.estopped ? "Release E-STOP" : "E-STOP") { loop.setEstop(!loop.estopped) }
                    .font(.title2.bold())
                    .frame(maxWidth: .infinity, minHeight: 60)
                    .background(loop.estopped ? Color.gray : Color.red, in: RoundedRectangle(cornerRadius: 12))
                    .foregroundStyle(.white)
            }
            .padding(20)
            .background(Color.black)
            .navigationTitle("Test drive")
            .navigationBarTitleDisplayMode(.inline)
            .toolbar { ToolbarItem(placement: .topBarTrailing) { Button("Done") { dismiss() } } }
        }
        .preferredColorScheme(.dark)
        .tint(Theme.accent)
        .onAppear { loop.start() }
        .onDisappear { loop.setStick(forward: 0, turn: 0); loop.stop() }
    }
}
