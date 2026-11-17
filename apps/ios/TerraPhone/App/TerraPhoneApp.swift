import SwiftUI

@main
struct TerraPhoneApp: App {
    @StateObject private var loop = RoverLoop()
    var body: some Scene { WindowGroup { ContentView(loop: loop, motors: loop.motors) } }
}

/// One screen: rover connection, start/stop, e-stop, joystick.
struct ContentView: View {
    @ObservedObject var loop: RoverLoop
    @ObservedObject var motors: BluetoothLink

    var body: some View {
        VStack(spacing: 16) {
            connection
            HStack {
                Button(loop.running ? "Stop" : "Start") { loop.running ? loop.stop() : loop.start() }
                    .buttonStyle(.borderedProminent)
                Spacer()
                Text(String(format: "L %+.2f  R %+.2f", loop.command.left, loop.command.right))
                    .font(.footnote.monospacedDigit()).foregroundStyle(.secondary)
            }
            Joystick { forward, turn in loop.setStick(forward: forward, turn: turn) }
                .disabled(!loop.running || loop.estopped)
                .opacity(loop.running && !loop.estopped ? 1 : 0.4)
            Button(loop.estopped ? "Release E-STOP" : "E-STOP") { loop.setEstop(!loop.estopped) }
                .font(.title2.bold()).frame(maxWidth: .infinity, minHeight: 64)
                .background(loop.estopped ? Color.gray : Color.red).foregroundStyle(.white)
                .clipShape(RoundedRectangle(cornerRadius: 14))
        }
        .padding()
    }

    @ViewBuilder private var connection: some View {
        VStack(alignment: .leading, spacing: 8) {
            HStack {
                Circle().fill(motors.ready ? Color.green : Color.orange).frame(width: 10, height: 10)
                Text(motors.status).font(.headline)
                Spacer()
                if motors.connected != nil { Button("Disconnect") { motors.disconnect() } }
                else { Button("Scan") { motors.scan() } }
            }
            if motors.connected == nil {
                ForEach(motors.rovers) { rover in
                    Button { motors.connect(rover.id) } label: {
                        Label(rover.pairing ? "\(rover.name) · pair" : rover.name, systemImage: "antenna.radiowaves.left.and.right")
                    }
                }
            }
        }
        .padding().background(Color(uiColor: .secondarySystemGroupedBackground)).clipShape(RoundedRectangle(cornerRadius: 14))
    }
}
