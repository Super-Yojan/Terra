import SwiftUI

/// Controls for the sensor timing experiment: name the condition, start the CSV, and
/// optionally run the bare Core Motion (and ARKit) logger that bypasses Terra's queues.
struct SensorTimingView: View {
    @ObservedObject var brain: PhoneController
    @State private var condition = "C2-terra-idle"
    @State private var recording = SensorTimingProbe.shared.isRecording
    @State private var bareIMU = false
    @State private var bareARLoad = false
    @State private var counts: [String: Int] = [:]
    @State private var error: String?
    @State private var bare = BareSensorLogger()
    private let refresh = Timer.publish(every: 1, on: .main, in: .common).autoconnect()

    var body: some View {
        Form {
            Section {
                TextField("Condition", text: $condition)
                    .textInputAutocapitalization(.never).autocorrectionDisabled()
                    .disabled(recording)
                Button(recording ? "Stop recording" : "Start recording") { toggleRecording() }
                if let file = SensorTimingProbe.shared.currentFile {
                    Text(file.lastPathComponent).font(.caption.monospaced()).foregroundStyle(.secondary)
                }
                if let error { Text(error).foregroundStyle(.red) }
            } header: { Text("Recording") } footer: {
                Text("Writes Documents/SensorTiming/<condition>-<time>.csv. Use the condition codes from the experiment sheet (C0–C5).")
            }
            Section {
                Toggle("Bare Core Motion logger", isOn: $bareIMU)
                Toggle("Bare ARKit + LiDAR load", isOn: $bareARLoad)
                    .disabled(brain.phoneTrackingActive)
            } header: { Text("Bare logger") } footer: {
                Text("Its handlers only stamp arrival time, on their own queues. The IMU logger can run beside Terra. The ARKit load needs Terra tracking stopped, because only one AR session can own the camera.")
            }
            .onChange(of: bareIMU) { _, _ in applyBare() }
            .onChange(of: bareARLoad) { _, _ in applyBare() }
            Section("Rows this file") {
                if counts.isEmpty { Text("–").foregroundStyle(.secondary) }
                ForEach(counts.keys.sorted(), id: \.self) { key in
                    LabeledContent(key, value: "\(counts[key] ?? 0)").monospacedDigit()
                }
            }
        }
        .navigationTitle("Sensor timing")
        .onReceive(refresh) { _ in
            counts = SensorTimingProbe.shared.rowCounts()
            recording = SensorTimingProbe.shared.isRecording
        }
        .onDisappear { if !recording { bare.stop(); bareIMU = false; bareARLoad = false } }
    }

    private func toggleRecording() {
        error = nil
        if recording {
            SensorTimingProbe.shared.stop()
            recording = false
        } else {
            do { try SensorTimingProbe.shared.start(condition: condition); recording = true }
            catch { self.error = "Could not open the CSV: \(error.localizedDescription)" }
        }
    }

    private func applyBare() {
        if bareIMU || bareARLoad { bare.start(arLoad: bareARLoad && !brain.phoneTrackingActive) } else { bare.stop() }
    }
}
