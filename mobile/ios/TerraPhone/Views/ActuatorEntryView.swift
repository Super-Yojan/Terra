import SwiftUI

struct ActuatorEntryView: View {
    @Binding var actuator: ActuatorDraft
    let capabilities: ActuatorCapabilities?
    private var ports: [String] { capabilities?.ports.keys.filter { capabilities?.ports[$0]?.kinds.contains(actuator.kind) == true }.sorted() ?? [] }
    var body: some View {
        Stepper("ID: \(actuator.id)", value: $actuator.id, in: 0...255)
        TextField("Name", text: $actuator.name)
        Picker("Output kind", selection: Binding(get: { actuator.kind }, set: { actuator.changeKind($0); actuator.port = "" })) {
            ForEach(capabilities?.supported_kinds ?? [], id: \.self) { Text($0.replacingOccurrences(of: "_", with: " ")).tag($0) }
        }
        Picker("Physical port", selection: $actuator.port) {
            Text("Choose capability port").tag("")
            ForEach(ports, id: \.self) { Text($0).tag($0) }
        }
        if let port = capabilities?.ports[actuator.port] { Text("\(port.frequency_hz) Hz · resources \(port.resources.joined(separator: ", "))").font(.caption) }
        Toggle("Invert output", isOn: $actuator.inverted).disabled(actuator.kind == "unidirectional_esc")
        if actuator.kind == "unidirectional_esc" { Text("Unidirectional ESCs cannot invert output; zero always means stop.").font(.footnote) }
        number("Minimum normalized command", value: $actuator.limits.min)
        number("Maximum normalized command", value: $actuator.limits.max)
        if actuator.kind == "positional_servo" {
            pulse("Minimum pulse µs", value: $actuator.calibration.min_us)
            pulse("Center pulse µs", value: $actuator.calibration.center_us)
            pulse("Maximum pulse µs", value: $actuator.calibration.max_us)
            Picker("Safe output", selection: Binding(get: { actuator.safe.type }, set: { actuator.safe = ActuatorSafe(type: $0, value: $0 == "position" ? 0 : nil) })) { Text("Configured position").tag("position"); Text("Disable PWM").tag("disabled") }
            if actuator.safe.type == "position" { optionalNumber("Safe position", value: $actuator.safe.value) }
            Text("Disabled PWM releases the servo while disarmed. Its control starts at center bounded by command limits on rearm.").font(.footnote)
        } else {
            Picker("Route", selection: Binding(get: { actuator.route.type }, set: { actuator.route = $0 == "manual" ? ActuatorRoute(type: $0, forward_coefficient: 1, turn_coefficient: 0) : ActuatorRoute(type: $0) })) { Text("Left effort").tag("left_effort"); Text("Right effort").tag("right_effort"); Text("Manual coefficients").tag("manual") }
            if actuator.route.type == "manual" { optionalNumber("Forward coefficient", value: $actuator.route.forward_coefficient); optionalNumber("Turn coefficient", value: $actuator.route.turn_coefficient) }
            if actuator.kind == "dc_motor" { optionalNumber("Maximum power fraction", value: $actuator.calibration.max_power_fraction) }
            if actuator.kind == "bidirectional_esc" { pulse("Reverse pulse µs", value: $actuator.calibration.reverse_us); pulse("Neutral pulse µs", value: $actuator.calibration.neutral_us); pulse("Forward pulse µs", value: $actuator.calibration.forward_us) }
            if actuator.kind == "unidirectional_esc" { pulse("Stop pulse µs", value: $actuator.calibration.stop_us); pulse("Full power pulse µs", value: $actuator.calibration.full_power_us) }
            if actuator.kind.contains("esc") { pulse("Arming duration ms", value: $actuator.calibration.arming_duration_ms) }
            Text("Safe propulsion is zero effort; ESCs maintain their neutral or stop pulse.").font(.footnote)
        }
    }
    private func number(_ title: String, value: Binding<Double>) -> some View { HStack { Text(title); TextField(title, value: value, format: .number).keyboardType(.numbersAndPunctuation).multilineTextAlignment(.trailing) } }
    private func optionalNumber(_ title: String, value: Binding<Double?>) -> some View { number(title, value: Binding(get: { value.wrappedValue ?? 0 }, set: { value.wrappedValue = $0 })) }
    private func pulse(_ title: String, value: Binding<Int?>) -> some View { HStack { Text(title); TextField(title, value: Binding(get: { value.wrappedValue ?? 0 }, set: { value.wrappedValue = $0 }), format: .number).keyboardType(.numberPad).multilineTextAlignment(.trailing) } }
}
