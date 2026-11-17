import SwiftUI

/// The phone's own operator input: a joystick that becomes `Goal.velocity`.
/// Screen up is forward, screen left is a left turn (+w).
struct Joystick: View {
    /// Called with forward and turn in [-1, 1]; (0, 0) on release.
    let onChange: (Double, Double) -> Void
    @GestureState private var drag: CGSize? = nil

    var body: some View {
        GeometryReader { geo in
            let diameter = min(geo.size.width, geo.size.height)
            let radius = max(1, diameter / 2 - 44)
            let d = drag ?? .zero
            let length = (d.width * d.width + d.height * d.height).squareRoot()
            let scale = length > radius ? radius / length : 1
            ZStack {
                Circle().fill(Color(uiColor: .tertiarySystemGroupedBackground))
                Circle().fill(Color.green.opacity(0.8)).frame(width: 78, height: 78)
                    .offset(x: d.width * scale, y: d.height * scale)
            }
            .frame(width: diameter, height: diameter)
            .frame(maxWidth: .infinity, maxHeight: .infinity)
            .contentShape(Circle())
            .gesture(DragGesture(minimumDistance: 0).updating($drag) { value, state, _ in state = value.translation })
            .onChange(of: drag) { _, value in
                let (f, t) = Self.command(value, radius: Double(radius))
                onChange(f, t)
            }
            .accessibilityLabel("Drive joystick")
        }
    }

    /// Drag offset → (forward, turn), with an 8% dead zone.
    static func command(_ drag: CGSize?, radius: Double) -> (Double, Double) {
        guard let drag else { return (0, 0) }
        let x = Double(drag.width), y = Double(drag.height)
        let distance = (x * x + y * y).squareRoot()
        let amount = min(distance / radius, 1)
        guard amount > 0.08, distance > 0 else { return (0, 0) }
        let magnitude = (amount - 0.08) / 0.92
        return (-y / distance * magnitude, -x / distance * magnitude)
    }
}
