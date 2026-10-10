import SwiftUI

struct JoystickPad: View {
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
