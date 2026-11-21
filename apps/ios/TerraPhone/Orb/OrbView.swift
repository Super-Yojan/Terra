import SwiftUI

/// The Terra orb (Orb.metal): a glowing ring, purple fading to blue-white,
/// on a transparent background. Draws across the whole view; the ring spans
/// about 4/5 of the shorter side. Cheap enough to animate full-screen.
/// `animated: false` shows one still frame (home screen); `true` animates at
/// the display rate (mounted screen).
struct OrbView: View {
    var animated: Bool
    /// The still frame, and where the animation starts so the two match.
    static let stillTime = 3.0
    /// Every use of time in the shader is a phase, so wrapping by whole 2π
    /// periods is seamless and keeps float precision on long runs.
    private static let period = 2 * Double.pi * 100   // ~10 min

    @State private var start = Date().addingTimeInterval(-OrbView.stillTime)

    var body: some View {
        GeometryReader { geo in
            TimelineView(.animation(paused: !animated)) { context in
                let t = animated
                    ? context.date.timeIntervalSince(start).truncatingRemainder(dividingBy: Self.period)
                    : Self.stillTime
                Rectangle()
                    .fill(.black)
                    .colorEffect(ShaderLibrary.terraOrb(.float2(geo.size.width, geo.size.height), .float(t)))
            }
        }
        .allowsHitTesting(false)
        .accessibilityHidden(true)
    }
}
