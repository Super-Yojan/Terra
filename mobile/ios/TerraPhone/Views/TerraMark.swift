import SwiftUI
import UIKit

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
