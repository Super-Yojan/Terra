import SwiftUI

struct OccupancyMapView: View {
    let grid: OccupancyGrid?
    let rover: SIMD3<Double>
    var goal: SIMD2<Double>?
    var onSelect: ((Double, Double) -> Void)?
    var body: some View {
        Canvas { context, size in
            guard let grid, let layout = MapLayout(size: size, grid: grid) else { return }
            var unknown = Path(), free = Path(), occupied = Path(), uncertain = Path()
            for row in 0..<Int(grid.height) {
                for col in 0..<Int(grid.width) {
                    let rect = layout.cell(col: col, row: row)
                    let value = grid.occupancy[row * Int(grid.width) + col]
                    if value < 0 { unknown.addRect(rect) }
                    else if value < 45 { free.addRect(rect) }
                    else if value > 65 { occupied.addRect(rect) }
                    else { uncertain.addRect(rect) }
                }
            }
            context.fill(unknown, with: .color(.gray.opacity(0.18)))
            context.fill(free, with: .color(.green.opacity(0.25)))
            context.fill(uncertain, with: .color(.gray.opacity(0.4)))
            context.fill(occupied, with: .foreground)
            if let goal, let point = layout.screen(x: goal.x, y: goal.y, grid: grid) {
                context.fill(Path(ellipseIn: CGRect(x: point.x - 6, y: point.y - 6, width: 12, height: 12)), with: .color(.orange))
            }
            if let point = layout.screen(x: rover.x, y: rover.y, grid: grid) {
                context.fill(Path(ellipseIn: CGRect(x: point.x - 5, y: point.y - 5, width: 10, height: 10)), with: .color(.blue))
                var heading = Path(); heading.move(to: point)
                heading.addLine(to: CGPoint(x: point.x + 16 * cos(rover.z), y: point.y - 16 * sin(rover.z)))
                context.stroke(heading, with: .color(.blue), lineWidth: 3)
            }
        }
        .overlay {
            if grid == nil {
                #if targetEnvironment(simulator)
                ContentUnavailableView("No map yet", systemImage: "map", description: Text("Start a rover or connect to Bevy to collect depth."))
                #else
                ContentUnavailableView("No map yet", systemImage: "map", description: Text("Start a rover to collect depth."))
                #endif
            }
        }
        .overlay {
            GeometryReader { geo in
                Color.clear.contentShape(Rectangle())
                    .gesture(SpatialTapGesture().onEnded { value in
                        guard let grid, let layout = MapLayout(size: geo.size, grid: grid),
                              let point = layout.world(at: value.location, grid: grid) else { return }
                        onSelect?(point.x, point.y)
                    })
            }
        }
        .accessibilityElement(children: .ignore)
        .accessibilityLabel("Local occupancy map")
        .accessibilityValue(accessibilitySummary)
        .accessibilityHint("Tap to choose a waypoint")
    }
    private var accessibilitySummary: String {
        guard let grid else { return "No depth observations yet" }
        let known = grid.occupancy.filter { $0 >= 0 }.count
        let occupied = grid.occupancy.filter { $0 > 65 }.count
        return "\(known) observed cells, \(occupied) occupied cells. Rover position \(String(format: "%.1f", rover.x)), \(String(format: "%.1f", rover.y)) metres."
    }
}
