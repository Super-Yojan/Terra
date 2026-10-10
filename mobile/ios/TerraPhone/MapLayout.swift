import SwiftUI

struct MapLayout {
    let scale: Double
    let left: Double
    let top: Double
    let gridWidth: Int
    let gridHeight: Int
    init?(size: CGSize, grid: OccupancyGrid) {
        guard grid.width > 0, grid.height > 0, grid.resolution > 0,
              grid.occupancy.count == Int(grid.width) * Int(grid.height) else { return nil }
        let scale = min(size.width / Double(grid.width), size.height / Double(grid.height))
        guard scale > 0 else { return nil }
        self.scale = scale
        left = (size.width - Double(grid.width) * scale) / 2
        top = (size.height - Double(grid.height) * scale) / 2
        gridWidth = Int(grid.width)
        gridHeight = Int(grid.height)
    }
    func cell(col: Int, row: Int) -> CGRect {
        CGRect(x: left + Double(col) * scale, y: top + Double(gridHeight - 1 - row) * scale, width: scale, height: scale)
    }
    func screen(x: Double, y: Double, grid: OccupancyGrid) -> CGPoint? {
        let point = CGPoint(
            x: left + (x - grid.originX) / grid.resolution * scale,
            y: top + (Double(gridHeight) - (y - grid.originY) / grid.resolution) * scale)
        let right = left + Double(gridWidth) * scale
        let bottom = top + Double(gridHeight) * scale
        guard point.x >= left, point.x <= right, point.y >= top, point.y <= bottom else { return nil }
        return point
    }
    func world(at location: CGPoint, grid: OccupancyGrid) -> SIMD2<Double>? {
        let col = (location.x - left) / scale
        let rowFromTop = (location.y - top) / scale
        guard col >= 0, col <= Double(gridWidth), rowFromTop >= 0, rowFromTop <= Double(gridHeight) else { return nil }
        return SIMD2(grid.originX + col * grid.resolution, grid.originY + (Double(gridHeight) - rowFromTop) * grid.resolution)
    }
}
