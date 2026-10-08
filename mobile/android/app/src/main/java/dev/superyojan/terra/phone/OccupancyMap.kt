package dev.superyojan.terra.phone

import android.graphics.Bitmap
import androidx.compose.foundation.Canvas
import androidx.compose.foundation.gestures.detectTapGestures
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.runtime.Composable
import androidx.compose.runtime.remember
import androidx.compose.ui.Modifier
import androidx.compose.ui.geometry.Offset
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.graphics.asImageBitmap
import androidx.compose.ui.graphics.drawscope.Stroke
import androidx.compose.ui.input.pointer.pointerInput
import dev.superyojan.terra.core.OccupancyGrid
import kotlin.math.cos
import kotlin.math.sin

private val unknown = 0x2E000000
private val free = 0x402E7D32
private val occupied = 0xFF1B4332.toInt()
private val uncertain = 0x66999999

/**
 * World +X is right and +Y is up, matching the iOS occupancy canvas.
 * A tap returns metres north and west in the grid frame.
 */
@Composable
fun OccupancyMap(
    grid: OccupancyGrid?,
    roverX: Double,
    roverY: Double,
    roverYaw: Double,
    goalNorth: Double?,
    goalWest: Double?,
    onSelect: (Double, Double) -> Unit,
    modifier: Modifier = Modifier,
) {
    val bitmap = remember(grid) { grid?.toBitmap()?.asImageBitmap() }
    Canvas(
        modifier
            .fillMaxSize()
            .pointerInput(grid) {
                detectTapGestures { offset ->
                    val current = grid ?: return@detectTapGestures
                    val layout = MapLayout(size.width.toDouble(), size.height.toDouble(), current) ?: return@detectTapGestures
                    layout.world(offset.x.toDouble(), offset.y.toDouble(), current)?.let { (north, west) ->
                        onSelect(north, west)
                    }
                }
            },
    ) {
        val current = grid ?: return@Canvas
        val image = bitmap ?: return@Canvas
        val layout = MapLayout(size.width.toDouble(), size.height.toDouble(), current) ?: return@Canvas
        drawImage(
            image,
            dstOffset = androidx.compose.ui.unit.IntOffset(layout.left.toInt(), layout.top.toInt()),
            dstSize = androidx.compose.ui.unit.IntSize(
                (current.width.toInt() * layout.scale).toInt().coerceAtLeast(1),
                (current.height.toInt() * layout.scale).toInt().coerceAtLeast(1),
            ),
        )
        fun marker(x: Double, y: Double): Offset? {
            val point = layout.screen(x, y, current) ?: return null
            return Offset(point.first.toFloat(), point.second.toFloat())
        }
        if (goalNorth != null && goalWest != null) {
            marker(goalNorth, goalWest)?.let { point ->
                drawCircle(Color(0xFFE07A3D), radius = 8f, center = point)
            }
        }
        marker(roverX, roverY)?.let { point ->
            drawCircle(Color(0xFF1565C0), radius = 7f, center = point)
            drawLine(
                Color(0xFF1565C0),
                point,
                Offset(
                    point.x + 22f * cos(roverYaw).toFloat(),
                    point.y - 22f * sin(roverYaw).toFloat(),
                ),
                strokeWidth = 4f,
            )
            drawCircle(Color(0xFF1565C0), radius = 7f, center = point, style = Stroke(width = 2f))
        }
    }
}

private fun OccupancyGrid.toBitmap(): Bitmap {
    val width = this.width.toInt()
    val height = this.height.toInt()
    val pixels = IntArray(width * height)
    for (row in 0 until height) {
        for (col in 0 until width) {
            val value = occupancy[row * width + col].toInt()
            val color = when {
                value < 0 -> unknown
                value < 45 -> free
                value > 65 -> occupied
                else -> uncertain
            }
            pixels[(height - 1 - row) * width + col] = color
        }
    }
    return Bitmap.createBitmap(pixels, width, height, Bitmap.Config.ARGB_8888)
}

private class MapLayout(
    val scale: Double,
    val left: Double,
    val top: Double,
    private val gridWidth: Int,
    private val gridHeight: Int,
) {
    companion object {
        operator fun invoke(width: Double, height: Double, grid: OccupancyGrid): MapLayout? {
            val cellsWide = grid.width.toInt()
            val cellsHigh = grid.height.toInt()
            if (cellsWide <= 0 || cellsHigh <= 0 || grid.resolution <= 0.0) return null
            if (grid.occupancy.size != cellsWide * cellsHigh) return null
            val scale = minOf(width / cellsWide, height / cellsHigh)
            if (scale <= 0.0) return null
            return MapLayout(
                scale = scale,
                left = (width - cellsWide * scale) / 2.0,
                top = (height - cellsHigh * scale) / 2.0,
                gridWidth = cellsWide,
                gridHeight = cellsHigh,
            )
        }
    }

    fun screen(x: Double, y: Double, grid: OccupancyGrid): Pair<Double, Double>? {
        val pointX = left + (x - grid.originX) / grid.resolution * scale
        val pointY = top + (gridHeight - (y - grid.originY) / grid.resolution) * scale
        val right = left + gridWidth * scale
        val bottom = top + gridHeight * scale
        if (pointX < left || pointX > right || pointY < top || pointY > bottom) return null
        return pointX to pointY
    }

    fun world(px: Double, py: Double, grid: OccupancyGrid): Pair<Double, Double>? {
        val col = (px - left) / scale
        val rowFromTop = (py - top) / scale
        if (col < 0 || col > gridWidth || rowFromTop < 0 || rowFromTop > gridHeight) return null
        val north = grid.originX + col * grid.resolution
        val west = grid.originY + (gridHeight - rowFromTop) * grid.resolution
        return north to west
    }
}
