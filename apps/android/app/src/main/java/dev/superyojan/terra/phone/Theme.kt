package dev.superyojan.terra.phone

import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.lightColorScheme
import androidx.compose.runtime.Composable
import androidx.compose.ui.graphics.Color

private val Forest = Color(0xFF1B4332)
private val Amber = Color(0xFFE07A3D)
private val Paper = Color(0xFFF7F4EF)

private val colors = lightColorScheme(
    primary = Forest,
    onPrimary = Color.White,
    secondary = Amber,
    background = Paper,
    surface = Color.White,
)

@Composable
fun TerraTheme(content: @Composable () -> Unit) {
    MaterialTheme(colorScheme = colors, content = content)
}
