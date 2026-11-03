package dev.superyojan.terra.phone

import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.text.KeyboardOptions
import androidx.compose.material3.Button
import androidx.compose.material3.Checkbox
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.OutlinedButton
import androidx.compose.material3.OutlinedTextField
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.setValue
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.text.input.KeyboardType
import androidx.compose.ui.unit.dp

/** Mount calibration through the shared Rust pose pipeline (same convention as iOS). */
@Composable
fun MountSection(session: TerraSession) {
    val mount = session.mount
    var height by remember(mount) { mutableStateOf("%.3f".format(mount.cameraHeight)) }
    var roll by remember(mount) { mutableStateOf("%.1f".format(mount.cameraRollDegrees)) }
    var pitch by remember(mount) { mutableStateOf("%.1f".format(mount.cameraPitchDegrees)) }
    var yaw by remember(mount) { mutableStateOf("%.1f".format(mount.cameraYawDegrees)) }
    SectionCard("Phone mount") {
        Readout("Camera height", "%.2f m".format(mount.cameraHeight))
        Readout("Roll · pitch · yaw", "%.0f° · %.0f° · %.0f°".format(mount.cameraRollDegrees, mount.cameraPitchDegrees, mount.cameraYawDegrees))
        Readout("Offset forward · left", "%.1f · %.1f cm".format(mount.cameraOffsetForward * 100, mount.cameraOffsetLeft * 100))
        Row(horizontalArrangement = Arrangement.spacedBy(8.dp)) {
            OutlinedButton(onClick = { session.applyTerraMount(topEdgeLeft = true) }) { Text("Terra · top edge left") }
            OutlinedButton(onClick = { session.applyTerraMount(topEdgeLeft = false) }) { Text("Top edge right") }
        }
        NumberField("Camera height (m)", height) { height = it }
        Row(horizontalArrangement = Arrangement.spacedBy(8.dp)) {
            NumberField("Roll °", roll, Modifier.weight(1f)) { roll = it }
            NumberField("Pitch °", pitch, Modifier.weight(1f)) { pitch = it }
            NumberField("Yaw °", yaw, Modifier.weight(1f)) { yaw = it }
        }
        Button(onClick = {
            val h = height.toDoubleOrNull()
            val r = roll.toDoubleOrNull()
            val p = pitch.toDoubleOrNull()
            val y = yaw.toDoubleOrNull()
            if (h == null || r == null || p == null || y == null) return@Button
            session.saveMount(
                mount.copy(
                    cameraHeight = h,
                    cameraRollDegrees = r,
                    cameraPitchDegrees = p,
                    cameraYawDegrees = y,
                    reviewed = true,
                ),
            )
        }) { Text("Save mount") }
        if (session.mountStatus.isNotEmpty()) Text(session.mountStatus, style = MaterialTheme.typography.bodySmall)

        Text("Camera offset from the turn centre", style = MaterialTheme.typography.titleSmall)
        Row(horizontalArrangement = Arrangement.spacedBy(8.dp)) {
            OutlinedButton(onClick = { session.beginOffsetFit() }, enabled = !session.fitCollecting) { Text("Start turn pass") }
            OutlinedButton(onClick = { session.finishOffsetFit() }, enabled = session.fitCollecting) { Text("Finish pass") }
        }
        Text(session.fitStatus, style = MaterialTheme.typography.bodySmall)

        Row(verticalAlignment = Alignment.CenterVertically) {
            Checkbox(checked = session.mountConfirmed, onCheckedChange = { session.confirmMount(it) })
            Text("The phone is in its mount. Motors stay held until this is checked.")
        }
        Text(
            "Uncalibrated means upright portrait with the rear camera forward. On Terra the phone is landscape with the screen toward the rover, so pick a preset: roll −90° for the top edge on the rover's left, +90° for the right (sign not yet checked on the rover). For the offset, run Phone sensors, turn the rover in place at least 90° for each pass, once each way.",
            style = MaterialTheme.typography.bodySmall,
        )
    }
}

@Composable
private fun NumberField(label: String, value: String, modifier: Modifier = Modifier.fillMaxWidth(), onChange: (String) -> Unit) {
    OutlinedTextField(
        value = value,
        onValueChange = onChange,
        modifier = modifier,
        label = { Text(label) },
        singleLine = true,
        keyboardOptions = KeyboardOptions(keyboardType = KeyboardType.Decimal),
    )
}
