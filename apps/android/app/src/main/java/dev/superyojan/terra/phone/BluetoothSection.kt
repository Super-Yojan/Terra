package dev.superyojan.terra.phone

import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.material3.Button
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.OutlinedButton
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.ui.Modifier
import androidx.compose.ui.unit.dp

@Composable
fun BluetoothSection(
    session: TerraSession,
    onScan: () -> Unit,
    onDisarm: () -> Unit,
) {
    val link = session.ble
    SectionCard("Bluetooth actuators") {
        if (link == null) {
            Text(
                "Bluetooth stays hidden on the emulator.",
                style = MaterialTheme.typography.bodySmall,
            )
            return@SectionCard
        }
        Readout("Link", link.statusText)
        Readout("Rover", link.summary)
        if (link.errorText.isNotEmpty()) {
            Text(link.errorText, color = MaterialTheme.colorScheme.error, style = MaterialTheme.typography.bodySmall)
            OutlinedButton(onClick = { link.clearError() }) { Text("Clear error") }
        }
        Row(horizontalArrangement = Arrangement.spacedBy(8.dp)) {
            Button(onClick = onScan, enabled = !link.scanning && !link.connected) {
                Text(if (link.scanning) "Scanning…" else "Scan")
            }
            OutlinedButton(onClick = { link.stopScan() }, enabled = link.scanning) { Text("Stop scan") }
        }
        link.rovers.forEach { rover ->
            val label = if (rover.pairing) "${rover.name} · pairing" else rover.name
            OutlinedButton(
                onClick = { link.connect(rover.address) },
                modifier = Modifier.fillMaxWidth(),
                enabled = !link.connected,
            ) { Text(label) }
        }
        if (link.connected) {
            OutlinedButton(onClick = { session.stop() }) { Text("Disconnect") }
        }
        Row(horizontalArrangement = Arrangement.spacedBy(8.dp)) {
            Button(
                onClick = { session.armBluetooth() },
                enabled = link.ready && !link.armed,
            ) { Text("Arm") }
            OutlinedButton(
                onClick = {
                    onDisarm()
                    session.disarmBluetooth()
                },
                enabled = link.connected,
            ) { Text("Disarm") }
        }
        Row(horizontalArrangement = Arrangement.spacedBy(8.dp)) {
            OutlinedButton(onClick = { link.stageActiveLayout() }, enabled = link.canStage) {
                Text("Stage layout")
            }
            OutlinedButton(onClick = { link.commitStagedLayout() }, enabled = link.canCommit) {
                Text("Commit")
            }
        }
        OutlinedButton(onClick = { link.refresh() }, enabled = link.connected && !link.armed) {
            Text("Refresh configuration")
        }
        Text(
            "Hold USR for three seconds, then scan for terra-XXXXXX. Accept pairing when the phone asks. Pairing does not arm the motors. Arm sends a safe output first. Motors move only while armed, with Phone IMU + VIO or Bluetooth drive running, the mount confirmed, and the Rust gate allowing it. Disarm, Stop, emergency stop, and leaving the app send safe output. Stop and leaving the app also drop the link. Stage sends the layout last read from the rover; Commit applies that stage. This screen has no layout editor. Radio behavior still needs a rover to verify.",
            style = MaterialTheme.typography.bodySmall,
        )
    }
}
