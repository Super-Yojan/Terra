package dev.superyojan.terra.phone

import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.text.KeyboardOptions
import androidx.compose.material3.Button
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.OutlinedButton
import androidx.compose.material3.OutlinedTextField
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.setValue
import androidx.compose.ui.Modifier
import androidx.compose.ui.text.input.KeyboardType
import androidx.compose.ui.text.input.PasswordVisualTransformation
import androidx.compose.ui.unit.dp

/** ARGOS fleet router. Connecting never arms motors; the Rust gate still needs arm and the mount check. */
@Composable
fun RouterSection(session: TerraSession) {
    val saved = remember { session.savedRouter() }
    var endpoint by remember { mutableStateOf(saved.endpoint) }
    var prefix by remember { mutableStateOf(saved.prefix) }
    var roverId by remember { mutableStateOf(saved.roverId) }
    var user by remember { mutableStateOf(saved.user) }
    var password by remember { mutableStateOf(saved.password) }
    SectionCard("ARGOS") {
        Readout("Router", session.routerStatus)
        OutlinedTextField(endpoint, { endpoint = it }, Modifier.fillMaxWidth(), label = { Text("Endpoint") }, singleLine = true)
        OutlinedTextField(prefix, { prefix = it }, Modifier.fillMaxWidth(), label = { Text("Topic prefix") }, singleLine = true)
        OutlinedTextField(
            roverId,
            { roverId = it },
            Modifier.fillMaxWidth(),
            label = { Text("Rover ID") },
            singleLine = true,
            keyboardOptions = KeyboardOptions(keyboardType = KeyboardType.Number),
        )
        OutlinedTextField(user, { user = it }, Modifier.fillMaxWidth(), label = { Text("User") }, singleLine = true)
        OutlinedTextField(
            password,
            { password = it },
            Modifier.fillMaxWidth(),
            label = { Text("Password") },
            singleLine = true,
            visualTransformation = PasswordVisualTransformation(),
            keyboardOptions = KeyboardOptions(keyboardType = KeyboardType.Password),
        )
        Row(horizontalArrangement = Arrangement.spacedBy(8.dp)) {
            Button(
                onClick = { session.connectRouter(endpoint, prefix, roverId, user, password) },
                enabled = !session.routerConnecting && !session.routerConnected,
            ) { Text(if (session.routerConnecting) "Connecting…" else "Connect") }
            OutlinedButton(onClick = { session.disconnectRouter() }) { Text("Disconnect") }
        }
        Text(
            "Start Phone sensors or Bluetooth drive first. While connected, ARGOS owns the velocity target and the phone stick is ignored. A lost link retries with back-off. Disconnect clears remote intent.",
            style = MaterialTheme.typography.bodySmall,
        )
    }
}
