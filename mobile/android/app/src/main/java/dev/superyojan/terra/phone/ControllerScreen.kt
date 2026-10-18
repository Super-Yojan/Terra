package dev.superyojan.terra.phone

import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.ExperimentalLayoutApi
import androidx.compose.foundation.layout.FlowRow
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.height
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.rememberScrollState
import androidx.compose.foundation.verticalScroll
import androidx.compose.material3.Button
import androidx.compose.material3.Card
import androidx.compose.material3.FilterChip
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.OutlinedButton
import androidx.compose.material3.OutlinedTextField
import androidx.compose.material3.Slider
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableFloatStateOf
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.setValue
import androidx.compose.ui.Modifier
import androidx.compose.ui.unit.dp
import dev.superyojan.terra.core.geographicPosition
import dev.superyojan.terra.core.tangentMetres
import kotlin.math.round

private const val REACH_METRES = 49.0
private val levelLabels = mapOf(
    "teleop" to "Teleop",
    "assisted_teleop" to "Assisted teleop",
    "waypoint" to "Waypoint",
    "supervised" to "Supervised search",
    "explore" to "Explore",
)

@Composable
fun ControllerScreen(session: TerraSession, onStartPhone: () -> Unit) {
    var forward by remember { mutableFloatStateOf(0f) }
    var yaw by remember { mutableFloatStateOf(0f) }
    var endpoint by remember { mutableStateOf(session.savedEndpoint()) }
    var roverId by remember { mutableStateOf(session.savedRoverId()) }
    var originLat by remember { mutableStateOf("38.8297") }
    var originLon by remember { mutableStateOf("-77.3075") }
    var goalLat by remember { mutableStateOf("38.82981") }
    var goalLon by remember { mutableStateOf("-77.3075") }
    var token by remember { mutableStateOf("gmu-north") }
    var entryNote by remember { mutableStateOf("") }
    val goal = remember(originLat, originLon, goalLat, goalLon) { projectGoal(originLat, originLon, goalLat, goalLon) }

    Column(
        Modifier
            .fillMaxSize()
            .verticalScroll(rememberScrollState())
            .padding(16.dp),
        verticalArrangement = Arrangement.spacedBy(12.dp),
    ) {
        Text("TerraPhone", style = MaterialTheme.typography.headlineMedium, color = MaterialTheme.colorScheme.primary)
        Text(
            "Shared Rust controller. Zero effort coasts. It is not a brake.",
            style = MaterialTheme.typography.bodySmall,
        )

        SectionCard("Controller") {
            Readout("Source", session.source)
            Readout("Status", session.status)
            Row(horizontalArrangement = Arrangement.spacedBy(8.dp)) {
                Button(onClick = {
                    forward = 0f
                    yaw = 0f
                    session.startSimulation()
                }) { Text("Simulated rover") }
                Button(onClick = {
                    forward = 0f
                    yaw = 0f
                    onStartPhone()
                }) { Text("Phone IMU + VIO") }
            }
            OutlinedButton(onClick = {
                forward = 0f
                yaw = 0f
                session.setTarget(0.0, 0.0)
                session.stop()
            }) { Text("Stop") }
        }

        if (session.emulator) {
            SectionCard("Zorvane · Zenoh") {
                OutlinedTextField(
                    endpoint,
                    { endpoint = it },
                    Modifier.fillMaxWidth(),
                    label = { Text("Zenoh endpoint") },
                    singleLine = true,
                )
                OutlinedTextField(
                    roverId,
                    { roverId = it },
                    Modifier.fillMaxWidth(),
                    label = { Text("Rover ID") },
                    singleLine = true,
                )
                Readout("Connection", session.zenohStatus)
                Button(
                    onClick = {
                        forward = 0f
                        yaw = 0f
                        session.startZenoh(endpoint, roverId)
                    },
                    enabled = !session.zenohConnecting,
                ) { Text(if (session.zenohConnecting) "Connecting…" else "Connect to Zorvane") }
                Text(
                    "The emulator reaches the host at tcp/10.0.2.2:7447. Connect starts at zero and subscribes to that rover’s depth camera. Stop, or leaving the app, disconnects with a final zero.",
                    style = MaterialTheme.typography.bodySmall,
                )
            }
        } else {
            SectionCard("Bluetooth actuators") {
                Text(
                    "A physical phone hides Zenoh, the same split as an iPhone. The Bluetooth client is not in this build. Framing stays in terra-actuators (actuator_route, actuator_encode_frame, actuator_fragment) for the follow-up.",
                    style = MaterialTheme.typography.bodySmall,
                )
            }
        }

        SectionCard("Velocity target") {
            Text("Forward  %.2f m/s".format(forward))
            Slider(
                value = forward,
                onValueChange = {
                    forward = snap(it)
                    session.setTarget(forward.toDouble(), yaw.toDouble())
                },
                valueRange = -1f..1f,
                enabled = !session.waypointActive,
            )
            Text("Left turn  %.2f rad/s".format(yaw))
            Slider(
                value = yaw,
                onValueChange = {
                    yaw = snap(it)
                    session.setTarget(forward.toDouble(), yaw.toDouble())
                },
                valueRange = -1f..1f,
                enabled = !session.waypointActive,
            )
            OutlinedButton(
                onClick = {
                    forward = 0f
                    yaw = 0f
                    session.setTarget(0.0, 0.0)
                },
                enabled = !session.waypointActive,
            ) { Text("Zero target") }
        }

        SectionCard("Feedback") {
            Readout("Measured forward", "%.2f m/s".format(session.measuredForward))
            Readout("Measured turn", "%.2f rad/s".format(session.measuredYaw))
            Readout("Left motor", "%+.3f".format(session.leftEffort))
            Readout("Right motor", "%+.3f".format(session.rightEffort))
            Text(
                "Effort is normalized from −1 to +1. Zero effort is coast, not a brake.",
                style = MaterialTheme.typography.bodySmall,
            )
        }

        SectionCard("Local occupancy map") {
            OccupancyMap(
                grid = session.occupancy,
                roverX = session.mapX,
                roverY = session.mapY,
                roverYaw = session.mapYaw,
                goalNorth = goal?.first,
                goalWest = goal?.second,
                onSelect = { north, west ->
                    if (session.waypointActive) return@OccupancyMap
                    val originLatitude = originLat.toDoubleOrNull() ?: return@OccupancyMap
                    val originLongitude = originLon.toDoubleOrNull() ?: return@OccupancyMap
                    val geo = try {
                        geographicPosition(originLatitude, originLongitude, north, west)
                    } catch (_: Exception) {
                        return@OccupancyMap
                    }
                    goalLat = "%.6f".format(geo.latitude)
                    goalLon = "%.6f".format(geo.longitude)
                },
                modifier = Modifier
                    .fillMaxWidth()
                    .height(280.dp),
            )
            Text(session.mapStatus, style = MaterialTheme.typography.bodySmall)
            Text("Free is green, occupied is dark, unknown is faint. Orange is the goal. Blue is the rover. +X north, +Y west.", style = MaterialTheme.typography.bodySmall)
            session.occupancy?.let { grid ->
                Text(
                    "%.0f × %.0f m · %.0f cm cells".format(
                        grid.width.toDouble() * grid.resolution,
                        grid.height.toDouble() * grid.resolution,
                        grid.resolution * 100.0,
                    ),
                    style = MaterialTheme.typography.bodySmall,
                )
            }
            OutlinedButton(onClick = { session.clearMap() }) { Text("Clear map") }
            Text(session.depthNote, style = MaterialTheme.typography.bodySmall)
        }

        SectionCard("Mission autonomy") {
            Text(session.autonomyReason, style = MaterialTheme.typography.bodySmall)
            LevelPicker(session) {
                forward = 0f
                yaw = 0f
            }
            if ("explore" !in session.supportedLevels) {
                Text(
                    "Explore stays off until the shared arbiter lists it. That level arrives with frontier exploration.",
                    style = MaterialTheme.typography.bodySmall,
                )
            }
            if (session.proposalText.isNotEmpty()) {
                Text(session.proposalText)
                Row(horizontalArrangement = Arrangement.spacedBy(8.dp)) {
                    Button(onClick = { session.decideProposal(true) }) { Text("Approve search target") }
                    OutlinedButton(onClick = { session.decideProposal(false) }) { Text("Reject") }
                }
            }
            Row(horizontalArrangement = Arrangement.spacedBy(8.dp)) {
                OutlinedButton(onClick = {
                    forward = 0f
                    yaw = 0f
                    session.setAutonomy("teleop")
                }) { Text("Take over") }
                OutlinedButton(onClick = {
                    forward = 0f
                    yaw = 0f
                    session.emergencyStop(false)
                }) { Text("Emergency stop") }
            }
            Row(horizontalArrangement = Arrangement.spacedBy(8.dp)) {
                OutlinedButton(onClick = { session.emergencyStop(true) }) { Text("Reset stop") }
                OutlinedButton(onClick = { session.finishLog() }) { Text("Finish run log") }
            }
        }

        SectionCard("Waypoint") {
            CoordField("Origin latitude", originLat, !session.waypointActive) { originLat = it }
            CoordField("Origin longitude", originLon, !session.waypointActive) { originLon = it }
            CoordField("Goal latitude", goalLat, !session.waypointActive) { goalLat = it }
            CoordField("Goal longitude", goalLon, !session.waypointActive) { goalLon = it }
            CoordField("Token", token, !session.waypointActive) { token = it }
            OutlinedButton(
                onClick = {
                    originLat = "38.8297"
                    originLon = "-77.3075"
                    goalLat = "38.82981"
                    goalLon = "-77.3075"
                    token = "gmu-north"
                },
                enabled = !session.waypointActive,
            ) { Text("Johnson Center, 12 m north") }
            Readout("Follower", session.waypointStatus)
            if (session.waypointActive) Readout("Distance", "%.1f m".format(session.waypointDistance))
            Button(
                onClick = {
                    val originLatitude = originLat.toDoubleOrNull()
                    val originLongitude = originLon.toDoubleOrNull()
                    val latitude = goalLat.toDoubleOrNull()
                    val longitude = goalLon.toDoubleOrNull()
                    if (originLatitude == null || originLongitude == null || latitude == null || longitude == null ||
                        originLatitude !in -85.0..85.0 || latitude !in -85.0..85.0 ||
                        originLongitude !in -180.0..180.0 || longitude !in -180.0..180.0
                    ) {
                        entryNote = "Enter latitude from −85 to 85 and longitude from −180 to 180."
                        return@Button
                    }
                    entryNote = ""
                    session.engageWaypoint(originLatitude, originLongitude, latitude, longitude, token, REACH_METRES)
                },
                enabled = !session.waypointActive,
            ) { Text("Go to waypoint") }
            OutlinedButton(
                onClick = { session.cancelWaypoint() },
                enabled = session.waypointActive,
            ) { Text("Cancel waypoint") }
            if (entryNote.isNotEmpty()) {
                Text(entryNote, color = MaterialTheme.colorScheme.error, style = MaterialTheme.typography.bodySmall)
            }
            Text(
                "Go runs MobileWaypoint on the phone and publishes that twist. On Zorvane the pose is the depth frame’s body position, and the goal topic stays idle so the simulator does not run a second follower.",
                style = MaterialTheme.typography.bodySmall,
            )
        }

        SectionCard("Shared Rust test") {
            Button(onClick = { session.runBenchmark() }) { Text("Run velocity benchmark") }
            Text(session.benchmark, style = MaterialTheme.typography.bodySmall)
        }

        SectionCard("Phone mounting") {
            Text(
                "Default: phone flat, screen up, top edge toward the rover’s front. Phone and rover origins are assumed coincident. Calibrate the mount before connecting motor hardware. Tracking loss commands neutral effort. Leaving the app stops the controller.",
                style = MaterialTheme.typography.bodySmall,
            )
        }
    }
}

@OptIn(ExperimentalLayoutApi::class)
@Composable
private fun LevelPicker(session: TerraSession, onSelect: () -> Unit) {
    FlowRow(horizontalArrangement = Arrangement.spacedBy(8.dp)) {
        session.supportedLevels.forEach { level ->
            FilterChip(
                selected = session.autonomyLevel == level,
                onClick = {
                    onSelect()
                    session.setAutonomy(level)
                },
                label = { Text(levelLabels[level] ?: level) },
            )
        }
        if ("explore" !in session.supportedLevels) {
            FilterChip(selected = false, onClick = {}, enabled = false, label = { Text("Explore") })
        }
    }
}

@Composable
private fun SectionCard(title: String, content: @Composable () -> Unit) {
    Card(Modifier.fillMaxWidth()) {
        Column(Modifier.padding(12.dp), verticalArrangement = Arrangement.spacedBy(8.dp)) {
            Text(title, style = MaterialTheme.typography.titleMedium)
            content()
        }
    }
}

@Composable
private fun Readout(label: String, value: String) {
    Row(Modifier.fillMaxWidth(), horizontalArrangement = Arrangement.SpaceBetween) {
        Text(label)
        Text(value)
    }
}

@Composable
private fun CoordField(label: String, value: String, enabled: Boolean, onChange: (String) -> Unit) {
    OutlinedTextField(
        value = value,
        onValueChange = onChange,
        modifier = Modifier.fillMaxWidth(),
        enabled = enabled,
        label = { Text(label) },
        singleLine = true,
    )
}

private fun snap(value: Float): Float = (round(value / 0.05f) * 0.05f).coerceIn(-1f, 1f)

private fun projectGoal(
    originLat: String,
    originLon: String,
    goalLat: String,
    goalLon: String,
): Pair<Double, Double>? {
    val originLatitude = originLat.toDoubleOrNull() ?: return null
    val originLongitude = originLon.toDoubleOrNull() ?: return null
    val latitude = goalLat.toDoubleOrNull() ?: return null
    val longitude = goalLon.toDoubleOrNull() ?: return null
    val point = try {
        tangentMetres(originLatitude, originLongitude, latitude, longitude)
    } catch (_: Exception) {
        return null
    }
    return point.north to point.west
}
