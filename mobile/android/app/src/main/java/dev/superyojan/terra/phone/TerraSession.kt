package dev.superyojan.terra.phone

import android.content.Context
import android.os.Handler
import android.os.HandlerThread
import android.os.Looper
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.setValue
import dev.superyojan.terra.core.ImuReading
import dev.superyojan.terra.core.MappingDepthFrame
import dev.superyojan.terra.core.MobileController
import dev.superyojan.terra.core.MobileOccupancyMap
import dev.superyojan.terra.core.MobileWaypoint
import dev.superyojan.terra.core.MobileZenohClient
import dev.superyojan.terra.core.OccupancyGrid
import dev.superyojan.terra.core.PoseVelocityFilter
import dev.superyojan.terra.core.SafetyState
import dev.superyojan.terra.core.SimulatedPlant
import dev.superyojan.terra.core.TwistSetpoint
import dev.superyojan.terra.core.VioReading
import dev.superyojan.terra.core.WaypointPhase
import dev.superyojan.terra.core.bodyYaw
import dev.superyojan.terra.core.defaultControlSettings
import dev.superyojan.terra.core.defaultOccupancySettings
import dev.superyojan.terra.core.defaultWaypointSettings
import dev.superyojan.terra.core.runVelocityBenchmark
import dev.superyojan.terra.core.simulatedRoomDepth
import dev.superyojan.terra.core.supportedAutonomyLevels
import org.json.JSONObject
import java.io.File
import java.util.UUID
import java.util.concurrent.CountDownLatch
import java.util.concurrent.TimeUnit

private enum class DriveMode { Stopped, Simulation, Phone, Remote }

/**
 * Thin shell over `terra-mobile`. Estimation, the plant, the waypoint follower,
 * occupancy integration, and Zenoh framing all stay in Rust.
 */
class TerraSession(context: Context) {
    private val appContext = context.applicationContext
    private val prefs = appContext.getSharedPreferences("terra-phone", Context.MODE_PRIVATE)
    private val thread = HandlerThread("terra.control").also { it.start() }
    private val control = Handler(thread.looper)
    private val main = Handler(Looper.getMainLooper())

    val emulator = isEmulator()
    val supportedLevels: List<String> = supportedAutonomyLevels()

    var source by mutableStateOf("Stopped")
    var status by mutableStateOf("Motor output disabled")
    var measuredForward by mutableStateOf(0.0)
    var measuredYaw by mutableStateOf(0.0)
    var leftEffort by mutableStateOf(0.0)
    var rightEffort by mutableStateOf(0.0)
    var benchmark by mutableStateOf("Tests feedback control under load and saturation.")
    var zenohStatus by mutableStateOf("Disconnected")
    var zenohConnecting by mutableStateOf(false)
    var waypointActive by mutableStateOf(false)
    var waypointStatus by mutableStateOf("No goal")
    var waypointDistance by mutableStateOf(0.0)
    var mapStatus by mutableStateOf("Start Simulated rover to build a map")
    var occupancy by mutableStateOf<OccupancyGrid?>(null)
    var mapX by mutableStateOf(0.0)
    var mapY by mutableStateOf(0.0)
    var mapYaw by mutableStateOf(0.0)
    var autonomyLevel by mutableStateOf(supportedLevels.firstOrNull() ?: "teleop")
    var autonomyReason by mutableStateOf("Idle")
    var proposalText by mutableStateOf("")
    var proposalId by mutableStateOf<ULong?>(null)
    var depthNote by mutableStateOf(
        "Phone mapping uses ARCore Depth when the device supports it. " +
            "Initial camera height is assumed 0.5 m above flat ground. " +
            "Zorvane mode uses the simulator exposure pose, with ground at robotics Z = 0.",
    )

    private var controller: MobileController? = null
    private var follower: MobileWaypoint? = null
    private var map: MobileOccupancyMap? = null
    private var zenoh: MobileZenohClient? = null
    private val plant = SimulatedPlant()
    private val velocityFilter = PoseVelocityFilter()
    private var sensors: PhoneSensors? = null
    private var mode = DriveMode.Stopped
    private var forward = 0.0
    private var yaw = 0.0
    private var goalLatched = false
    private var looping = false
    private var tick = 0
    private var simulatedTime = 0.0
    private var remotePose: Pose? = null
    private var phonePose: Pose? = null
    private var proposalRun = ""
    private var generation = 0
    private val loop = object : Runnable {
        override fun run() {
            if (!looping) return
            try {
                step()
            } catch (error: Exception) {
                fail(error)
                return
            }
            control.postDelayed(this, 10)
        }
    }

    fun discardZenohOnDevice() {
        if (!emulator) {
            prefs.edit().remove(ENDPOINT_KEY).remove(ROVER_KEY).apply()
        }
    }

    fun savedEndpoint(): String = if (emulator) {
        prefs.getString(ENDPOINT_KEY, null) ?: "tcp/10.0.2.2:7447"
    } else {
        "tcp/10.0.2.2:7447"
    }

    fun savedRoverId(): String = if (emulator) prefs.getString(ROVER_KEY, null) ?: "0" else "0"

    fun setTarget(forward: Double, yaw: Double) {
        control.post {
            if (!goalLatched) {
                this.forward = forward
                this.yaw = yaw
            }
        }
    }

    fun startSimulation() {
        stop()
        control.post {
            try {
                prepare(DriveMode.Simulation)
                beginLoop()
                publish { source = "Simulated motor plant" }
            } catch (error: Exception) {
                fail(error)
            }
        }
    }

    fun startPhone() {
        stop()
        control.post {
            val epoch = generation
            val phone = PhoneSensors(
                appContext,
                control,
                onImu = { sample ->
                    if (epoch != generation || mode != DriveMode.Phone) return@PhoneSensors
                    try {
                        controller?.pushImu(sample)
                    } catch (error: Exception) {
                        fail(error)
                    }
                },
                onCamera = { camera ->
                    if (epoch != generation || mode != DriveMode.Phone) return@PhoneSensors
                    acceptCamera(camera)
                },
                onDepth = { frame ->
                    if (epoch != generation || mode != DriveMode.Phone) return@PhoneSensors
                    try {
                        val grid = map ?: return@PhoneSensors
                        grid.recenter(frame.cameraX, frame.cameraY)
                        grid.integrateDepth(frame)
                        val yaw = phonePose?.yaw ?: 0.0
                        showMap(grid.snapshot(), frame.cameraX, frame.cameraY, yaw, "ARCore depth · 10 Hz")
                    } catch (error: Exception) {
                        publish { mapStatus = "Map: ${error.message}" }
                    }
                },
            )
            if (!phone.start()) {
                publish {
                    source = "Stopped"
                    status = phone.status
                    mapStatus = phone.depthMessage()
                }
                phone.close()
                return@post
            }
            try {
                prepare(DriveMode.Phone)
                sensors = phone
                beginLoop()
                publish {
                    source = "SensorManager + ARCore"
                    mapStatus = phone.depthMessage()
                }
            } catch (error: Exception) {
                phone.close()
                fail(error)
            }
        }
    }

    fun startZenoh(endpoint: String, roverId: String) {
        if (!emulator) return
        val id = roverId.trim().toULongOrNull()
        if (id == null) {
            zenohStatus = "Rover ID must be a nonnegative integer"
            return
        }
        if (zenohConnecting) return
        zenohConnecting = true
        stop()
        // stop() publishes "Disconnected" onto the main looper. Queue the
        // connecting status after that post so it is not overwritten.
        main.post {
            zenohConnecting = true
            zenohStatus = "Connecting…"
        }
        val trimmed = endpoint.trim()
        prefs.edit().putString(ENDPOINT_KEY, trimmed).putString(ROVER_KEY, id.toString()).apply()
        control.post {
            try {
                map = MobileOccupancyMap(defaultOccupancySettings())
                plant.reset()
                zenoh = MobileZenohClient(trimmed, "terra/rover", id)
                forward = 0.0
                yaw = 0.0
                tick = 0
                mode = DriveMode.Remote
                remotePose = null
                beginLoop()
                publish {
                    zenohConnecting = false
                    source = "Zorvane rover $id over Zenoh"
                    status = "Remote velocity targets · local feedback unavailable"
                    zenohStatus = "Zenoh session open · waiting for depth"
                    occupancy = null
                    mapX = 0.0
                    mapY = 0.0
                    mapYaw = 0.0
                    mapStatus = "Waiting for simulator depth and exposure pose"
                }
            } catch (error: Exception) {
                fail(error)
            }
        }
    }

    /** Leaving the foreground and Stop both end here. Disconnect publishes a final zero. */
    fun stop() {
        generation += 1
        if (Looper.myLooper() == thread.looper) {
            stopOnControl()
            return
        }
        val done = CountDownLatch(1)
        control.post {
            try {
                stopOnControl()
            } finally {
                done.countDown()
            }
        }
        done.await(2, TimeUnit.SECONDS)
    }

    fun engageWaypoint(
        originLatitude: Double,
        originLongitude: Double,
        latitude: Double,
        longitude: Double,
        token: String,
        halfExtent: Double,
    ) {
        control.post {
            if (mode == DriveMode.Stopped) {
                publish {
                    waypointStatus = if (emulator) {
                        "Start a rover or connect to Zorvane, then go"
                    } else {
                        "Start a rover, then go"
                    }
                }
                return@post
            }
            try {
                val guide = follower ?: MobileWaypoint(defaultWaypointSettings()).also { follower = it }
                guide.setOrigin(originLatitude, originLongitude)
                val accepted = guide.setGoal(
                    latitude,
                    longitude,
                    null,
                    token.trim().ifEmpty { null },
                    halfExtent,
                )
                goalLatched = accepted
                publish {
                    waypointActive = accepted
                    waypointDistance = 0.0
                    waypointStatus = if (accepted) {
                        "Goal latched · waiting for a pose"
                    } else {
                        "That point is outside ${halfExtent.toInt()} m of the origin"
                    }
                }
            } catch (error: Exception) {
                goalLatched = false
                publish {
                    waypointActive = false
                    waypointStatus = error.message ?: "Waypoint rejected"
                }
            }
        }
    }

    fun cancelWaypoint() {
        control.post {
            try {
                follower?.cancel()
            } catch (_: Exception) {
            }
            goalLatched = false
            publish {
                waypointActive = false
                waypointDistance = 0.0
                waypointStatus = "Teleop"
            }
        }
    }

    fun setAutonomy(level: String) {
        control.post {
            if (level !in supportedLevels) return@post
            forward = 0.0
            yaw = 0.0
            val payload = JSONObject()
                .put("level", level)
                .put("token", UUID.randomUUID().toString())
                .toString()
            try {
                if (mode == DriveMode.Remote) {
                    zenoh?.sendAction("autonomy", payload)
                } else {
                    val now = if (mode == DriveMode.Simulation) simulatedTime else PhoneSensors.monotonicSeconds()
                    controller?.autonomyRequest("autonomy", payload, now)
                }
            } catch (error: Exception) {
                publish { autonomyReason = error.message ?: "Autonomy request failed" }
            }
        }
    }

    fun emergencyStop(reset: Boolean) {
        control.post {
            forward = 0.0
            yaw = 0.0
            goalLatched = false
            try {
                follower?.cancel()
            } catch (_: Exception) {
            }
            val payload = JSONObject()
                .put("action", if (reset) "reset" else "stop")
                .put("token", UUID.randomUUID().toString())
                .toString()
            try {
                if (mode == DriveMode.Remote) {
                    zenoh?.sendAction("safety", payload)
                } else {
                    val now = if (mode == DriveMode.Simulation) simulatedTime else PhoneSensors.monotonicSeconds()
                    controller?.autonomyRequest("safety", payload, now)
                }
            } catch (error: Exception) {
                publish { autonomyReason = error.message ?: "Stop failed" }
            }
            publish {
                waypointActive = false
                waypointStatus = if (reset) "Stop reset" else "Emergency stop"
            }
        }
    }

    fun decideProposal(approve: Boolean) {
        val id = proposalId ?: return
        val run = proposalRun
        control.post {
            val payload = JSONObject()
                .put("run_id", run)
                .put("proposal_id", id.toLong())
                .put("decision", if (approve) "approve" else "reject")
                .put("token", UUID.randomUUID().toString())
                .toString()
            try {
                if (mode == DriveMode.Remote) {
                    zenoh?.sendAction("goal/decision", payload)
                } else {
                    val now = if (mode == DriveMode.Simulation) simulatedTime else PhoneSensors.monotonicSeconds()
                    controller?.autonomyRequest("goal/decision", payload, now)
                }
            } catch (error: Exception) {
                publish { autonomyReason = error.message ?: "Decision failed" }
            }
        }
    }

    fun finishLog() {
        control.post {
            try {
                controller?.endRecording()
            } catch (error: Exception) {
                publish { autonomyReason = error.message ?: "Could not close the run log" }
            }
        }
    }

    fun clearMap() {
        control.post {
            try {
                map?.clear()
                publish { occupancy = null }
            } catch (error: Exception) {
                publish { mapStatus = error.message ?: "Could not clear the map" }
            }
        }
    }

    fun runBenchmark() {
        control.post {
            try {
                val result = runVelocityBenchmark()
                publish {
                    benchmark = "%s · speed error %.3f m/s · turn error %.3f rad/s".format(
                        if (result.passed) "Passed" else "Failed",
                        result.forwardError,
                        result.yawError,
                    )
                }
            } catch (error: Exception) {
                fail(error)
            }
        }
    }

    fun close() {
        stop()
        thread.quitSafely()
    }

    private fun acceptCamera(camera: CameraObservation) {
        try {
            val velocity = velocityFilter.push(camera.x, camera.y, camera.z, camera.timestamp, camera.tracked)
            val brain = controller ?: return
            brain.pushVio(
                VioReading(
                    timestamp = camera.timestamp,
                    positionX = camera.x,
                    positionY = camera.y,
                    positionZ = camera.z,
                    quaternionX = camera.quaternionX,
                    quaternionY = camera.quaternionY,
                    quaternionZ = camera.quaternionZ,
                    quaternionW = camera.quaternionW,
                    velocityX = velocity?.x ?: 0.0,
                    velocityY = velocity?.y ?: 0.0,
                    velocityZ = velocity?.z ?: 0.0,
                    tracked = camera.tracked && velocity != null,
                ),
            )
            if (camera.tracked) {
                phonePose = Pose(
                    camera.x,
                    camera.y,
                    bodyYaw(camera.quaternionX, camera.quaternionY, camera.quaternionZ, camera.quaternionW),
                )
            } else {
                phonePose = null
                publish { mapStatus = "Tracking lost · map paused" }
            }
        } catch (error: Exception) {
            fail(error)
        }
    }

    private fun prepare(next: DriveMode) {
        val brain = MobileController(defaultControlSettings())
        val runId = UUID.randomUUID().toString()
        val log = File(appContext.filesDir, "Terra-run-$runId.jsonl")
        brain.beginRecording(log.absolutePath, runId)
        controller = brain
        map = MobileOccupancyMap(defaultOccupancySettings())
        plant.reset()
        velocityFilter.reset()
        mode = next
        forward = 0.0
        yaw = 0.0
        tick = 0
        simulatedTime = 0.0
        phonePose = null
        remotePose = null
        publish {
            occupancy = null
            mapX = 0.0
            mapY = 0.0
            mapYaw = 0.0
            mapStatus = if (next == DriveMode.Simulation) "Simulated depth · 10 Hz" else "Waiting for scene depth"
        }
    }

    private fun beginLoop() {
        looping = true
        control.removeCallbacks(loop)
        control.post(loop)
    }

    private fun step() {
        if (mode == DriveMode.Remote) {
            val link = zenoh ?: return
            link.takeDepth()?.let { frame ->
                remotePose = Pose(frame.bodyX, frame.bodyY, frame.bodyYaw)
                val grid = map ?: return@let
                grid.recenter(frame.bodyX, frame.bodyY)
                grid.integrateDepth(
                    MappingDepthFrame(
                        timestamp = frame.timestamp,
                        width = frame.width,
                        height = frame.height,
                        fx = frame.fx,
                        fy = frame.fy,
                        cx = frame.cx,
                        cy = frame.cy,
                        cameraX = frame.cameraX,
                        cameraY = frame.cameraY,
                        cameraZ = frame.cameraZ,
                        quaternionX = frame.quaternionX,
                        quaternionY = frame.quaternionY,
                        quaternionZ = frame.quaternionZ,
                        quaternionW = frame.quaternionW,
                        depthMetres = frame.depthMetres,
                    ),
                )
                showMap(grid.snapshot(), frame.bodyX, frame.bodyY, frame.bodyYaw, "Simulator depth · exposure pose")
            }
            val command = commandFor(remotePose)
            link.setTarget(command.forward, command.yaw)
            tick += 1
            if (tick % 10 == 0) {
                val text = link.status()
                publish { zenohStatus = text }
                readAuthority(link.autonomyStatus())
            }
            return
        }
        val brain = controller ?: return
        if (mode == DriveMode.Stopped) return
        if (mode == DriveMode.Phone) sensors?.poll()
        val now = if (mode == DriveMode.Simulation) simulatedTime else PhoneSensors.monotonicSeconds()
        val pose = if (mode == DriveMode.Simulation) plant.state() else null
        if (pose != null) {
            brain.pushImu(
                ImuReading(
                    timestamp = now,
                    accelerationForward = pose.accelerationForward,
                    accelerationLeft = pose.forward * pose.yawRate,
                    accelerationUp = 0.0,
                    gyroRoll = 0.0,
                    gyroPitch = 0.0,
                    gyroYaw = pose.yawRate,
                ),
            )
            if (tick % 5 == 0) {
                brain.pushVio(
                    VioReading(
                        timestamp = now,
                        positionX = pose.x,
                        positionY = pose.y,
                        positionZ = 0.0,
                        quaternionX = pose.quaternionX,
                        quaternionY = pose.quaternionY,
                        quaternionZ = pose.quaternionZ,
                        quaternionW = pose.quaternionW,
                        velocityX = pose.velocityX,
                        velocityY = pose.velocityY,
                        velocityZ = 0.0,
                        tracked = true,
                    ),
                )
            }
        }
        val current = when (mode) {
            DriveMode.Simulation -> pose?.let { Pose(it.x, it.y, it.yaw) }
            DriveMode.Phone -> phonePose
            else -> null
        }
        val command = commandFor(current)
        brain.setTarget(TwistSetpoint(timestamp = now, forward = command.forward, yawRate = command.yaw))
        val output = brain.step(now)
        if (mode == DriveMode.Simulation && pose != null) {
            if (tick % 10 == 0) integrateRoom(pose.x, pose.y, pose.yaw, now)
            plant.step(output.leftEffort, output.rightEffort)
            simulatedTime += 0.01
        }
        tick += 1
        if (tick % 10 == 0) {
            publish {
                measuredForward = output.estimatedForward
                measuredYaw = output.estimatedYawRate
                leftEffort = output.leftEffort
                rightEffort = output.rightEffort
                status = when (output.safety) {
                    SafetyState.ACTIVE -> "Feedback control active"
                    SafetyState.SENSOR_NOT_READY -> "Waiting for IMU and VIO"
                    SafetyState.TRACKING_LOST -> "Tracking lost · neutral output"
                    SafetyState.STALE_SENSORS -> "Stale sensors · neutral output"
                    SafetyState.STALE_TARGET -> "Target expired · neutral output"
                    SafetyState.INVALID_TIME -> "Clock mismatch · neutral output"
                }
            }
            readAuthority(brain.autonomyStatus())
        }
    }

    private fun commandFor(pose: Pose?): Twist {
        if (!goalLatched) return Twist(forward, yaw)
        val guide = follower ?: return Twist(0.0, 0.0)
        val here = pose ?: return Twist(0.0, 0.0)
        val step = guide.step(here.x, here.y, here.yaw)
        if (step.phase != WaypointPhase.ACTIVE) {
            goalLatched = false
        }
        if (tick % 10 == 0 || step.phase != WaypointPhase.ACTIVE) {
            publish {
                waypointActive = step.phase == WaypointPhase.ACTIVE
                waypointDistance = step.distance
                waypointStatus = when (step.phase) {
                    WaypointPhase.IDLE -> "Teleop"
                    WaypointPhase.ACTIVE -> "Steering · %.1f m".format(step.distance)
                    WaypointPhase.ARRIVED -> "Arrived"
                }
            }
        }
        return if (step.phase == WaypointPhase.ACTIVE) {
            Twist(step.forward, step.yawRate)
        } else {
            Twist(forward, yaw)
        }
    }

    private fun integrateRoom(x: Double, y: Double, yaw: Double, timestamp: Double) {
        val grid = map ?: return
        val frame = simulatedRoomDepth(x, y, yaw)
        grid.recenter(x, y)
        grid.integrateDepth(
            MappingDepthFrame(
                timestamp = timestamp,
                width = frame.width,
                height = frame.height,
                fx = frame.fx,
                fy = frame.fy,
                cx = frame.cx,
                cy = frame.cy,
                cameraX = frame.cameraX,
                cameraY = frame.cameraY,
                cameraZ = frame.cameraZ,
                quaternionX = frame.quaternionX,
                quaternionY = frame.quaternionY,
                quaternionZ = frame.quaternionZ,
                quaternionW = frame.quaternionW,
                depthMetres = frame.depthMetres,
            ),
        )
        showMap(grid.snapshot(), x, y, yaw, "Simulated depth · 10 Hz")
    }

    private fun showMap(grid: OccupancyGrid, x: Double, y: Double, yaw: Double, text: String) {
        publish {
            occupancy = grid
            mapX = x
            mapY = y
            mapYaw = yaw
            mapStatus = text
        }
    }

    private fun readAuthority(raw: String) {
        if (raw.isBlank()) return
        val objectJson = try {
            JSONObject(raw)
        } catch (_: Exception) {
            return
        }
        val statusJson = objectJson.optJSONObject("status") ?: objectJson
        val level = statusJson.optString("requested_level", autonomyLevel)
        val reason = statusJson.optString("reason", "Unknown")
        val goal = objectJson.optJSONObject("goal")
        val proposal = objectJson.optJSONObject("proposal")
        publish {
            autonomyLevel = level
            autonomyReason = reason
            if (proposal == null || proposal.length() == 0) {
                proposalText = ""
                proposalId = null
                proposalRun = ""
            } else {
                proposalRun = proposal.optString("run_id")
                proposalId = proposal.optLong("proposal_id").toULong()
                proposalText = "Search target %.1f, %.1f m".format(
                    proposal.optDouble("x"),
                    proposal.optDouble("y"),
                )
            }
            if (!goalLatched && goal != null && goal.optString("state") == "active") {
                waypointActive = true
                waypointDistance = goal.optDouble("distance", waypointDistance)
                waypointStatus = "active"
            }
        }
    }

    private fun stopOnControl() {
        looping = false
        control.removeCallbacks(loop)
        sensors?.close()
        sensors = null
        mode = DriveMode.Stopped
        forward = 0.0
        yaw = 0.0
        goalLatched = false
        remotePose = null
        phonePose = null
        zenoh?.disconnect()
        zenoh = null
        try {
            controller?.reset()
        } catch (_: Exception) {
        }
        controller = null
        publish {
            source = "Stopped"
            status = "Motor output disabled"
            mapStatus = "Map paused"
            waypointActive = false
            waypointDistance = 0.0
            waypointStatus = "No goal"
            zenohConnecting = false
            zenohStatus = "Disconnected"
            leftEffort = 0.0
            rightEffort = 0.0
            measuredForward = 0.0
            measuredYaw = 0.0
        }
    }

    private fun fail(error: Exception) {
        looping = false
        control.removeCallbacks(loop)
        sensors?.close()
        sensors = null
        mode = DriveMode.Stopped
        goalLatched = false
        zenoh?.disconnect()
        zenoh = null
        try {
            controller?.reset()
        } catch (_: Exception) {
        }
        publish {
            status = error.message ?: "Controller stopped"
            leftEffort = 0.0
            rightEffort = 0.0
            mapStatus = "Map paused"
            zenohConnecting = false
            zenohStatus = error.message ?: "Disconnected"
            waypointActive = false
            waypointStatus = "No goal"
        }
    }

    private fun publish(block: () -> Unit) {
        if (Looper.myLooper() == Looper.getMainLooper()) block() else main.post(block)
    }

    private data class Pose(val x: Double, val y: Double, val yaw: Double)
    private data class Twist(val forward: Double, val yaw: Double)

    companion object {
        private const val ENDPOINT_KEY = "zenohEndpoint"
        private const val ROVER_KEY = "zenohRoverID"
    }
}
