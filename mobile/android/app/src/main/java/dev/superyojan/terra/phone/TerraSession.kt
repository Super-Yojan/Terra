package dev.superyojan.terra.phone

import android.content.Context
import android.os.Handler
import android.os.HandlerThread
import android.os.Looper
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.setValue
import dev.superyojan.terra.core.ConnectionPolicy
import dev.superyojan.terra.core.DriveLoopSnapshot
import dev.superyojan.terra.core.DriveSource
import dev.superyojan.terra.core.ImuReading
import dev.superyojan.terra.core.MappingDepthFrame
import dev.superyojan.terra.core.MobileController
import dev.superyojan.terra.core.MobileOccupancyMap
import dev.superyojan.terra.core.MobileWaypoint
import dev.superyojan.terra.core.MobileZenohClient
import dev.superyojan.terra.core.MotorCommand
import dev.superyojan.terra.core.MotorGate
import dev.superyojan.terra.core.OccupancyGrid
import dev.superyojan.terra.core.OffsetFitCancel
import dev.superyojan.terra.core.OffsetFitOutcome
import dev.superyojan.terra.core.OffsetFitState
import dev.superyojan.terra.core.PhoneDriveLoop
import dev.superyojan.terra.core.PhoneMountCalibration
import dev.superyojan.terra.core.PhonePose
import dev.superyojan.terra.core.PhonePosePipeline
import dev.superyojan.terra.core.PoseAxes
import dev.superyojan.terra.core.RouterSettings
import dev.superyojan.terra.core.SafetyState
import dev.superyojan.terra.core.SimulatedPlant
import dev.superyojan.terra.core.TwistSetpoint
import dev.superyojan.terra.core.VioReading
import dev.superyojan.terra.core.WaypointPhase
import dev.superyojan.terra.core.defaultControlSettings
import dev.superyojan.terra.core.defaultOccupancySettings
import dev.superyojan.terra.core.defaultWaypointSettings
import dev.superyojan.terra.core.mountCalibrationFromJson
import dev.superyojan.terra.core.mountCalibrationToJson
import dev.superyojan.terra.core.mountIsPlausible
import dev.superyojan.terra.core.parseRouterSettings
import dev.superyojan.terra.core.runVelocityBenchmark
import dev.superyojan.terra.core.simulatedRoomDepth
import dev.superyojan.terra.core.supportedAutonomyLevels
import dev.superyojan.terra.core.terraLandscapeMount
import dev.superyojan.terra.core.uncalibratedMount
import org.json.JSONObject
import java.io.File
import java.util.UUID
import java.util.concurrent.CountDownLatch
import java.util.concurrent.TimeUnit

private enum class DriveMode { Stopped, Simulation, Phone, Bluetooth, Remote }

/**
 * Thin shell over `terra-mobile`. Estimation, the plant, the waypoint follower,
 * occupancy integration, and Zenoh framing all stay in Rust.
 *
 * Phone and Bluetooth drive run the Rust [PhoneDriveLoop]: it owns the 100 Hz
 * control tick and the [MotorGate], and the Bluetooth writer takes its frames.
 * This class polls ARCore, forwards the phone stick and link facts, connects
 * ARGOS, and copies a snapshot into Compose state at 10 Hz.
 */
class TerraSession(context: Context) {
    private val appContext = context.applicationContext
    private val prefs = appContext.getSharedPreferences("terra-phone", Context.MODE_PRIVATE)
    private val thread = HandlerThread("terra.control").also { it.start() }
    private val control = Handler(thread.looper)
    private val routerThread = HandlerThread("terra.router").also { it.start() }
    private val router = Handler(routerThread.looper)
    private val main = Handler(Looper.getMainLooper())

    val emulator = isEmulator()
    val ble: TerraBluetooth? = if (emulator) null else TerraBluetooth(appContext)
    val supportedLevels: List<String> = supportedAutonomyLevels()
    /** Mount confirmation and the e-stop latch outlive each drive session. */
    private val gate = MotorGate()
    private val connectionPolicy = ConnectionPolicy()
    private val pipeline = PhonePosePipeline(loadMount(), PoseAxes.DEVICE, 0.0)

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
        "Phone mapping uses ARCore raw depth with its confidence when the device supports it. " +
            "The floor sits the mount's camera height below the phone, or 0.5 m without a calibration. " +
            "Zorvane mode uses the simulator exposure pose, with ground at robotics Z = 0.",
    )
    var routerStatus by mutableStateOf("Disconnected")
    var routerConnecting by mutableStateOf(false)
    var routerConnected by mutableStateOf(false)
    var motorStatus by mutableStateOf("Motors idle")
    var emergencyLatched by mutableStateOf(false)
    var mount by mutableStateOf(pipeline.calibration())
    var mountConfirmed by mutableStateOf(false)
    var mountStatus by mutableStateOf("")
    var fitStatus by mutableStateOf("No turn pass running")
    var fitCollecting by mutableStateOf(false)
    var runLog by mutableStateOf<File?>(null)
    /** Bluetooth drive: the stick is normalized effort instead of m/s and rad/s. */
    var normalizedStick by mutableStateOf(false)

    @Volatile private var controller: MobileController? = null
    @Volatile private var driveLoop: PhoneDriveLoop? = null
    private var map: MobileOccupancyMap? = null
    private var follower: MobileWaypoint? = null
    private var zenoh: MobileZenohClient? = null
    private val plant = SimulatedPlant()
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
    private var syncedLayout: String? = null
    private var routerAttempt = 0
    private var routerRetryAt: Double? = null
    /** Operator disconnected or never connected: no automatic retry. */
    private var routerSuppressed = true
    private var routerUser = ""
    private var routerPassword = ""
    private var routerSettings: RouterSettings? = null
    private var routerBusy = false
    private var handledLosses = 0u
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

    init {
        // The Bluetooth drive tick writes whatever the Rust gate decided last.
        ble?.setFrameSource { driveLoop?.takeMotorCommand(FRAME_MAX_AGE_S) ?: MotorCommand.Idle }
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
        stop(keepBluetooth = true)
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
        stop(keepBluetooth = true)
        control.post {
            val epoch = generation
            val phone = PhoneSensors(
                appContext,
                control,
                pipeline,
                mountHeight = { mountHeight() },
                onImu = { sample ->
                    if (epoch != generation || mode != DriveMode.Phone) return@PhoneSensors
                    try {
                        controller?.pushImu(sample)
                    } catch (error: Exception) {
                        fail(error)
                    }
                },
                onPose = { pose ->
                    if (epoch != generation || mode != DriveMode.Phone) return@PhoneSensors
                    acceptPose(pose)
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
                startDriveLoop(DriveSource.PHONE)
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

    /** Bluetooth effort through the Rust gate with no phone sensors: the phone stick is normalized effort. */
    fun startBluetoothDrive() {
        stop(keepBluetooth = true)
        control.post {
            try {
                prepare(DriveMode.Bluetooth)
                startDriveLoop(DriveSource.BLUETOOTH_MANUAL)
                beginLoop()
                publish {
                    source = "Bluetooth drive · no phone sensors"
                    mapStatus = "No map without phone sensors"
                }
            } catch (error: Exception) {
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

    fun armBluetooth() {
        ble?.arm()
    }

    fun disarmBluetooth() {
        ble?.disarm()
    }

    fun scanBluetooth() {
        ble?.startScan()
    }

    /**
     * Leaving the foreground and Stop both end here. Disconnect publishes a final
     * zero. Switching drive modes keeps the Bluetooth link but disarms it.
     */
    fun stop(keepBluetooth: Boolean = false) {
        if (keepBluetooth) ble?.disarm() else ble?.disarmAndDisconnect()
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
                .apply { if (level == "explore") put("budget_seconds", 120.0) }
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
        if (!reset) ble?.disarm()
        control.post {
            forward = 0.0
            yaw = 0.0
            goalLatched = false
            try {
                if (reset) {
                    gate.clearEmergencyStop()
                    gate.clearStopReason()
                } else {
                    gate.emergencyStop(PhoneSensors.monotonicSeconds())
                }
            } catch (_: Exception) {
            }
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

    /** Close the run log so [runLog] can be shared; the controller keeps running unrecorded. */
    fun finishLog() {
        control.post {
            try {
                controller?.endRecording()
                publish { autonomyReason = "Run log closed · share it below" }
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
        ble?.setFrameSource(null)
        ble?.close()
        thread.quitSafely()
        routerThread.quitSafely()
    }

    private fun acceptPose(pose: PhonePose) {
        try {
            controller?.pushVio(pose.vio)
            driveLoop?.setTracking(pose.tracked)
            if (pose.tracked) {
                phonePose = Pose(pose.vio.positionX, pose.vio.positionY, pose.yaw)
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
        val log = File(File(appContext.filesDir, RUN_LOG_DIR).also { it.mkdirs() }, "Terra-run-$runId.jsonl")
        brain.beginRecording(log.absolutePath, runId)
        val grid = MobileOccupancyMap(defaultOccupancySettings())
        // The planner and ARGOS read the same map the phone integrates.
        brain.attachOccupancyMap(grid)
        controller = brain
        map = grid
        syncedLayout = null
        plant.reset()
        mode = next
        forward = 0.0
        yaw = 0.0
        tick = 0
        simulatedTime = 0.0
        phonePose = null
        remotePose = null
        publish {
            runLog = log
            normalizedStick = next == DriveMode.Bluetooth
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
        driveLoop?.let { drive ->
            // The Rust loop ticks on its own thread; this thread feeds it.
            if (mode == DriveMode.Phone) sensors?.poll()
            val command = commandFor(phonePose)
            drive.setLocalInput(command.forward, command.yaw)
            tick += 1
            if (tick % 10 == 0) serviceDrive(brain, drive)
            return
        }
        val now = simulatedTime
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
        val command = commandFor(pose?.let { Pose(it.x, it.y, it.yaw) })
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
                status = safetyText(output.safety)
            }
            readAuthority(brain.autonomyStatus())
        }
    }

    private fun startDriveLoop(source: DriveSource) {
        val brain = controller ?: return
        gate.reset()
        val drive = PhoneDriveLoop(brain, gate, source, PhoneSensors.monotonicSeconds())
        drive.start()
        driveLoop = drive
        handledLosses = 0u
        if (!routerSuppressed) routerRetryAt = 0.0
    }

    private fun stopDriveLoop() {
        driveLoop?.stop()
        driveLoop = null
    }

    /** 10 Hz on the sensor thread: layout and link facts in, snapshot out. */
    private fun serviceDrive(brain: MobileController, drive: PhoneDriveLoop) {
        val link = ble
        val layout = link?.driveLayout
        if (layout != syncedLayout) {
            syncedLayout = layout
            try {
                brain.setActuatorLayout(layout ?: "")
            } catch (error: Exception) {
                publish { motorStatus = "Rover layout rejected: ${error.message}" }
            }
        }
        val connected = link?.connected == true
        val armed = link?.ready == true && link.armed
        val routes = brain.layoutSupportsVioControl()
        drive.setLink(connected, armed, mode == DriveMode.Phone, routes)
        val snapshot = drive.snapshot()
        snapshot.error?.let { throw IllegalStateException(it) }
        serviceRouter(snapshot)
        serviceFit()
        val output = snapshot.output
        val note = when {
            snapshot.emergencyStopped -> "Emergency stop latched · Reset stop to drive"
            !connected -> "No rover link"
            !armed -> "Arm the rover to drive"
            mode == DriveMode.Phone && !routes -> "Phone drive needs left and right wheel routes in the rover layout"
            snapshot.stopReason != null -> "Held · ${snapshot.stopReason}"
            else -> "Driving through the Rust motor gate"
        }
        publish {
            motorStatus = note
            emergencyLatched = snapshot.emergencyStopped
            if (output != null) {
                measuredForward = output.estimatedForward
                measuredYaw = output.estimatedYawRate
                leftEffort = output.leftEffort
                rightEffort = output.rightEffort
                status = safetyText(output.safety)
            }
        }
        readAuthority(brain.autonomyStatus())
    }

    private fun safetyText(safety: SafetyState): String = when (safety) {
        SafetyState.ACTIVE -> "Feedback control active"
        SafetyState.SENSOR_NOT_READY -> "Waiting for IMU and VIO"
        SafetyState.TRACKING_LOST -> "Tracking lost · neutral output"
        SafetyState.STALE_SENSORS -> "Stale sensors · neutral output"
        SafetyState.STALE_TARGET -> "Target expired · neutral output"
        SafetyState.INVALID_TIME -> "Clock mismatch · neutral output"
    }

    /** ARGOS link on a phone. The controller holds the session; Rust gates every motor frame. */
    fun connectRouter(endpoint: String, prefix: String, roverId: String, user: String, password: String) {
        val settings = parseRouterSettings(endpoint, prefix, roverId)
        if (settings == null) {
            routerStatus = "Enter tcp/ or tls/host:port, a topic prefix, and a numeric rover ID"
            return
        }
        prefs.edit()
            .putString(ROUTER_ENDPOINT_KEY, settings.endpoint)
            .putString(ROUTER_PREFIX_KEY, settings.prefix)
            .putString(ROUTER_ROVER_KEY, settings.roverId.toString())
            .putString(ROUTER_USER_KEY, user.trim())
            .putString(ROUTER_PASSWORD_KEY, password)
            .apply()
        control.post {
            routerSettings = settings
            routerUser = user.trim()
            routerPassword = password
            routerSuppressed = false
            routerRetryAt = null
            attemptRouter()
        }
    }

    fun disconnectRouter() {
        control.post {
            routerSuppressed = true
            routerRetryAt = null
            routerAttempt += 1
            routerBusy = false
            forward = 0.0
            yaw = 0.0
            try {
                controller?.disconnectDashboard()
            } catch (_: Exception) {
            }
            publish {
                routerConnecting = false
                routerConnected = false
                routerStatus = "Disconnected · remote intent cleared"
            }
        }
    }

    fun savedRouter(): SavedRouter = SavedRouter(
        endpoint = prefs.getString(ROUTER_ENDPOINT_KEY, null) ?: DEFAULT_ROUTER_ENDPOINT,
        prefix = prefs.getString(ROUTER_PREFIX_KEY, null) ?: DEFAULT_ROUTER_PREFIX,
        roverId = prefs.getString(ROUTER_ROVER_KEY, null) ?: "0",
        user = prefs.getString(ROUTER_USER_KEY, null) ?: DEFAULT_ROUTER_USER,
        password = prefs.getString(ROUTER_PASSWORD_KEY, null) ?: "",
    )

    /** Control thread. One attempt at a time; a stale attempt's link is dropped. */
    private fun attemptRouter() {
        val settings = routerSettings ?: return
        val brain = controller
        if (brain == null || driveLoop == null) {
            publish { routerStatus = "Start Phone sensors or Bluetooth drive, then connect" }
            return
        }
        if (routerBusy) return
        routerBusy = true
        routerAttempt += 1
        val attempt = routerAttempt
        val user = routerUser
        val password = routerPassword
        brain.setObservabilityRover(settings.roverId.toString())
        publish {
            routerConnecting = true
            routerStatus = "Connecting…"
        }
        router.post {
            val error = try {
                // Keeps an accepted onboard mission; network setup stays off the sensor thread.
                brain.reconnectDashboardAuthenticated(settings.endpoint, settings.prefix, settings.roverId, user, password)
                null
            } catch (failure: Exception) {
                failure
            }
            control.post {
                routerBusy = false
                if (attempt != routerAttempt || brain !== controller) {
                    if (error == null) runCatching { brain.detachDashboardLink() }
                    return@post
                }
                if (error == null) {
                    runCatching { connectionPolicy.routerConnected() }
                    routerRetryAt = null
                    publish {
                        routerConnecting = false
                        routerConnected = true
                        routerStatus = "Connected · rover ${settings.roverId} · motors still need arm and mount check"
                    }
                } else {
                    routerRetryAt = PhoneSensors.monotonicSeconds() + routerDelay()
                    publish {
                        routerConnecting = false
                        routerConnected = false
                        routerStatus = "${error.message ?: "Router connection failed"} · retrying"
                    }
                }
            }
        }
    }

    private fun routerDelay(): Double = runCatching { connectionPolicy.nextRouterDelay() }.getOrDefault(5.0)

    private fun serviceRouter(snapshot: DriveLoopSnapshot) {
        if (snapshot.linkLosses > handledLosses) {
            handledLosses = snapshot.linkLosses
            forward = 0.0
            yaw = 0.0
            routerRetryAt = PhoneSensors.monotonicSeconds() + routerDelay()
            val keeps = runCatching { controller?.acceptedOnboardMission() == true }.getOrDefault(false)
            publish {
                routerConnected = false
                routerStatus = if (keeps) "Router interrupted · continuing onboard mission" else "Router connection lost · retrying"
            }
        }
        val retry = routerRetryAt ?: return
        if (routerSuppressed || routerBusy || snapshot.dashboard == "connected") return
        if (PhoneSensors.monotonicSeconds() >= retry) {
            routerRetryAt = null
            attemptRouter()
        }
    }

    fun mountHeight(): Double? = try {
        pipeline.calibration().takeIf { mountIsPlausible(it) }?.cameraHeight
    } catch (_: Exception) {
        null
    }

    /** Landscape on Terra, screen toward the rover, 0.23 m camera height. */
    fun applyTerraMount(topEdgeLeft: Boolean) = saveMount(terraLandscapeMount(topEdgeLeft))

    fun saveMount(calibration: PhoneMountCalibration) {
        control.post {
            val text = try {
                pipeline.setCalibration(calibration)
                prefs.edit().putString(MOUNT_KEY, mountCalibrationToJson(calibration)).apply()
                publish { mount = calibration }
                "Saved. Restart Phone sensors so the map floor uses the new height."
            } catch (error: Exception) {
                error.message ?: "Mount rejected"
            }
            publish { mountStatus = text }
        }
    }

    /** The person checked the phone is in its mount. Motors stay held until then. */
    fun confirmMount(confirmed: Boolean) {
        mountConfirmed = confirmed
        control.post {
            try {
                gate.setMountConfirmed(confirmed)
                if (confirmed) gate.clearStopReason()
            } catch (_: Exception) {
            }
        }
    }

    /** Turn the rover in place at least 90°, then [finishOffsetFit]; twice, in opposite directions. */
    fun beginOffsetFit() {
        control.post {
            val text = try {
                pipeline.beginOffsetFit()
                "Collecting · turn the rover in place"
            } catch (error: Exception) {
                error.message ?: "Could not start the turn pass"
            }
            publish { fitStatus = text }
        }
    }

    fun finishOffsetFit() {
        control.post {
            val text = try {
                when (val outcome = pipeline.finishOffsetFit()) {
                    OffsetFitOutcome.NotCollecting -> "No turn pass was running"
                    OffsetFitOutcome.Rejected -> "Pass rejected · turn at least 90° in place over 2 s or more"
                    OffsetFitOutcome.Disagreed -> "The two passes differ by over 5 cm · start again"
                    is OffsetFitOutcome.FirstPass ->
                        "First pass %.1f cm RMS · now turn the other way and run a second pass".format(outcome.rms * 100)
                    is OffsetFitOutcome.Accepted -> {
                        val updated = pipeline.calibration().copy(
                            cameraOffsetForward = outcome.forward,
                            cameraOffsetLeft = outcome.left,
                        )
                        pipeline.setCalibration(updated)
                        prefs.edit().putString(MOUNT_KEY, mountCalibrationToJson(updated)).apply()
                        publish { mount = updated }
                        "Offsets saved · %.1f cm forward, %.1f cm left".format(outcome.forward * 100, outcome.left * 100)
                    }
                }
            } catch (error: Exception) {
                error.message ?: "Turn fit failed"
            }
            publish {
                fitCollecting = false
                fitStatus = text
            }
        }
    }

    private fun serviceFit() {
        val state = try {
            pipeline.offsetFitState()
        } catch (_: Exception) {
            return
        }
        when (state) {
            is OffsetFitState.Collecting -> publish {
                fitCollecting = true
                fitStatus = "Collecting · ${state.samples} frames"
            }
            is OffsetFitState.Cancelled -> publish {
                fitCollecting = false
                fitStatus = "Pass cancelled · ${if (state.reason == OffsetFitCancel.TRACKING_LOST) "tracking lost" else "turn took too long"}"
            }
            OffsetFitState.Idle -> publish { fitCollecting = false }
        }
    }

    private fun loadMount(): PhoneMountCalibration {
        val saved = prefs.getString(MOUNT_KEY, null) ?: return uncalibratedMount()
        return try {
            mountCalibrationFromJson(saved)
        } catch (_: Exception) {
            uncalibratedMount()
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
        stopDriveLoop()
        routerBusy = false
        routerAttempt += 1
        try {
            gate.reset()
        } catch (_: Exception) {
        }
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
            routerConnecting = false
            routerConnected = false
            if (!routerSuppressed) routerStatus = "Reconnects when a drive mode starts"
            motorStatus = "Motors idle"
            leftEffort = 0.0
            rightEffort = 0.0
            measuredForward = 0.0
            measuredYaw = 0.0
        }
    }

    private fun fail(error: Exception) {
        looping = false
        control.removeCallbacks(loop)
        stopDriveLoop()
        routerBusy = false
        routerAttempt += 1
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
            routerConnecting = false
            routerConnected = false
            motorStatus = "Motors idle"
            waypointActive = false
            waypointStatus = "No goal"
        }
    }

    private fun publish(block: () -> Unit) {
        if (Looper.myLooper() == Looper.getMainLooper()) block() else main.post(block)
    }

    private data class Pose(val x: Double, val y: Double, val yaw: Double)
    private data class Twist(val forward: Double, val yaw: Double)

    data class SavedRouter(
        val endpoint: String,
        val prefix: String,
        val roverId: String,
        val user: String,
        val password: String,
    )

    companion object {
        private const val ENDPOINT_KEY = "zenohEndpoint"
        private const val ROVER_KEY = "zenohRoverID"
        private const val ROUTER_ENDPOINT_KEY = "dashboardRouterEndpoint"
        private const val ROUTER_PREFIX_KEY = "dashboardTopicPrefix"
        private const val ROUTER_ROVER_KEY = "dashboardRoverID"
        private const val ROUTER_USER_KEY = "dashboardRouterUser"
        // App-private preferences; the iOS app keeps this in the keychain.
        private const val ROUTER_PASSWORD_KEY = "dashboardRouterPassword"
        private const val MOUNT_KEY = "mountCalibration"
        /** Same defaults as the iOS app. */
        const val DEFAULT_ROUTER_ENDPOINT = "tls/argos.34.135.204.210.sslip.io:7448"
        const val DEFAULT_ROUTER_PREFIX = "terra/phone"
        const val DEFAULT_ROUTER_USER = "terra"
        const val RUN_LOG_DIR = "runs"
        /** A Rust frame older than this is not written; the rover lease then stops it. */
        private const val FRAME_MAX_AGE_S = 0.15
    }
}
