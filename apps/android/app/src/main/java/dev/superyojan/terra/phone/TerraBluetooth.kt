package dev.superyojan.terra.phone

import android.annotation.SuppressLint
import android.bluetooth.BluetoothAdapter
import android.bluetooth.BluetoothDevice
import android.bluetooth.BluetoothGatt
import android.bluetooth.BluetoothGattCallback
import android.bluetooth.BluetoothGattCharacteristic
import android.bluetooth.BluetoothGattDescriptor
import android.bluetooth.BluetoothManager
import android.bluetooth.BluetoothProfile
import android.bluetooth.le.ScanCallback
import android.bluetooth.le.ScanFilter
import android.bluetooth.le.ScanResult
import android.bluetooth.le.ScanSettings
import android.content.BroadcastReceiver
import android.content.Context
import android.content.Intent
import android.content.IntentFilter
import android.content.pm.PackageManager
import android.os.Build
import android.os.Handler
import android.os.HandlerThread
import android.os.Looper
import android.os.ParcelUuid
import android.os.SystemClock
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.setValue
import androidx.core.content.ContextCompat
import dev.superyojan.terra.core.ActuatorAssembler
import dev.superyojan.terra.core.MotorCommand
import dev.superyojan.terra.core.actuatorEncodeControl
import dev.superyojan.terra.core.actuatorEncodeFrame
import dev.superyojan.terra.core.actuatorEncodeRelayDrive
import dev.superyojan.terra.core.actuatorFragment
import dev.superyojan.terra.core.actuatorRouteProfiles
import dev.superyojan.terra.core.actuatorRouteStick
import org.json.JSONArray
import org.json.JSONObject
import java.util.UUID
import java.util.concurrent.CountDownLatch
import java.util.concurrent.TimeUnit

data class DiscoveredRover(val address: String, val name: String, val pairing: Boolean)

/**
 * GATT client for a Terra rover.
 *
 * Kotlin owns the radio only. Stick mixing, relay percent pairs, `TA` frames,
 * control envelopes, and fragment reassembly come from `terra-mobile`.
 * The current rover service speaks streamed-relay pairs. A status document
 * with a session and no `streamed_relay` mode uses the documented `TA` frames.
 * Pairing and disconnect never leave the link armed.
 *
 * While a controller runs, [setFrameSource] hands the drive loop the Rust
 * motor gate's frames instead of the sliders.
 */
@SuppressLint("MissingPermission")
class TerraBluetooth(context: Context) {
    private val appContext = context.applicationContext
    private val main = Handler(Looper.getMainLooper())
    private val thread = HandlerThread("terra.bluetooth").also { it.start() }
    private val handler = Handler(thread.looper)
    private val adapter: BluetoothAdapter? =
        appContext.getSystemService(BluetoothManager::class.java)?.adapter

    var rovers by mutableStateOf<List<DiscoveredRover>>(emptyList())
        private set
    var scanning by mutableStateOf(false)
        private set
    var connected by mutableStateOf(false)
        private set
    var ready by mutableStateOf(false)
        private set
    var armed by mutableStateOf(false)
        private set
    var canStage by mutableStateOf(false)
        private set
    var canCommit by mutableStateOf(false)
        private set
    var statusText by mutableStateOf("Disconnected")
        private set
    var errorText by mutableStateOf("")
        private set
    var summary by mutableStateOf("Hold USR for three seconds, then scan for terra-XXXXXX.")
        private set
    /** Drive profiles, or the full layout on a framed session, once ready; for the Rust actuator frame. */
    var driveLayout by mutableStateOf<String?>(null)
        private set

    private val found = linkedMapOf<String, DiscoveredRover>()
    private var gatt: BluetoothGatt? = null
    private var retiring: BluetoothGatt? = null
    private var driveChar: BluetoothGattCharacteristic? = null
    private var statusChar: BluetoothGattCharacteristic? = null
    private var controlChar: BluetoothGattCharacteristic? = null
    private var repliesChar: BluetoothGattCharacteristic? = null
    private var setupOnly = false
    private var mtu = 23
    private var awaitingBond = false
    private var connectDeadline: Runnable? = null

    private var statusAssembler = ActuatorAssembler(JSON_LIMIT.toUInt())
    private var replyAssembler = ActuatorAssembler(JSON_LIMIT.toUInt())
    private var subscribeStep = SubscribeStep.Idle
    private var sync = Sync.Idle
    private var messageId = 0
    private var requestId = 999
    private var sequence = 0L
    private var sessionId: Long? = null
    private var revision: Long? = null
    private var controlMode = ""
    private var linkFramed = false
    private var linkConnected = false
    private var layoutAvailable = false
    private var fault: String? = null
    private var stopReason: String? = null
    private var outputActive = false
    private var arming = false
    private var gateOpen: Boolean? = null
    private var configurationAllowed: Boolean? = null
    private var layoutJson: String? = null
    private var profilesJson: String? = null
    private var capabilitiesJson = "{}"
    private var profileIds = emptyList<Int>()
    private var profileRows = mutableListOf<JSONObject>()
    private var stagedRequestId: Long? = null
    private var stagedRevision: Long? = null
    private var localArmed = false
    private var effortForward = 0f
    private var effortTurn = 0f
    private var frameSource: (() -> MotorCommand)? = null
    private var sentFrame = false
    private var waiting: Waiting? = null
    private var shutdownLatch: CountDownLatch? = null

    private var active: Outgoing? = null
    private val priority = ArrayDeque<Outgoing>()
    private val handshake = ArrayDeque<Outgoing>()
    private val configuration = ArrayDeque<Outgoing>()
    private var pendingDrive: Outgoing? = null
    private var writeStartedAt: Long? = null

    private val tick = object : Runnable {
        override fun run() {
            if (gatt == null) return
            try {
                onTick()
            } catch (error: Exception) {
                fail(error.message ?: "Bluetooth link failed")
                return
            }
            handler.postDelayed(this, 25)
        }
    }
    private val driveLoop = object : Runnable {
        override fun run() {
            if (!localArmed || gatt == null) return
            try {
                queueMotion()
            } catch (error: Exception) {
                localArmed = false
                publish { armed = false }
                report(error.message ?: "Drive stopped")
                try {
                    queueSafe(priority = true)
                } catch (_: Exception) {
                }
                publishState()
                return
            }
            handler.postDelayed(this, 50)
        }
    }

    private val bondReceiver = object : BroadcastReceiver() {
        override fun onReceive(context: Context?, intent: Intent?) {
            if (intent?.action != BluetoothDevice.ACTION_BOND_STATE_CHANGED) return
            val device = deviceExtra(intent) ?: return
            val state = intent.getIntExtra(BluetoothDevice.EXTRA_BOND_STATE, BluetoothDevice.BOND_NONE)
            handler.post {
                if (!awaitingBond || gatt?.device?.address != device.address) return@post
                when (state) {
                    BluetoothDevice.BOND_BONDED -> {
                        awaitingBond = false
                        statusChar?.let { read(it) }
                    }
                    BluetoothDevice.BOND_NONE -> {
                        awaitingBond = false
                        report("Pairing was not accepted")
                    }
                }
            }
        }
    }

    private val scanCallback = object : ScanCallback() {
        override fun onScanResult(callbackType: Int, result: ScanResult) {
            handler.post { remember(result) }
        }

        override fun onBatchScanResults(results: MutableList<ScanResult>) {
            handler.post { results.forEach { remember(it) } }
        }

        override fun onScanFailed(errorCode: Int) {
            handler.post {
                publish {
                    scanning = false
                    errorText = "Scan failed ($errorCode)"
                    statusText = "Not scanning"
                }
            }
        }
    }

    private val callback = object : BluetoothGattCallback() {
        override fun onConnectionStateChange(gatt: BluetoothGatt, status: Int, newState: Int) {
            handler.post { connectionChanged(gatt, status, newState) }
        }

        override fun onMtuChanged(gatt: BluetoothGatt, mtu: Int, status: Int) {
            handler.post {
                if (gatt != this@TerraBluetooth.gatt) return@post
                if (status == BluetoothGatt.GATT_SUCCESS && mtu >= 23) this@TerraBluetooth.mtu = mtu
                gatt.discoverServices()
            }
        }

        override fun onServicesDiscovered(gatt: BluetoothGatt, status: Int) {
            handler.post {
                if (gatt != this@TerraBluetooth.gatt) return@post
                if (status != BluetoothGatt.GATT_SUCCESS) {
                    fail("Characteristic discovery failed")
                    return@post
                }
                adopt(gatt)
            }
        }

        @Deprecated("Deprecated in Java")
        override fun onCharacteristicRead(
            gatt: BluetoothGatt,
            characteristic: BluetoothGattCharacteristic,
            status: Int,
        ) {
            if (Build.VERSION.SDK_INT >= 33) return
            @Suppress("DEPRECATION")
            val value = characteristic.value ?: byteArrayOf()
            handler.post { inbound(gatt, characteristic, value, status, notification = false) }
        }

        override fun onCharacteristicRead(
            gatt: BluetoothGatt,
            characteristic: BluetoothGattCharacteristic,
            value: ByteArray,
            status: Int,
        ) {
            handler.post { inbound(gatt, characteristic, value, status, notification = false) }
        }

        @Deprecated("Deprecated in Java")
        override fun onCharacteristicChanged(
            gatt: BluetoothGatt,
            characteristic: BluetoothGattCharacteristic,
        ) {
            if (Build.VERSION.SDK_INT >= 33) return
            @Suppress("DEPRECATION")
            val value = characteristic.value ?: byteArrayOf()
            handler.post { inbound(gatt, characteristic, value, BluetoothGatt.GATT_SUCCESS, notification = true) }
        }

        override fun onCharacteristicChanged(
            gatt: BluetoothGatt,
            characteristic: BluetoothGattCharacteristic,
            value: ByteArray,
        ) {
            handler.post { inbound(gatt, characteristic, value, BluetoothGatt.GATT_SUCCESS, notification = true) }
        }

        override fun onCharacteristicWrite(
            gatt: BluetoothGatt,
            characteristic: BluetoothGattCharacteristic,
            status: Int,
        ) {
            handler.post { writeFinished(gatt, characteristic, status) }
        }

        override fun onDescriptorWrite(gatt: BluetoothGatt, descriptor: BluetoothGattDescriptor, status: Int) {
            handler.post { descriptorFinished(gatt, descriptor, status) }
        }
    }

    init {
        val filter = IntentFilter(BluetoothDevice.ACTION_BOND_STATE_CHANGED)
        if (Build.VERSION.SDK_INT >= 33) {
            appContext.registerReceiver(bondReceiver, filter, Context.RECEIVER_NOT_EXPORTED)
        } else {
            @Suppress("UnspecifiedRegisterReceiverFlag")
            appContext.registerReceiver(bondReceiver, filter)
        }
    }

    fun startScan() {
        handler.post { startScanOnThread() }
    }

    fun stopScan() {
        handler.post { stopScanOnThread() }
    }

    fun connect(address: String) {
        handler.post { connectOnThread(address) }
    }

    fun disconnect() {
        handler.post { disarmOnThread(); disconnectOnThread("Disconnected") }
    }

    fun arm() {
        handler.post { armOnThread() }
    }

    fun disarm() {
        handler.post { disarmOnThread() }
    }

    fun setEffort(forward: Double, turn: Double) {
        handler.post {
            if (!forward.isFinite() || !turn.isFinite()) return@post
            effortForward = forward.toFloat().coerceIn(-1f, 1f)
            effortTurn = turn.toFloat().coerceIn(-1f, 1f)
        }
    }

    /**
     * While set, each 50 ms drive tick sends the source's command instead of
     * the slider effort: a frame as is, a stop as neutral output, and idle as
     * nothing (one neutral output right after a frame). Arm still gates output.
     */
    fun setFrameSource(source: (() -> MotorCommand)?) {
        handler.post {
            frameSource = source
            sentFrame = false
        }
    }

    fun stageActiveLayout() {
        handler.post { stageOnThread() }
    }

    fun commitStagedLayout() {
        handler.post { commitOnThread() }
    }

    fun refresh() {
        handler.post {
            if (gatt == null || subscribeStep != SubscribeStep.Done) return@post
            if (localArmed) {
                report("Disarm before refreshing configuration")
                return@post
            }
            startSync()
        }
    }

    fun clearError() {
        publish { errorText = "" }
    }

    fun permissionDenied() {
        publish {
            scanning = false
            errorText = "Bluetooth permission is required to scan and connect."
            statusText = "Permission required"
        }
    }

    /** Stop and leaving the foreground end here. Safe output is queued, then the link drops. */
    fun disarmAndDisconnect() {
        val latch = CountDownLatch(1)
        val posted = handler.post {
            shutdownLatch = latch
            disarmOnThread()
            if (gatt == null && retiring == null) {
                finishShutdown("Disconnected")
                return@post
            }
            handler.postDelayed({ finishShutdown("Disconnected") }, 700)
        }
        if (!posted) latch.countDown()
        latch.await(1200, TimeUnit.MILLISECONDS)
    }

    fun close() {
        disarmAndDisconnect()
        try {
            appContext.unregisterReceiver(bondReceiver)
        } catch (_: Exception) {
        }
        thread.quitSafely()
    }

    private fun startScanOnThread() {
        if (!permitted()) {
            permissionDenied()
            return
        }
        val radio = adapter
        if (radio == null || !radio.isEnabled) {
            publish {
                errorText = "Turn Bluetooth on, then scan again."
                statusText = "Bluetooth is off"
            }
            return
        }
        val scanner = radio.bluetoothLeScanner
        if (scanner == null) {
            publish { errorText = "This phone has no Bluetooth LE radio."; statusText = "Bluetooth unavailable" }
            return
        }
        found.clear()
        publish { rovers = emptyList(); scanning = true; errorText = ""; statusText = "Scanning" }
        val filter = ScanFilter.Builder().setServiceUuid(ParcelUuid(SERVICE)).build()
        val settings = ScanSettings.Builder().setScanMode(ScanSettings.SCAN_MODE_LOW_LATENCY).build()
        try {
            scanner.startScan(listOf(filter), settings, scanCallback)
        } catch (error: SecurityException) {
            permissionDenied()
        }
    }

    private fun stopScanOnThread() {
        try {
            adapter?.bluetoothLeScanner?.stopScan(scanCallback)
        } catch (_: SecurityException) {
        }
        publish { scanning = false }
    }

    private fun remember(result: ScanResult) {
        val device = result.device ?: return
        val advertised = result.scanRecord?.deviceName ?: device.name ?: "Terra rover"
        val pairing = advertised.endsWith("-pair")
        val name = if (pairing) advertised.removeSuffix("-pair") else advertised
        found[device.address] = DiscoveredRover(device.address, name.ifBlank { "Terra rover" }, pairing)
        publish { rovers = found.values.toList() }
    }

    private fun connectOnThread(address: String) {
        if (!permitted()) {
            permissionDenied()
            return
        }
        val radio = adapter ?: run {
            report("Bluetooth is unavailable")
            return
        }
        val device = try {
            radio.getRemoteDevice(address)
        } catch (_: IllegalArgumentException) {
            report("That rover address is not valid")
            return
        }
        stopScanOnThread()
        disarmOnThread()
        disconnectOnThread("Connecting")
        clearLink()
        mtu = 23
        publish { statusText = "Connecting"; errorText = ""; connected = false; summary = deviceLabel(device) }
        try {
            gatt = device.connectGatt(appContext, false, callback, BluetoothDevice.TRANSPORT_LE, BluetoothDevice.PHY_LE_1M_MASK, handler)
        } catch (error: SecurityException) {
            permissionDenied()
            return
        }
        val timeout = Runnable {
            if (gatt != null && subscribeStep != SubscribeStep.Done) fail("Connection timed out")
        }
        connectDeadline = timeout
        handler.postDelayed(timeout, 15_000)
    }

    private fun connectionChanged(gatt: BluetoothGatt, status: Int, newState: Int) {
        if (gatt == retiring) {
            retiring = null
            gatt.close()
            finishShutdown("Disconnected")
            return
        }
        if (gatt != this.gatt) return
        when (newState) {
            BluetoothProfile.STATE_CONNECTED -> {
                publish { connected = true; statusText = "Discovering services" }
                handler.post(tick)
                if (!gatt.requestMtu(185)) gatt.discoverServices()
            }
            BluetoothProfile.STATE_DISCONNECTED -> {
                localArmed = false
                this.gatt = null
                gatt.close()
                clearLink()
                publish {
                    connected = false
                    ready = false
                    armed = false
                    canStage = false
                    canCommit = false
                    statusText = if (status == BluetoothGatt.GATT_SUCCESS) "Disconnected" else "Disconnected. Scan and connect again."
                }
                finishShutdown("Disconnected")
            }
        }
    }

    private fun adopt(gatt: BluetoothGatt) {
        val service = gatt.getService(SERVICE) ?: run {
            fail("Terra service unavailable")
            return
        }
        driveChar = service.getCharacteristic(DRIVE)
        statusChar = service.getCharacteristic(STATUS)
        controlChar = service.getCharacteristic(CONTROL)
        repliesChar = service.getCharacteristic(REPLIES)
        val status = statusChar ?: run {
            fail("Status unavailable")
            return
        }
        setupOnly = driveChar == null && controlChar == null && repliesChar == null
        gatt.requestConnectionPriority(BluetoothGatt.CONNECTION_PRIORITY_HIGH)
        publish {
            statusText = if (setupOnly) {
                "Accept Bluetooth pairing on your phone. Motors stay disarmed."
            } else {
                "Authenticating owner"
            }
        }
        read(status)
    }

    private fun inbound(
        gatt: BluetoothGatt,
        characteristic: BluetoothGattCharacteristic,
        value: ByteArray,
        status: Int,
        notification: Boolean,
    ) {
        if (gatt != this.gatt) return
        if (status == INSUFFICIENT_AUTHENTICATION || status == INSUFFICIENT_ENCRYPTION) {
            requestBond(gatt.device)
            return
        }
        if (status != BluetoothGatt.GATT_SUCCESS) {
            fail("Encrypted read failed ($status). Accept pairing, then connect again.")
            return
        }
        val assembler = when (characteristic.uuid) {
            STATUS -> statusAssembler
            REPLIES -> replyAssembler
            else -> return
        }
        if (value.isEmpty()) {
            if (!notification) requestBond(gatt.device)
            return
        }
        val document = try {
            logicalDocument(value, assembler, notification)
        } catch (error: Exception) {
            fail(error.message ?: "Malformed Bluetooth message. Reconnect required.")
            return
        } ?: return
        val text = document.toString(Charsets.UTF_8)
        val json = try {
            JSONObject(text)
        } catch (_: Exception) {
            if (!notification && subscribeStep == SubscribeStep.Idle && !setupOnly) subscribeStatus()
            else fail("Malformed Bluetooth message. Reconnect required.")
            return
        }
        if (json.optInt("schema_version") != 1) {
            if (!notification && subscribeStep == SubscribeStep.Idle && !setupOnly) subscribeStatus()
            return
        }
        if (setupOnly || json.optString("type") == "setup") {
            if (json.optString("type") == "setup" && json.optBoolean("pairing_confirmed")) {
                publish { statusText = "Pairing saved. Reconnect after the rover finishes startup."; errorText = "" }
                disconnectOnThread("Pairing saved. Reconnect after the rover finishes startup.")
            } else {
                fail("Pairing was not confirmed")
            }
            return
        }
        when (characteristic.uuid) {
            STATUS -> applyStatus(json)
            REPLIES -> applyReply(json)
        }
    }

    private fun logicalDocument(value: ByteArray, assembler: ActuatorAssembler, notification: Boolean): ByteArray? {
        if (!notification && value.isNotEmpty() && value[0] == '{'.code.toByte()) {
            val text = value.toString(Charsets.UTF_8)
            try {
                JSONObject(text)
                return value
            } catch (_: Exception) {
            }
        }
        return assembler.push(value, SystemClock.elapsedRealtime().toULong())
    }

    private fun applyStatus(json: JSONObject) {
        when (json.optString("type")) {
            "command_acceptance" -> {
                val accepted = json.optBoolean("accepted")
                publish {
                    summary = if (accepted) "Rover accepted the last command" else "Rover rejected the last command"
                }
                if (!accepted) {
                    localArmed = false
                    handler.removeCallbacks(driveLoop)
                    publish { armed = false; errorText = "Command rejected. Arm again with a fresh safe output." }
                }
                return
            }
            "status" -> Unit
            else -> if (!json.has("control_mode") && !json.has("active_revision") && !json.has("session")) return
        }
        connectDeadline?.let { handler.removeCallbacks(it) }
        connectDeadline = null
        controlMode = json.optString("control_mode")
        if (json.has("session") && !json.isNull("session")) {
            val incoming = json.getLong("session")
            if (sessionId != incoming) {
                sessionId = incoming
                sequence = 0
            }
        }
        if (json.has("connected")) linkConnected = json.optBoolean("connected")
        if (json.has("active_revision") && !json.isNull("active_revision")) {
            val incoming = json.getLong("active_revision")
            val changed = revision != null && revision != incoming
            revision = incoming
            if (changed && sync == Sync.Done) startSync()
        }
        if (json.has("layout_available")) layoutAvailable = json.optBoolean("layout_available")
        if (json.has("output_active")) outputActive = json.optBoolean("output_active")
        if (json.has("arming")) arming = json.optBoolean("arming")
        if (json.has("hardware_gate_open_confirmed")) gateOpen = json.optBoolean("hardware_gate_open_confirmed")
        if (json.has("configuration_allowed")) configurationAllowed = json.optBoolean("configuration_allowed")
        fault = if (!json.has("fault") || json.isNull("fault")) null else json.optString("fault").takeIf { it.isNotBlank() }
        stopReason = json.optString("stop_reason").takeIf { it.isNotBlank() }
        if (!layoutAvailable) {
            profilesJson = null
            profileRows.clear()
        }
        linkFramed = sessionId != null && controlMode != "streamed_relay" && layoutJson != null
        if (json.optBoolean("armed", false) == false && linkFramed) {
            // Rover disarmed itself. Stop streaming motion.
            if (localArmed && (fault != null || json.optString("stop_reason") == "disconnected")) {
                localArmed = false
                handler.removeCallbacks(driveLoop)
            }
        }
        if (subscribeStep == SubscribeStep.Idle) subscribeStatus()
        publishState()
    }

    private fun applyReply(json: JSONObject) {
        val id = if (json.has("request_id")) json.getLong("request_id") else return
        val pending = waiting ?: return
        if (pending.id != id) return
        waiting = null
        replyAssembler.clear()
        if (json.optString("result") != "ok") {
            val message = configurationError(json)
            if (pending.operation == "stage_layout" || pending.operation == "commit_layout") {
                report(message)
            } else {
                sync = Sync.Failed
                report(message)
            }
            publishState()
            pump()
            return
        }
        val payload = json.opt("payload")
        when (pending.operation) {
            "capabilities" -> {
                capabilitiesJson = if (payload is JSONObject) payload.toString() else "{}"
                sendControl("read_layout", "{}")
                sync = Sync.Layout
            }
            "read_layout" -> {
                layoutJson = if (payload is JSONObject) payload.toString() else null
                linkFramed = sessionId != null && controlMode != "streamed_relay" && layoutJson != null
                if (paginationEnabled()) {
                    sync = Sync.Index
                    sendControl("layout_index", "{}")
                } else {
                    finishSync()
                }
            }
            "layout_index" -> {
                val objectPayload = payload as? JSONObject
                val entries = objectPayload?.optJSONArray("actuators")
                val count = objectPayload?.optInt("actuator_count", -1) ?: -1
                if (objectPayload == null || entries == null || count != entries.length()) {
                    sync = Sync.Failed
                    report("Actuator index was incomplete")
                } else if (!objectPayload.optBoolean("layout_available") || entries.length() == 0) {
                    profilesJson = null
                    layoutAvailable = false
                    finishSync()
                } else {
                    profileIds = (0 until entries.length()).map { entries.getJSONObject(it).getInt("id") }
                    profileRows = mutableListOf()
                    sync = Sync.Profile
                    requestNextProfile()
                }
            }
            "read_drive_profile" -> {
                val actuator = (payload as? JSONObject)?.optJSONObject("actuator")
                if (actuator == null) {
                    sync = Sync.Failed
                    report("Drive profile was incomplete")
                } else {
                    profileRows.add(actuator)
                    if (profileRows.size < profileIds.size) requestNextProfile() else finishProfiles()
                }
            }
            "stage_layout" -> {
                val objectPayload = payload as? JSONObject
                stagedRequestId = objectPayload.longOrNull("staged_request_id")
                stagedRevision = objectPayload.longOrNull("staged_revision")
                publish { errorText = ""; statusText = "Layout staged. Commit to apply it." }
            }
            "commit_layout" -> {
                stagedRequestId = null
                stagedRevision = null
                publish { errorText = ""; statusText = "Layout committed. Reading it back." }
                startSync()
            }
        }
        publishState()
        pump()
    }

    private fun finishProfiles() {
        val rows = JSONArray()
        profileRows.forEach { rows.put(it) }
        profilesJson = JSONObject()
            .put("schema_version", 1)
            .put("revision", revision ?: 0)
            .put("actuators", rows)
            .toString()
        finishSync()
    }

    private fun finishSync() {
        sync = Sync.Done
        publishState()
    }

    private fun requestNextProfile() {
        val id = profileIds[profileRows.size]
        val expected = revision ?: run {
            report("Status has no revision yet")
            return
        }
        val payload = JSONObject().put("expected_revision", expected).put("actuator_id", id).toString()
        sendControl("read_drive_profile", payload)
    }

    private fun startSync() {
        if (waiting != null || configuration.isNotEmpty()) return
        sync = Sync.Capabilities
        stagedRequestId = null
        stagedRevision = null
        profilesJson = null
        profileRows.clear()
        sendControl("capabilities", "{}")
        publishState()
    }

    private fun subscribeStatus() {
        val status = statusChar ?: return
        val replies = repliesChar ?: run {
            fail("Replies unavailable")
            return
        }
        subscribeStep = SubscribeStep.Status
        if (!enableNotify(status)) {
            fail("Status subscription failed")
            return
        }
        if (replies.descriptors.isEmpty()) {
            // Some stacks report notification as already on. Still require the reply subscribe.
        }
    }

    private fun descriptorFinished(gatt: BluetoothGatt, descriptor: BluetoothGattDescriptor, status: Int) {
        if (gatt != this.gatt) return
        writeStartedAt = null
        if (status != BluetoothGatt.GATT_SUCCESS || descriptor.uuid != CCCD) {
            fail("Status subscription failed")
            return
        }
        when (subscribeStep) {
            SubscribeStep.Status -> {
                val replies = repliesChar ?: run {
                    fail("Replies unavailable")
                    return
                }
                subscribeStep = SubscribeStep.Replies
                if (!enableNotify(replies)) fail("Reply subscription failed")
            }
            SubscribeStep.Replies -> {
                subscribeStep = SubscribeStep.Done
                startSync()
            }
            SubscribeStep.Idle, SubscribeStep.Done -> Unit
        }
    }

    private fun enableNotify(characteristic: BluetoothGattCharacteristic): Boolean {
        val gatt = gatt ?: return false
        if (!gatt.setCharacteristicNotification(characteristic, true)) return false
        val descriptor = characteristic.getDescriptor(CCCD) ?: return false
        return writeDescriptor(gatt, descriptor, BluetoothGattDescriptor.ENABLE_NOTIFICATION_VALUE)
    }

    private fun armOnThread() {
        if (!readyOnThread()) {
            report("Connect and finish reading the layout before arming")
            return
        }
        if (fault != null) {
            report("Fault: $fault")
            return
        }
        effortForward = 0f
        effortTurn = 0f
        localArmed = false
        handler.removeCallbacks(driveLoop)
        handshake.clear()
        pendingDrive = null
        try {
            if (linkFramed) {
                handshake.add(framedDrive(safeValues(), stamp = false, onComplete = null))
                handshake.add(framedCommand("arm") { startDriveLoop() })
                publish { statusText = "Sending safe output, then arm" }
            } else {
                queueSafe(priority = true)
                startDriveLoop()
            }
        } catch (error: Exception) {
            report(error.message ?: "Arm failed")
            return
        }
        pump()
        publishState()
    }

    private fun startDriveLoop() {
        localArmed = true
        handler.removeCallbacks(driveLoop)
        handler.post(driveLoop)
        publish { armed = true; errorText = ""; statusText = "Armed · motors follow the Rust gate" }
    }

    private fun disarmOnThread() {
        localArmed = false
        effortForward = 0f
        effortTurn = 0f
        handler.removeCallbacks(driveLoop)
        handshake.clear()
        pendingDrive = null
        if (active?.characteristic == DRIVE && !linkFramed) active?.chunks?.clear()
        try {
            if (gatt != null && (profilesJson != null || layoutJson != null)) queueSafe(priority = true)
        } catch (error: Exception) {
            report(error.message ?: "Could not send safe output")
        }
        publish { armed = false }
        publishState()
        maybeFinishShutdown()
    }

    private fun stageOnThread() {
        if (localArmed) {
            report("Disarm before changing the layout")
            return
        }
        if (gateOpen == false) {
            report("Open the hardware gate before configuration. Motors stay disarmed.")
            return
        }
        if (configurationAllowed == false) {
            report("The rover is not accepting configuration")
            return
        }
        val layout = layoutJson ?: run {
            report("No layout has been read yet")
            return
        }
        val payload = JSONObject().put("layout", JSONObject(layout)).toString()
        try {
            sendControl("stage_layout", payload)
            publish { statusText = "Staging the layout read from the rover" }
        } catch (error: Exception) {
            report(error.message ?: "Could not stage the layout")
        }
    }

    private fun commitOnThread() {
        if (localArmed) {
            report("Disarm before changing the layout")
            return
        }
        val request = stagedRequestId ?: return
        val staged = stagedRevision ?: return
        val payload = JSONObject()
            .put("staged_request_id", request)
            .put("staged_revision", staged)
            .toString()
        try {
            sendControl("commit_layout", payload)
            publish { statusText = "Committing the staged layout" }
        } catch (error: Exception) {
            report(error.message ?: "Could not commit the layout")
        }
    }

    private fun queueMotion() {
        val source = frameSource ?: return queueValues(routedValues(effortForward, effortTurn))
        when (val command = source()) {
            is MotorCommand.Drive -> {
                queueValues(command.valuesJson)
                sentFrame = true
            }
            is MotorCommand.Stop -> {
                queueValues(safeValues())
                sentFrame = false
            }
            MotorCommand.Idle -> if (sentFrame) {
                queueValues(safeValues())
                sentFrame = false
            }
        }
    }

    /** Drive values on the drive characteristic, never `disarm`, so the link stays armed. */
    private fun queueValues(values: String) {
        if (linkFramed) {
            pendingDrive = framedDrive(values, stamp = true, onComplete = null)
        } else {
            if (active?.characteristic == DRIVE) active?.chunks?.clear()
            pendingDrive = relayPacket(values, stamp = true)
        }
        pump()
    }

    private fun queueSafe(priority: Boolean) {
        val values = safeValues()
        val packet = if (linkFramed && sessionId != null) {
            framedCommand("disarm", null)
        } else {
            relayPacket(values, stamp = false)
        }
        if (priority) {
            this.priority.clear()
            this.priority.add(packet)
        } else {
            pendingDrive = packet
        }
        pump()
    }

    private fun safeValues(): String = routedValues(0f, 0f)

    /** Streamed-relay prefers compact drive profiles. Framed sessions, and a layout with no profiles, use the full layout. */
    private fun routedValues(forward: Float, turn: Float): String {
        val profiles = profilesJson
        return if (!linkFramed && profiles != null) {
            actuatorRouteProfiles(profiles, forward, turn)
        } else {
            actuatorRouteStick(layoutJson ?: error("No layout"), forward, turn)
        }
    }

    private fun JSONObject?.longOrNull(key: String): Long? {
        if (this == null || !has(key) || isNull(key)) return null
        return getLong(key)
    }

    private fun framedDrive(valuesJson: String, stamp: Boolean, onComplete: (() -> Unit)?): Outgoing {
        val bytes = frameBytes("drive", JSONArray(valuesJson))
        return Outgoing(fragment(bytes), DRIVE, if (stamp) SystemClock.elapsedRealtime() else null, null, onComplete)
    }

    private fun framedCommand(kind: String, onComplete: (() -> Unit)?): Outgoing {
        val bytes = frameBytes(kind, JSONArray())
        return Outgoing(fragment(bytes), CONTROL, null, null, onComplete)
    }

    private fun frameBytes(kind: String, values: JSONArray): ByteArray {
        val session = sessionId ?: error("Waiting for a rover session")
        val activeRevision = revision ?: error("Waiting for a layout revision")
        val next = sequence + 1
        if (next >= 0xFFFF_FFFE) error("Sequence exhausted. Reconnect.")
        val frame = JSONObject()
            .put("kind", kind)
            .put("session", session)
            .put("revision", activeRevision)
            .put("sequence", next)
            .put("values", values)
        val bytes = actuatorEncodeFrame(frame.toString())
        sequence = next
        return bytes
    }

    private fun relayPacket(valuesJson: String, stamp: Boolean): Outgoing {
        val bytes = actuatorEncodeRelayDrive(limitsDocument(), valuesJson)
        return Outgoing(relayChunks(bytes), DRIVE, if (stamp) SystemClock.elapsedRealtime() else null, null, null)
    }

    private fun limitsDocument(): String = profilesJson ?: layoutJson ?: error("No actuator limits")

    private fun sendControl(operation: String, payloadJson: String) {
        if (waiting != null || configuration.isNotEmpty()) error("A configuration request is already in flight")
        if (requestId == Int.MAX_VALUE) error("Request IDs exhausted. Reconnect.")
        requestId += 1
        val id = requestId
        val envelope = JSONObject()
            .put("schema_version", 1)
            .put("request_id", id)
            .put("operation", operation)
            .put("payload", JSONObject(payloadJson))
            .toString()
        val bytes = try {
            actuatorEncodeControl(envelope)
        } catch (error: Exception) {
            if (operation == "layout_index" || operation == "read_drive_profile" || operation == "stage_layout") {
                envelope.toByteArray(Charsets.UTF_8)
            } else {
                throw error
            }
        }
        configuration.add(Outgoing(fragment(bytes), CONTROL, null, id, null))
        waiting = Waiting(id.toLong(), operation, SystemClock.elapsedRealtime())
        pump()
    }

    private fun fragment(bytes: ByteArray): ArrayDeque<ByteArray> {
        val parts = actuatorFragment(nextMessageId(), bytes, writeLength().toUInt())
        return ArrayDeque(parts)
    }

    private fun relayChunks(bytes: ByteArray): ArrayDeque<ByteArray> {
        val capacity = (writeLength() / 2) * 2
        if (capacity < 2 || bytes.isEmpty() || bytes.size % 2 != 0) {
            error("Drive payload is not a set of actuator pairs")
        }
        val chunks = ArrayDeque<ByteArray>()
        var offset = 0
        while (offset < bytes.size) {
            val end = minOf(offset + capacity, bytes.size)
            chunks.add(bytes.copyOfRange(offset, end))
            offset = end
        }
        return chunks
    }

    private fun nextMessageId(): UShort {
        messageId = (messageId + 1) and 0xFFFF
        return messageId.toUShort()
    }

    private fun pump() {
        if (writeStartedAt != null || gatt == null) return
        if (active?.chunks?.isEmpty() == true) {
            val done = active
            active = null
            done?.onComplete?.invoke()
        }
        if (active == null) {
            active = when {
                priority.isNotEmpty() -> priority.removeFirst()
                handshake.isNotEmpty() -> handshake.removeFirst()
                configuration.isNotEmpty() -> configuration.removeFirst()
                pendingDrive != null -> pendingDrive.also { pendingDrive = null }
                else -> null
            }
        }
        val packet = active ?: run {
            maybeFinishShutdown()
            return
        }
        val produced = packet.producedAt
        if (produced != null && SystemClock.elapsedRealtime() - produced >= 50) {
            active = null
            pump()
            return
        }
        val characteristic = characteristic(packet.characteristic) ?: run {
            active = null
            return
        }
        if (packet.chunks.isEmpty()) {
            active = null
            pump()
            return
        }
        val bytes = packet.chunks.removeFirst()
        val gatt = gatt ?: return
        writeStartedAt = SystemClock.elapsedRealtime()
        if (!writeCharacteristic(gatt, characteristic, bytes)) {
            writeStartedAt = null
            packet.chunks.addFirst(bytes)
            handler.postDelayed({ pump() }, 25)
        }
    }

    private fun writeFinished(gatt: BluetoothGatt, characteristic: BluetoothGattCharacteristic, status: Int) {
        if (gatt != this.gatt) return
        if (active?.characteristic != characteristic.uuid || writeStartedAt == null) return
        writeStartedAt = null
        if (status != BluetoothGatt.GATT_SUCCESS) {
            if (active?.requestId != null) {
                waiting = null
                active = null
                report("Configuration write rejected")
                pump()
                return
            }
            fail("Bluetooth write failed. Reconnect before driving.")
            return
        }
        if (active?.chunks?.isEmpty() == true) {
            val done = active
            active = null
            done?.onComplete?.invoke()
        }
        pump()
    }

    private fun onTick() {
        val writeAt = writeStartedAt
        if (writeAt != null && SystemClock.elapsedRealtime() - writeAt >= 2_000) {
            fail("Bluetooth write stalled. Reconnect required.")
            return
        }
        val pending = waiting
        if (pending != null && SystemClock.elapsedRealtime() - pending.startedAt >= 5_000) {
            waiting = null
            if (active?.requestId?.toLong() == pending.id) active?.chunks?.clear()
            sync = Sync.Failed
            report("Configuration timed out. Refresh to try again.")
        }
        pump()
    }

    private fun readyOnThread(): Boolean {
        if (gatt == null || fault != null || subscribeStep != SubscribeStep.Done || sync != Sync.Done) return false
        if (!linkConnected && controlMode == "streamed_relay") return false
        return profilesJson != null || layoutJson != null
    }

    private fun publishState() {
        val live = gatt != null
        val readyNow = live && (readyOnThread() && (layoutAvailable || profilesJson != null))
        val mode = when {
            linkFramed -> "framed session"
            controlMode.isNotBlank() -> controlMode
            else -> "link"
        }
        val actuators = profileIds.size
        val revisionText = revision?.toString() ?: "unknown"
        val layoutNow = if (readyNow) profilesJson ?: layoutJson else null
        val text = when {
            !live -> {
                publish { driveLayout = null }
                return
            }
            fault != null -> "Fault: $fault"
            localArmed -> if (outputActive) "Armed · output active" else "Armed · neutral output"
            arming -> "Arming"
            readyNow -> "Connected · motors disarmed"
            sync == Sync.Failed -> "Configuration failed"
            sync != Sync.Done && sync != Sync.Idle -> "Reading capabilities and layout"
            layoutAvailable -> "Connected"
            else -> "No active layout"
        }
        val detail = buildString {
            append("Revision ")
            append(revisionText)
            append(" · ")
            append(actuators)
            append(if (actuators == 1) " actuator · " else " actuators · ")
            append(mode)
            stopReason?.let { append(" · "); append(it) }
        }
        val armedNow = localArmed
        val stageNow = readyNow && !armedNow && layoutJson != null && gateOpen != false && configurationAllowed != false && fault == null
        val commitNow = stagedRequestId != null && !armedNow && live
        publish {
            ready = readyNow
            driveLayout = layoutNow
            armed = armedNow
            canStage = stageNow
            canCommit = commitNow
            summary = detail
            statusText = text
        }
    }

    private fun paginationEnabled(): Boolean {
        return try {
            val features = JSONObject(capabilitiesJson).optJSONArray("features") ?: return false
            (0 until features.length()).any { features.optString(it) == "actuator_pagination_v1" }
        } catch (_: Exception) {
            false
        }
    }

    private fun configurationError(json: JSONObject): String {
        val errors = json.optJSONArray("errors") ?: return "Configuration rejected"
        val parts = (0 until errors.length()).mapNotNull { index ->
            val item = errors.optJSONObject(index) ?: return@mapNotNull null
            listOf(item.optString("code"), item.optString("message")).filter { it.isNotBlank() }.joinToString(": ")
        }
        return parts.joinToString("; ").ifBlank { "Configuration rejected" }
    }

    private fun disconnectOnThread(reason: String) {
        handler.removeCallbacks(driveLoop)
        handler.removeCallbacks(tick)
        connectDeadline?.let { handler.removeCallbacks(it) }
        connectDeadline = null
        localArmed = false
        val current = gatt
        gatt = null
        clearLink()
        if (current != null) {
            retiring = current
            try {
                current.disconnect()
            } catch (_: SecurityException) {
                retiring = null
                current.close()
            }
        }
        stopScanOnThread()
        publish {
            connected = false
            ready = false
            driveLayout = null
            armed = false
            canStage = false
            canCommit = false
            scanning = false
            statusText = reason
        }
    }

    private fun finishShutdown(reason: String) {
        val latch = shutdownLatch ?: return
        shutdownLatch = null
        if (gatt != null || retiring == null) disconnectOnThread(reason)
        latch.countDown()
    }

    private fun maybeFinishShutdown() {
        if (shutdownLatch == null) return
        if (active != null || priority.isNotEmpty() || handshake.isNotEmpty() || writeStartedAt != null) return
        finishShutdown("Disconnected")
    }

    private fun fail(message: String) {
        localArmed = false
        handler.removeCallbacks(driveLoop)
        report(message)
        disconnectOnThread(message)
    }

    private fun clearLink() {
        sync = Sync.Idle
        subscribeStep = SubscribeStep.Idle
        setupOnly = false
        awaitingBond = false
        sessionId = null
        revision = null
        sequence = 0
        controlMode = ""
        linkFramed = false
        linkConnected = false
        layoutAvailable = false
        fault = null
        stopReason = null
        outputActive = false
        arming = false
        gateOpen = null
        configurationAllowed = null
        layoutJson = null
        profilesJson = null
        capabilitiesJson = "{}"
        profileIds = emptyList()
        profileRows.clear()
        stagedRequestId = null
        stagedRevision = null
        requestId = 999
        waiting = null
        active = null
        priority.clear()
        handshake.clear()
        configuration.clear()
        pendingDrive = null
        writeStartedAt = null
        driveChar = null
        statusChar = null
        controlChar = null
        repliesChar = null
        try {
            statusAssembler.close()
            replyAssembler.close()
        } catch (_: Exception) {
        }
        statusAssembler = ActuatorAssembler(JSON_LIMIT.toUInt())
        replyAssembler = ActuatorAssembler(JSON_LIMIT.toUInt())
    }

    private fun requestBond(device: BluetoothDevice) {
        if (device.bondState == BluetoothDevice.BOND_BONDED) {
            statusChar?.let { read(it) }
            return
        }
        awaitingBond = true
        publish { statusText = "Accept Bluetooth pairing on your phone. Motors stay disarmed."; errorText = "" }
        try {
            device.createBond()
        } catch (_: SecurityException) {
            permissionDenied()
        }
    }

    private fun read(characteristic: BluetoothGattCharacteristic) {
        val gatt = gatt ?: return
        if (!gatt.readCharacteristic(characteristic)) report("Could not read rover status")
    }

    private fun characteristic(uuid: UUID): BluetoothGattCharacteristic? = when (uuid) {
        DRIVE -> driveChar
        CONTROL -> controlChar
        else -> null
    }

    private fun writeCharacteristic(
        gatt: BluetoothGatt,
        characteristic: BluetoothGattCharacteristic,
        value: ByteArray,
    ): Boolean {
        return if (Build.VERSION.SDK_INT >= 33) {
            gatt.writeCharacteristic(characteristic, value, BluetoothGattCharacteristic.WRITE_TYPE_DEFAULT) ==
                android.bluetooth.BluetoothStatusCodes.SUCCESS
        } else {
            @Suppress("DEPRECATION")
            characteristic.writeType = BluetoothGattCharacteristic.WRITE_TYPE_DEFAULT
            @Suppress("DEPRECATION")
            characteristic.value = value
            @Suppress("DEPRECATION")
            gatt.writeCharacteristic(characteristic)
        }
    }

    private fun writeDescriptor(gatt: BluetoothGatt, descriptor: BluetoothGattDescriptor, value: ByteArray): Boolean {
        writeStartedAt = SystemClock.elapsedRealtime()
        val ok = if (Build.VERSION.SDK_INT >= 33) {
            gatt.writeDescriptor(descriptor, value) == android.bluetooth.BluetoothStatusCodes.SUCCESS
        } else {
            @Suppress("DEPRECATION")
            descriptor.value = value
            @Suppress("DEPRECATION")
            gatt.writeDescriptor(descriptor)
        }
        if (!ok) writeStartedAt = null
        return ok
    }

    private fun writeLength(): Int = (mtu - 3).coerceIn(20, 512)

    private fun permitted(): Boolean {
        val needed = if (Build.VERSION.SDK_INT >= 31) {
            arrayOf(android.Manifest.permission.BLUETOOTH_SCAN, android.Manifest.permission.BLUETOOTH_CONNECT)
        } else {
            arrayOf(android.Manifest.permission.ACCESS_FINE_LOCATION)
        }
        return needed.all {
            ContextCompat.checkSelfPermission(appContext, it) == PackageManager.PERMISSION_GRANTED
        }
    }

    private fun deviceLabel(device: BluetoothDevice): String {
        val name = try {
            device.name
        } catch (_: SecurityException) {
            null
        }
        return name ?: device.address
    }

    private fun deviceExtra(intent: Intent): BluetoothDevice? {
        return if (Build.VERSION.SDK_INT >= 33) {
            intent.getParcelableExtra(BluetoothDevice.EXTRA_DEVICE, BluetoothDevice::class.java)
        } else {
            @Suppress("DEPRECATION")
            intent.getParcelableExtra(BluetoothDevice.EXTRA_DEVICE)
        }
    }

    private fun report(message: String) {
        publish { errorText = message }
    }

    private fun publish(block: () -> Unit) {
        if (Looper.myLooper() == Looper.getMainLooper()) block() else main.post(block)
    }

    private class Outgoing(
        val chunks: ArrayDeque<ByteArray>,
        val characteristic: UUID,
        val producedAt: Long?,
        val requestId: Int?,
        val onComplete: (() -> Unit)?,
    )

    private data class Waiting(val id: Long, val operation: String, val startedAt: Long)

    private enum class Sync { Idle, Capabilities, Layout, Index, Profile, Done, Failed }
    private enum class SubscribeStep { Idle, Status, Replies, Done }

    companion object {
        private val SERVICE: UUID = UUID.fromString("7e5a0010-4c2b-4f91-9e3a-1d8c6b2a0f10")
        private val DRIVE: UUID = UUID.fromString("7e5a0011-4c2b-4f91-9e3a-1d8c6b2a0f10")
        private val STATUS: UUID = UUID.fromString("7e5a0012-4c2b-4f91-9e3a-1d8c6b2a0f10")
        private val CONTROL: UUID = UUID.fromString("7e5a0013-4c2b-4f91-9e3a-1d8c6b2a0f10")
        private val REPLIES: UUID = UUID.fromString("7e5a0014-4c2b-4f91-9e3a-1d8c6b2a0f10")
        private val CCCD: UUID = UUID.fromString("00002902-0000-1000-8000-00805f9b34fb")
        private const val JSON_LIMIT = 16384
        private const val INSUFFICIENT_AUTHENTICATION = 5
        private const val INSUFFICIENT_ENCRYPTION = 15
    }
}
