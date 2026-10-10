import Foundation
import Combine
import CoreBluetooth
import OSLog

final class BluetoothLink: NSObject, ObservableObject, CBCentralManagerDelegate, CBPeripheralDelegate {
    @Published private(set) var radioState: TerraConnectionFacts.Radio = .unknown
    @Published private(set) var isReady = false
    @Published private(set) var configurationReady = false
    @Published private(set) var connectionGeneration: UInt64 = 0
    @Published private(set) var armed = false
    @Published private(set) var arming = false
    @Published private(set) var status = "Disconnected"
    @Published private(set) var peripheralsJSON = "[]"
    @Published private(set) var statusJSON = "{}"
    @Published private(set) var capabilitiesJSON = "{}"
    @Published private(set) var layoutIndexJSON = "{}"
    @Published private(set) var driveProfileJSON = "{}"
    @Published private(set) var replyJSON = "{}"
    @Published private(set) var connectedIdentifier: UUID?
    enum Lifecycle { case paired(UUID), authenticated(UUID), closed(retryable: Bool) }
    var onDiscovery: ((TerraDiscoveredRover, UInt64) -> Void)?
    var onLifecycle: ((Lifecycle, UInt64) -> Void)?
    private var automationGeneration: UInt64 = 0
    private var attemptStarted: TimeInterval?
    private var suppressCloseEvent = false
    private func emit(_ event: Lifecycle) {
        let generation = automationGeneration
        DispatchQueue.main.async { self.onLifecycle?(event, generation) }
    }
    func resumeAutomation(generation: UInt64) { queue.async {
        self.automationGeneration = generation
        if self.peripheral == nil { self.emit(.closed(retryable: true)) }
    } }
    func startDiscovery(generation: UInt64) { queue.async {
        self.automationGeneration = generation
        self.cancelAutomaticSelection(); self.scanningRequested = true
        self.found.removeAll(); self.advertisedNames.removeAll()
        self.startRequestedScan()
    } }
    func connect(identifier: UUID, generation: UInt64) { queue.async {
        self.automationGeneration = generation
        self.deferredIdentifier = identifier; self.connectDeferredIfPossible()
    } }
    private let log = TerraLog.bluetooth
    private var lastLoggedConnection = ""
    private var lastLoggedSafety = ""
    private var lastStatusCallbackAt: TimeInterval?
    private var acceptedStatusCount = 0
    private let queue = DispatchQueue(label: "terra.bluetooth")
    private var central: CBCentralManager!
    private var scanningRequested = false
    private var automaticSelection: ((UUID) -> Void)?
    private var preferredRover: UUID?
    private var discoveryWindow: DispatchWorkItem?
    private var advertisedNames: [UUID: String] = [:]
    private var found: [UUID: CBPeripheral] = [:]
    private var peripheral: CBPeripheral?
    private var retiring: CBPeripheral?
    private var deferredIdentifier: UUID?
    private var synchronizedRevision: UInt32?
    private var hasCapabilities = false
    private var synchronizationStarted = false
    private var capabilityObject: [String: Any] = [:]
    private var safeValues: [[String: Any]] = []
    private var supportsPagination = false
    private var profileSync = ActuatorProfileSynchronization()
    private var synchronizationRound: UInt64 = 0
    private var synchronizationRequests: [UInt32: UInt64] = [:]
    private var lastObservedRevision: UInt32?
    private var awaitingSafeSequence: UInt32?
    private var awaitingSafeAt: TimeInterval?
    private var usedRequests: Set<UInt32> = []
    private let policy = BluetoothSession()
    private var callbackGeneration: UInt64 = 0
    private var characteristics: [CBUUID: CBCharacteristic] = [:]
    private var statusAssembly = BluetoothJSONAssembler()
    private var replyAssembly = BluetoothJSONAssembler(timeout: 2, limit: 4080)
    private var admitting = false
    private var setup = false
    private var timer: DispatchSourceTimer?
    private var messageID: UInt16 = 0
    private var requestID: UInt32 = max(0x80000000, UInt32(clamping: UserDefaults.standard.integer(forKey: "bluetoothSynchronizationRequestID")))
    private var requests: [UInt32: String] = [:]
    private var retryEnvelopes: [UInt32: Data] = [:]
    private var failedRequests: Set<UInt32> = []
    private var configuration: [Packet] = []
    private var waitingRequest: UInt32?
    private var waitingRequestAt: TimeInterval?
    private var armSafe: Packet?
    private var priority: Packet?
    private var pendingDrive: Packet?
    private var active: Packet?
    private var writeAt: TimeInterval?
    private struct Packet { var chunks: [Data]; let characteristic: CBUUID; let id: UInt16; let request: UInt32?; let producedAt: TimeInterval? }
    private static func uuid(_ suffix: String) -> CBUUID { CBUUID(string: "7e5a00\(suffix)-4c2b-4f91-9e3a-1d8c6b2a0f10") }
    private let serviceID = BluetoothLink.uuid("10")
    private let driveID = BluetoothLink.uuid("11")
    private let statusID = BluetoothLink.uuid("12")
    private let controlID = BluetoothLink.uuid("13")
    private let repliesID = BluetoothLink.uuid("14")
    private var now: TimeInterval { ProcessInfo.processInfo.systemUptime }
    override init() {
        super.init(); central = CBCentralManager(delegate: self, queue: queue)
        let timer = DispatchSource.makeTimerSource(queue: queue)
        timer.schedule(deadline: .now(), repeating: .milliseconds(25))
        timer.setEventHandler { [weak self] in self?.tick() }; timer.resume(); self.timer = timer
    }
    deinit { timer?.cancel() }
    private func publish(_ text: String) {
        if text != lastLoggedConnection {
            lastLoggedConnection = text
            log.info("Connection: \(text, privacy: .public)")
        }
        recordConnection(text)
        DispatchQueue.main.async { self.status = text }
    }
    // Bounded local debug trace can be retrieved over the paired developer link.
    // No actuator values, ownership credentials or sensor data are recorded.
    private func recordConnection(_ text: String) {
        #if DEBUG
        guard let directory = FileManager.default.urls(for: .documentDirectory, in: .userDomainMask).first else { return }
        let url = directory.appendingPathComponent("ConnectionDiagnostics.json")
        var rows = (try? Data(contentsOf: url)).flatMap { try? JSONSerialization.jsonObject(with: $0) as? [[String: String]] } ?? []
        rows.append(["time": ISO8601DateFormatter().string(from: Date()), "event": text])
        if rows.count > 100 { rows.removeFirst(rows.count - 100) }
        if let bytes = try? JSONSerialization.data(withJSONObject: rows) { try? bytes.write(to: url, options: .atomic) }
        #endif
    }
    func scan() { queue.async {
        self.cancelAutomaticSelection()
        self.scanningRequested = true
        self.startRequestedScan()
    } }
    func scanForAutomaticConnection(preferred: UUID?, onSelect: @escaping (UUID) -> Void) {
        queue.async {
            guard self.peripheral == nil, self.automaticSelection == nil else { return }
            self.found.removeAll(); self.advertisedNames.removeAll()
            DispatchQueue.main.async { self.peripheralsJSON = "[]" }
            self.preferredRover = preferred
            self.automaticSelection = onSelect
            self.scanningRequested = true
            self.startRequestedScan()
        }
    }
    func cancelAutomaticConnection() { queue.async {
        self.cancelAutomaticSelection()
        self.scanningRequested = false
        self.central.stopScan()
    } }
    private func cancelAutomaticSelection() {
        discoveryWindow?.cancel(); discoveryWindow = nil; automaticSelection = nil
    }
    private func startRequestedScan() {
        guard scanningRequested else { return }
        guard central.state == .poweredOn else {
            publish(central.state == .unauthorized ? "Bluetooth permission required" : "Waiting for Bluetooth")
            return
        }
        central.scanForPeripherals(withServices: [serviceID], options: [CBCentralManagerScanOptionAllowDuplicatesKey: true])
        publish(automaticSelection == nil ? "Scanning" : "Searching for your rover…")
    }
    private func selectAutomaticRover() {
        guard let selection = automaticSelection else { return }
        if let id = TerraAutoConnectionPolicy.select(candidates: Array(found.keys), preferred: preferredRover) {
            cancelAutomaticSelection(); scanningRequested = false; central.stopScan()
            selection(id)
        } else if preferredRover == nil, found.count > 1 {
            cancelAutomaticSelection(); scanningRequested = false; central.stopScan()
            publish("Multiple rovers found · choose one in Robots")
        }
    }
    func connect(identifier: UUID) { queue.async {
        self.deferredIdentifier = identifier
        self.connectDeferredIfPossible()
    } }
    private func connectDeferredIfPossible() {
        guard peripheral == nil, retiring == nil, let identifier = deferredIdentifier else { return }
        deferredIdentifier = nil
        guard let selected = found[identifier] else { publish("Scan and select the rover again"); return }
        clear(); peripheral = selected; callbackGeneration = policy.generation
        attemptStarted = now
        selected.delegate = self; scanningRequested = false; cancelAutomaticSelection(); central.stopScan(); central.connect(selected); publish("Connecting")
    }
    func disconnect() { queue.async {
        self.cancelAutomaticSelection(); self.scanningRequested = false; self.central.stopScan()
        self.deferredIdentifier = nil; self.suppressCloseEvent = true; self.close("Disconnected"); self.suppressCloseEvent = false
    } }
    private func clear() {
        synchronizationRound &+= 1; synchronizationRequests.removeAll(); profileSync.clear()
        supportsPagination = false; lastObservedRevision = nil
        policy.reset(); usedRequests.removeAll(); awaitingSafeAt = nil; safeValues = []; awaitingSafeSequence = nil; synchronizedRevision = nil; hasCapabilities = false; synchronizationStarted = false; capabilityObject = [:]; characteristics.removeAll(); statusAssembly.clear(); replyAssembly.clear()
        active = nil; armSafe = nil; waitingRequest = nil; waitingRequestAt = nil; pendingDrive = nil; priority = nil; configuration.removeAll(); requests.removeAll(); retryEnvelopes.removeAll(); failedRequests.removeAll()
        writeAt = nil; admitting = false; setup = false
        lastStatusCallbackAt = nil; acceptedStatusCount = 0; attemptStarted = nil
        let generation = policy.generation
        DispatchQueue.main.async { self.connectionGeneration = generation; self.configurationReady = false; self.isReady = false; self.armed = false; self.arming = false; self.connectedIdentifier = nil; self.capabilitiesJSON = "{}"; self.layoutIndexJSON = "{}"; self.driveProfileJSON = "{}"; self.statusJSON = "{}" }
    }
    private func close(_ reason: String) {
        if peripheral != nil && !suppressCloseEvent {
            emit(.closed(retryable: TerraBluetoothFailurePolicy.retryable(reason)))
        }
        log.error("Closing link: \(reason, privacy: .public)"); let old = peripheral; peripheral = nil; clear(); if let old = old { retiring = old; central.cancelPeripheralConnection(old) }; publish(reason) }
    private func terminal(_ reason: String) { emit(.closed(retryable: true)); peripheral?.delegate = nil; peripheral = nil; retiring = nil; clear(); publish(reason) }
    private func current(_ p: CBPeripheral) -> Bool { peripheral === p && callbackGeneration == policy.generation }
    func send(frame: Data, producedAt: TimeInterval) { queue.async {
        guard self.awaitingSafeSequence == nil else { return }
        for effect in self.policy.drive(frame, producedAt: producedAt, now: self.now) {
            if case .send(let bytes) = effect { do { self.pendingDrive = try self.packet(bytes, characteristic: self.driveID, producedAt: producedAt); self.pump() } catch { self.close("Invalid drive frame") } }
        }
    } }
    func drive(valuesJSON: String, producedAt: TimeInterval) { queue.async {
        // Expected motion inhibition must not preempt fragmented repair/reset requests.
        guard self.policy.ready, self.policy.motionRequested else { return }
        do {
            guard self.awaitingSafeSequence == nil, self.now >= producedAt, self.now - producedAt < 0.1 else { return }
            let values = try JSONSerialization.jsonObject(with: Data(valuesJSON.utf8))
            // Safe output remains freshly produced throughout the ESC arming interval.
            let selected: Any = (self.policy.armed && self.awaitingSafeSequence == nil) ? values : self.safeValues
            let frame = try self.policy.frame(values: selected)
            for effect in self.policy.drive(frame, producedAt: producedAt, now: self.now) {
                if case .send(let bytes) = effect { self.pendingDrive = try self.packet(bytes, characteristic: self.driveID, producedAt: producedAt) }
            }; self.pump()
        } catch { self.stop() }
    } }
    func arm() { queue.async {
        self.log.notice("Arm requested; ready=\(self.policy.ready)")
        do {
            guard self.policy.ready, self.awaitingSafeSequence == nil else { throw BluetoothPolicyError.unavailable }
            let produced = self.now
            let safe = try self.policy.frame(values: self.safeValues)
            for effect in self.policy.drive(safe, producedAt: produced, now: self.now) {
                if case .send(let bytes) = effect { self.awaitingSafeSequence = self.policy.sequence; self.awaitingSafeAt = produced; self.pendingDrive = nil; self.armSafe = try self.packet(bytes, characteristic: self.driveID, producedAt: produced) }
            }; self.pump()
        } catch { self.publish("Synchronize layout and safe output before arming") }
    } }
    func emergencyStop() { queue.async {
        self.log.notice("Emergency stop requested")
        self.policy.stop(); self.awaitingSafeSequence = nil; self.awaitingSafeAt = nil; self.armSafe = nil; self.pendingDrive = nil
        do { self.priority = try self.packet(self.policy.control("emergency_stop"), characteristic: self.controlID); self.pump() }
        catch { self.close("Emergency stop; reconnect required") }
    } }
    func disarm() { queue.async { self.stop() } }
    private func stop() {
        // Cleanup after a closed link must preserve the original disconnect reason.
        guard peripheral != nil, characteristics[controlID] != nil, policy.session != nil else { policy.stop(); return }
        log.notice("Sending disarm command")
        policy.stop(); awaitingSafeSequence = nil; awaitingSafeAt = nil; armSafe = nil; pendingDrive = nil
        do { priority = try packet(policy.control("disarm"), characteristic: controlID); pump() }
        catch { close("Disarmed; reconnect required") }
    }
    func request(envelope: Data, generation: UInt64) { queue.async {
        guard generation == self.policy.generation else { return }
        do {
            guard self.configuration.count < 16, let object = try JSONSerialization.jsonObject(with: envelope) as? [String: Any],
                  object["schema_version"] as? Int == 1, let id = object["request_id"] as? UInt32,
                  let operation = object["operation"] as? String, !self.usedRequests.contains(id) else { throw BluetoothPolicyError.malformed }
            guard envelope.count <= 4080 else { throw BluetoothPolicyError.malformed }
            let packet = try self.packet(envelope, characteristic: self.controlID, request: id)
            self.usedRequests.insert(id); self.requests[id] = operation; self.retryEnvelopes[id] = envelope
            self.configuration.append(packet); self.pump()
        } catch {
            if let object = try? JSONSerialization.jsonObject(with: envelope) as? [String: Any], let id = object["request_id"] as? UInt32 {
                let reply: [String: Any] = ["schema_version": 1, "request_id": id, "result": "error", "active_revision": self.policy.revision,
                    "errors": [["code": "configuration_request", "message": "Invalid, oversized, or duplicate configuration request"]], "payload": NSNull()]
                if let data = try? JSONSerialization.data(withJSONObject: reply) {
                    DispatchQueue.main.async { self.replyJSON = String(decoding: data, as: UTF8.self) }
                }
            }
            self.publish("Invalid or duplicate configuration request")
        }
    } }
    /// Replays the exact envelope so the rover can return its cached acknowledgment.
    func retryConfiguration(requestID: UInt32, generation: UInt64) { queue.async {
        guard generation == self.policy.generation, self.failedRequests.contains(requestID),
              let envelope = self.retryEnvelopes[requestID], self.configuration.count < 16 else { return }
        do {
            guard let object = try JSONSerialization.jsonObject(with: envelope) as? [String: Any],
                  let operation = object["operation"] as? String else { throw BluetoothPolicyError.malformed }
            let packet = try self.packet(envelope, characteristic: self.controlID, request: requestID)
            self.failedRequests.remove(requestID); self.requests[requestID] = operation
            self.configuration.append(packet); self.pump()
        } catch { self.publish("Configuration retry unavailable; reconnect required") }
    } }

    private func synchronize() {
        guard !synchronizationStarted, characteristics[repliesID]?.isNotifying == true, characteristics[statusID]?.isNotifying == true else { return }
        synchronizationStarted = true
        queueSynchronization("capabilities")
    }

    /// Called after commit or explicit retry. No editable actuator data is fetched here.
    func refreshConfigurationIndex() { queue.async { self.restartProfiles() } }

    private func invalidateProfiles() {
        synchronizedRevision = nil; safeValues = []; profileSync.clear(); policy.invalidateLayout()
        DispatchQueue.main.async { self.isReady = false; self.driveProfileJSON = "{}" }
    }

    private func restartProfiles() {
        guard peripheral != nil, synchronizationStarted else { return }
        synchronizationRound &+= 1
        invalidateProfiles()
        DispatchQueue.main.async { self.configurationReady = false; self.layoutIndexJSON = "{}" }
        if supportsPagination { queueSynchronization("layout_index") }
        else { queueSynchronization("capabilities") }
    }

    private func queueSynchronization(_ operation: String, payload: [String: Any] = [:]) {
        guard requestID < UInt32.max else { close("Request IDs exhausted"); return }
        requestID += 1
        while usedRequests.contains(requestID), requestID < UInt32.max { requestID += 1 }
        do {
            UserDefaults.standard.set(Int(requestID), forKey: "bluetoothSynchronizationRequestID")
            let data = try JSONSerialization.data(withJSONObject: ["schema_version": 1, "request_id": requestID, "operation": operation, "payload": payload] as [String: Any])
            usedRequests.insert(requestID); requests[requestID] = operation
            synchronizationRequests[requestID] = synchronizationRound
            configuration.append(try packet(data, characteristic: controlID, request: requestID))
            pump()
        } catch { close("Configuration encoding failed") }
    }

    private func finishProfiles() throws {
        guard let profile = profileSync.completed, profile.revision == policy.revision else { throw ActuatorProfileError.staleRevision }
        let json = String(decoding: try JSONEncoder().encode(profile), as: UTF8.self)
        _ = try ActuatorDriveProfileSet.decode(json: json)
        safeValues = profile.safeValues; synchronizedRevision = profile.revision
        policy.synchronized(revision: profile.revision)
        let ready = policy.ready
        DispatchQueue.main.async { self.driveProfileJSON = json; self.isReady = ready }
        log.info("Drive profiles synchronized; revision=\(profile.revision) count=\(profile.actuators.count)")
    }

    private func nextProfile() throws {
        guard let revision = profileSync.revision else { throw BluetoothPolicyError.malformed }
        if let id = profileSync.remaining.first {
            queueSynchronization("read_drive_profile", payload: ["expected_revision": revision, "actuator_id": id])
        } else { try finishProfiles() }
    }

    private func configurationFailed(_ message: String, code: String = "configuration_transfer") {
        guard let id = waitingRequest else { return }
        let operation = requests.removeValue(forKey: id)
        let sync = synchronizationRequests.removeValue(forKey: id)
        if sync == nil, retryEnvelopes[id] != nil { failedRequests.insert(id) }
        waitingRequest = nil; waitingRequestAt = nil; replyAssembly.clear()
        if active?.request == id { active?.chunks.removeAll(); if writeAt == nil { active = nil } }
        log.error("Configuration transfer failed; operation=\(operation ?? "unknown", privacy: .public) request=\(id): \(message, privacy: .public)")
        let reply: [String: Any] = ["schema_version": 1, "request_id": id, "result": "error", "active_revision": policy.revision,
            "errors": [["code": code, "message": message]], "payload": NSNull()]
        if let data = try? JSONSerialization.data(withJSONObject: reply) {
            DispatchQueue.main.async { self.replyJSON = String(decoding: data, as: UTF8.self) }
        }
        if sync != nil { invalidateProfiles(); DispatchQueue.main.async { self.configurationReady = false } }
        publish(message + " · retry configuration")
        pump()
    }

    private func packet(_ bytes: Data, characteristic: CBUUID, producedAt: TimeInterval? = nil, request: UInt32? = nil) throws -> Packet {
        guard let p = peripheral, characteristics[characteristic] != nil else { throw BluetoothPolicyError.unavailable }
        let occupied = Set(([active, pendingDrive, priority, armSafe].compactMap { $0?.id }) + configuration.map { $0.id })
        repeat { messageID &+= 1 } while occupied.contains(messageID)
        let chunks = try actuatorFragment(messageId: messageID, payload: bytes, maximumWriteLength: UInt32(p.maximumWriteValueLength(for: .withResponse)))
        return Packet(chunks: chunks, characteristic: characteristic, id: messageID, request: request, producedAt: producedAt)
    }
    private func pump() {
        guard writeAt == nil, let p = peripheral else { return }
        // Disarm cancels remaining drive fragments at the next ATT boundary.
        if priority != nil { active = priority; priority = nil }
        if active == nil {
            if let safe = armSafe { active = safe; armSafe = nil }
            else if awaitingSafeSequence != nil { return }
            else if let drive = pendingDrive { active = drive; pendingDrive = nil }
            else if waitingRequest == nil, !configuration.isEmpty {
                let request = configuration.removeFirst(); active = request
                waitingRequest = request.request; waitingRequestAt = now
            }
        }
        guard var packet = active else { return }
        if let produced = packet.producedAt, now < produced || now - produced >= 0.1 { active = nil; stop(); return }
        guard let characteristic = characteristics[packet.characteristic], !packet.chunks.isEmpty else { active = nil; return }
        let bytes = packet.chunks.removeFirst(); active = packet; writeAt = now
        p.writeValue(bytes, for: characteristic, type: .withResponse)
    }
    private func tick() {
        guard peripheral != nil else { return }
        if let started = attemptStarted, now - started >= 15 { close("Connection timed out"); return }
        guard !setup else { return }
        if let started = awaitingSafeAt, now - started >= 0.3 { stop(); return }
        // ATT acknowledgement is transport liveness, not motion freshness.
        // Packet age remains limited to 100 ms; the rover watchdog remains 200 ms.
        let writeDeadline: TimeInterval = 2.0
        if let started = writeAt, now - started >= writeDeadline { close("Bluetooth write stalled; reconnect required"); return }
        // Motion paths already require fresh status (policy.stale, 0.3 s). This
        // watchdog is link liveness only: the rover blocks its worker loop while
        // fsyncing staged/committed layouts, so brief gaps must not disconnect.
        if let age = policy.statusAge(now: now), age >= 2.0 {
            let callbackAge = lastStatusCallbackAt.map { String(format: "%.2fs", now - $0) } ?? "none"
            let detail = "last status callback: \(callbackAge), accepted: \(acceptedStatusCount)"
            log.error("Status timeout: \(detail, privacy: .public)")
            close("Bluetooth status stalled; reconnect required (\(detail))"); return
        }
        if let started = waitingRequestAt, now - started >= 5 {
            configurationFailed("Configuration transfer timed out", code: "configuration_timeout"); return
        }
        // Replies use application-fragmented notifications. Polling raw reads
        // can truncate large capabilities documents and mix read bytes into the assembler.
        pump()
    }
    func centralManagerDidUpdateState(_ central: CBCentralManager) {
        let radio: TerraConnectionFacts.Radio
        switch central.state {
        case .poweredOn: radio = .ready
        case .unauthorized: radio = .unauthorized
        case .poweredOff, .unsupported: radio = .off
        default: radio = .unknown
        }
        DispatchQueue.main.async { self.radioState = radio }
        if central.state == .poweredOn { startRequestedScan() }
        else { close(central.state == .unauthorized ? "Bluetooth permission required" : "Bluetooth unavailable") }
    }
    func centralManager(_ central: CBCentralManager, didDiscover p: CBPeripheral, advertisementData: [String: Any], rssi RSSI: NSNumber) {
        guard scanningRequested else { return }
        recordConnection("Discovered rover advertisement")
        found[p.identifier] = p
        let localName = advertisementData[CBAdvertisementDataLocalNameKey] as? String ?? advertisedNames[p.identifier] ?? ""
        let pairing = localName.hasPrefix("terra-") && localName.hasSuffix("-pair")
        let displayName = pairing ? String(localName.dropLast(5)) : (localName.isEmpty ? p.name ?? "Terra rover" : localName)
        let generation = automationGeneration
        DispatchQueue.main.async { self.onDiscovery?(.init(identifier: p.identifier, name: displayName, pairing: pairing), generation) }
        if automaticSelection != nil {
            if preferredRover == p.identifier {
                selectAutomaticRover()
            } else if discoveryWindow == nil {
                // Collect nearby candidates before selecting a first-time rover.
                let window = DispatchWorkItem { [weak self] in
                    self?.discoveryWindow = nil; self?.selectAutomaticRover()
                }
                discoveryWindow = window
                queue.asyncAfter(deadline: .now() + 2, execute: window)
            }
        }
        advertisedNames[p.identifier] = localName.isEmpty ? p.name ?? "Terra peripheral" : localName
        let items = found.values.map { ["identifier": $0.identifier.uuidString, "name": advertisedNames[$0.identifier] ?? $0.name ?? "Terra peripheral"] }
        if let data = try? JSONSerialization.data(withJSONObject: items) { DispatchQueue.main.async { self.peripheralsJSON = String(decoding: data, as: UTF8.self) } }
    }
    func centralManager(_ central: CBCentralManager, didConnect p: CBPeripheral) { guard current(p) else { return }; p.discoverServices([serviceID]) }
    func centralManager(_ central: CBCentralManager, didFailToConnect p: CBPeripheral, error: Error?) { if retiring === p { p.delegate = nil; retiring = nil; connectDeferredIfPossible(); return }; guard current(p) else { return }; terminal("Connection failed; reconnect explicitly") }
    func centralManager(_ central: CBCentralManager, didDisconnectPeripheral p: CBPeripheral, error: Error?) { if retiring === p { p.delegate = nil; retiring = nil; connectDeferredIfPossible(); return }; guard current(p) else { return }; terminal("Disconnected; explicit reconnect and arm required") }
    func peripheral(_ p: CBPeripheral, didDiscoverServices error: Error?) {
        guard current(p) else { return }
        guard error == nil, let service = p.services?.first(where: { $0.uuid == serviceID }) else { close("Terra service unavailable"); return }
        p.discoverCharacteristics([driveID, statusID, controlID, repliesID], for: service)
    }
    func peripheral(_ p: CBPeripheral, didDiscoverCharacteristicsFor service: CBService, error: Error?) {
        guard current(p) else { return }
        guard error == nil else { close("Characteristic discovery failed"); return }
        for c in service.characteristics ?? [] { characteristics[c.uuid] = c }
        guard let status = characteristics[statusID] else { close("Status unavailable"); return }
        setup = characteristics[driveID] == nil && characteristics[controlID] == nil && characteristics[repliesID] == nil
        admitting = true; p.readValue(for: status); publish(setup ? "Accept Bluetooth pairing on your phone" : "Authenticating owner")
    }
    func peripheral(_ p: CBPeripheral, didUpdateNotificationStateFor c: CBCharacteristic, error: Error?) {
        guard current(p), characteristics[c.uuid] === c else { return }
        guard error == nil, c.isNotifying else { close("Status subscription failed"); return }; synchronize()
    }
    func peripheral(_ p: CBPeripheral, didWriteValueFor c: CBCharacteristic, error: Error?) {
        guard current(p), characteristics[c.uuid] === c, writeAt != nil, active?.characteristic == c.uuid else { return }
        writeAt = nil
        guard error == nil else {
            if active?.request != nil {
                if waitingRequest != nil { configurationFailed("Configuration write rejected") }
                else { active = nil; pump() }
                return
            }
            close("Bluetooth write failed; reconnect required"); return
        }
        if active?.chunks.isEmpty == true { active = nil }
        // ATT acknowledgement does not update armed/applied/accepted state.
        pump()
    }
    func peripheral(_ p: CBPeripheral, didUpdateValueFor c: CBCharacteristic, error: Error?) {
        guard current(p), characteristics[c.uuid] === c else { return }
        if c.uuid == statusID { lastStatusCallbackAt = now }
        guard error == nil, let bytes = c.value else {
            let detail = error.map { "\(($0 as NSError).domain) \(($0 as NSError).code): \($0.localizedDescription)" } ?? "empty value"
            log.error("Read failed for \(c.uuid.uuidString, privacy: .public): \(detail, privacy: .public)")
            close("Encrypted read/notification failed: \(detail)"); return
        }
        do {
            // CoreBluetooth delivers a completed ATT long read as one raw JSON value.
            let document: Data?
            if bytes.count <= 16384, (try? JSONSerialization.jsonObject(with: bytes)) is [String: Any] { document = bytes }
            else if c.uuid == statusID { document = try statusAssembly.push(bytes, now: now) }
            else if c.uuid == repliesID { document = try replyAssembly.push(bytes, now: now) }
            else { return }
            guard let document = document, let object = try JSONSerialization.jsonObject(with: document) as? [String: Any], object["schema_version"] as? Int == 1 else { return }
            let text = String(decoding: document, as: UTF8.self)
            if setup {
                guard object["type"] as? String == "setup", object["pairing_confirmed"] as? Bool == true else { throw BluetoothPolicyError.malformed }
                emit(.paired(p.identifier))
                suppressCloseEvent = true; close("Pairing saved · waiting for rover startup"); suppressCloseEvent = false; return
            }
            if c.uuid == statusID {
                if object["type"] as? String == "status" {
                    let safetySummary = "armed=\(object["armed"] ?? "missing") arming=\(object["arming"] ?? "missing") stop_reason=\(object["stop_reason"] ?? "missing") fault=\(object["fault"] ?? "none") gate=\(object["gate_mode"] ?? "missing") configuration_allowed=\(object["configuration_allowed"] ?? "missing")"
                    if safetySummary != lastLoggedSafety {
                        lastLoggedSafety = safetySummary
                        log.notice("Rover safety: \(safetySummary, privacy: .public)")
                    }
                    let incomingRevision = object["active_revision"] as? UInt32
                    let revisionChanged = lastObservedRevision != nil && lastObservedRevision != incomingRevision
                    lastObservedRevision = incomingRevision
                    let effects = policy.status(object, now: now)
                    if revisionChanged, synchronizationStarted { restartProfiles() }
                    if effects.isEmpty {
                        log.error("Status rejected: revision=\(String(describing: object["active_revision"]), privacy: .public), bytes=\(bytes.count)")
                    } else { acceptedStatusCount += 1 }
                    for effect in effects { if case .state(let text) = effect { publish(text) } }
                    if hasCapabilities, let revision = synchronizedRevision { policy.synchronized(revision: revision) }
                    let configReady = hasCapabilities && supportsPagination && policy.session != nil && profileSync.revision == policy.revision
                    let ready = policy.ready, armed = policy.armed, arming = policy.arming
                    DispatchQueue.main.async { self.configurationReady = configReady; self.isReady = ready; self.armed = armed; self.arming = arming }
                    DispatchQueue.main.async { self.statusJSON = text; self.connectedIdentifier = p.identifier }
                    if admitting {
                        guard !effects.isEmpty else { throw BluetoothPolicyError.malformed }
                        attemptStarted = nil
                        emit(.authenticated(p.identifier))
                        admitting = false
                        guard let replies = characteristics[repliesID] else { throw BluetoothPolicyError.unavailable }
                        p.setNotifyValue(true, for: c); p.setNotifyValue(true, for: replies)
                        // iOS may retain subscription state across app relaunch and
                        // omit a notification-state callback for an already active subscription.
                        synchronize()
                    }
                } else if object["type"] as? String == "command_acceptance" {
                    if object["session"] as? UInt32 == policy.session, object["sequence"] as? UInt32 == awaitingSafeSequence, awaitingSafeSequence != nil {
                        awaitingSafeSequence = nil; awaitingSafeAt = nil
                        guard object["accepted"] as? Bool == true else { stop(); return }
                        priority = try packet(policy.arm(now: now), characteristic: controlID); pump()
                    }
                    // Kept distinct from periodic state; UI may inspect this event.
                    DispatchQueue.main.async { self.replyJSON = text }
                }
            } else if c.uuid == repliesID {
                guard let id = object["request_id"] as? UInt32, id == waitingRequest, let operation = requests.removeValue(forKey: id) else { return }
                waitingRequest = nil; waitingRequestAt = nil; replyAssembly.clear()
                let round = synchronizationRequests.removeValue(forKey: id)
                retryEnvelopes.removeValue(forKey: id); failedRequests.remove(id)
                if let round, round != synchronizationRound { pump(); return }
                DispatchQueue.main.async { self.replyJSON = text }
                guard object["result"] as? String == "ok" else {
                    if round != nil {
                        invalidateProfiles()
                        DispatchQueue.main.async { self.configurationReady = false }
                        if (object["errors"] as? [[String: Any]])?.contains(where: { $0["code"] as? String == "stale_revision" }) == true {
                            restartProfiles()
                        }
                    }
                    publish("Configuration rejected: \(object["errors"] ?? [])")
                    pump(); return
                }
                if let round, round == synchronizationRound, let payload = object["payload"] as? [String: Any] {
                    switch operation {
                    case "capabilities":
                        capabilityObject = payload; hasCapabilities = true
                        supportsPagination = (payload["features"] as? [String])?.contains("actuator_pagination_v1") == true
                        let data = try JSONSerialization.data(withJSONObject: payload)
                        DispatchQueue.main.async { self.capabilitiesJSON = String(decoding: data, as: UTF8.self) }
                        if supportsPagination { queueSynchronization("layout_index") }
                        else {
                            invalidateProfiles()
                            DispatchQueue.main.async { self.configurationReady = false }
                            publish("Update rover service for actuator pagination")
                        }
                    case "layout_index":
                        guard let revision = payload["revision"] as? UInt32, revision == policy.revision,
                              let entries = payload["actuators"] as? [[String: Any]],
                              let count = payload["actuator_count"] as? Int, count == entries.count,
                              let available = payload["layout_available"] as? Bool else { throw ActuatorProfileError.staleRevision }
                        let ids = try entries.map { entry -> Int in
                            guard let id = entry["id"] as? Int, entry["name"] is String, entry["kind"] is String else { throw BluetoothPolicyError.malformed }
                            return id
                        }
                        let data = try JSONSerialization.data(withJSONObject: payload)
                        try profileSync.begin(revision: revision, ids: ids)
                        DispatchQueue.main.async { self.layoutIndexJSON = String(decoding: data, as: UTF8.self); self.configurationReady = true }
                        if available { try nextProfile() }
                        else { publish("No active layout · add actuators while disarmed") }
                    case "read_drive_profile":
                        guard let revision = payload["revision"] as? UInt32, revision == policy.revision,
                              let entry = payload["actuator"] as? [String: Any] else { throw ActuatorProfileError.staleRevision }
                        let data = try JSONSerialization.data(withJSONObject: entry)
                        let actuator = try JSONDecoder().decode(ActuatorDriveProfile.self, from: data)
                        try profileSync.accept(revision: revision, actuator: actuator)
                        try nextProfile()
                    default: break
                    }
                }
                pump()
            }
        } catch ActuatorProfileError.staleRevision {
            restartProfiles()
        } catch {
            if c.uuid == repliesID, waitingRequest != nil {
                configurationFailed("Invalid or incomplete configuration reply"); return
            }
            log.error("Decode failed for \(c.uuid.uuidString, privacy: .public): \(String(describing: error), privacy: .public)")
            close("Malformed Bluetooth message; reconnect required")
        }
    }
}
