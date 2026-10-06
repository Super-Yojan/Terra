import Foundation
import Combine
import CoreBluetooth

final class BluetoothLink: NSObject, ObservableObject, CBCentralManagerDelegate, CBPeripheralDelegate {
    @Published private(set) var isReady = false
    @Published private(set) var armed = false
    @Published private(set) var arming = false
    @Published private(set) var status = "Disconnected"
    @Published private(set) var peripheralsJSON = "[]"
    @Published private(set) var statusJSON = "{}"
    @Published private(set) var capabilitiesJSON = "{}"
    @Published private(set) var layoutJSON = "{}"
    @Published private(set) var replyJSON = "{}"
    @Published private(set) var connectedIdentifier: UUID?
    private let queue = DispatchQueue(label: "terra.bluetooth")
    private var central: CBCentralManager!
    private var found: [UUID: CBPeripheral] = [:]
    private var peripheral: CBPeripheral?
    private var retiring: CBPeripheral?
    private var synchronizedRevision: UInt32?
    private var hasCapabilities = false
    private var safeValues: [[String: Any]] = []
    private var awaitingSafeSequence: UInt32?
    private var awaitingSafeAt: TimeInterval?
    private var usedRequests: Set<UInt32> = []
    private var replyReadAt: TimeInterval = 0
    private let policy = BluetoothSession()
    private var callbackGeneration: UInt64 = 0
    private var characteristics: [CBUUID: CBCharacteristic] = [:]
    private var statusAssembly = BluetoothJSONAssembler()
    private var replyAssembly = BluetoothJSONAssembler()
    private var admitting = false
    private var setup = false
    private var timer: DispatchSourceTimer?
    private var messageID: UInt16 = 0
    private var requestID: UInt32 = 0
    private var requests: [UInt32: String] = [:]
    private var configuration: [Packet] = []
    private var priority: Packet?
    private var pendingDrive: Packet?
    private var active: Packet?
    private var writeAt: TimeInterval?
    private struct Packet { var chunks: [Data]; let characteristic: CBUUID; let id: UInt16; let producedAt: TimeInterval? }
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
    private func publish(_ text: String) { DispatchQueue.main.async { self.status = text } }
    func scan() { queue.async { guard self.central.state == .poweredOn else { return }; self.central.scanForPeripherals(withServices: [self.serviceID]); self.publish("Scanning") } }
    func connect(identifier: UUID) { queue.async {
        guard self.peripheral == nil, self.retiring == nil, let selected = self.found[identifier] else { return }
        self.clear(); self.peripheral = selected; self.callbackGeneration = self.policy.generation
        selected.delegate = self; self.central.stopScan(); self.central.connect(selected); self.publish("Connecting")
    } }
    func disconnect() { queue.async { self.close("Disconnected") } }
    private func clear() {
        policy.reset(); usedRequests.removeAll(); awaitingSafeAt = nil; safeValues = []; awaitingSafeSequence = nil; synchronizedRevision = nil; hasCapabilities = false; characteristics.removeAll(); statusAssembly.clear(); replyAssembly.clear()
        active = nil; pendingDrive = nil; priority = nil; configuration.removeAll(); requests.removeAll()
        writeAt = nil; admitting = false; setup = false
        DispatchQueue.main.async { self.isReady = false; self.armed = false; self.arming = false; self.connectedIdentifier = nil; self.capabilitiesJSON = "{}"; self.layoutJSON = "{}"; self.statusJSON = "{}" }
    }
    private func close(_ reason: String) { let old = peripheral; peripheral = nil; clear(); if let old = old { retiring = old; central.cancelPeripheralConnection(old) }; publish(reason) }
    private func current(_ p: CBPeripheral) -> Bool { peripheral === p && callbackGeneration == policy.generation }
    func send(frame: Data, producedAt: TimeInterval) { queue.async {
        for effect in self.policy.drive(frame, producedAt: producedAt, now: self.now) {
            if case .send(let bytes) = effect { do { self.pendingDrive = try self.packet(bytes, characteristic: self.driveID, producedAt: producedAt); self.pump() } catch { self.close("Invalid drive frame") } }
        }
    } }
    func drive(valuesJSON: String, producedAt: TimeInterval) { queue.async {
        do {
            guard self.now >= producedAt, self.now - producedAt < 0.1 else { return }
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
        do {
            guard self.policy.ready, self.awaitingSafeSequence == nil else { throw BluetoothPolicyError.unavailable }
            let produced = self.now
            let safe = try self.policy.frame(values: self.safeValues)
            for effect in self.policy.drive(safe, producedAt: produced, now: self.now) {
                if case .send(let bytes) = effect { self.awaitingSafeSequence = self.policy.sequence; self.awaitingSafeAt = produced; self.pendingDrive = try self.packet(bytes, characteristic: self.driveID, producedAt: produced) }
            }; self.pump()
        } catch { self.publish("Synchronize layout and safe output before arming") }
    } }
    func disarm() { queue.async { self.stop() } }
    private func stop() {
        policy.stop(); awaitingSafeSequence = nil; awaitingSafeAt = nil; pendingDrive = nil
        do { priority = try packet(policy.control("disarm"), characteristic: controlID); pump() }
        catch { close("Disarmed; reconnect required") }
    }
    func request(envelope: Data) { queue.async {
        do {
            guard self.configuration.count < 16, let object = try JSONSerialization.jsonObject(with: envelope) as? [String: Any],
                  object["schema_version"] as? Int == 1, let id = object["request_id"] as? UInt32,
                  let operation = object["operation"] as? String, !self.usedRequests.contains(id) else { throw BluetoothPolicyError.malformed }
            self.usedRequests.insert(id); self.requests[id] = operation
            self.configuration.append(try self.packet(envelope, characteristic: self.controlID)); self.pump()
        } catch { self.publish("Invalid or duplicate configuration request") }
    } }
    private func synchronize() {
        guard requests.isEmpty, characteristics[repliesID]?.isNotifying == true, characteristics[statusID]?.isNotifying == true else { return }
        for operation in ["capabilities", "read_layout"] {
            guard requestID < UInt32.max else { close("Request IDs exhausted"); return }; requestID += 1
            while usedRequests.contains(requestID), requestID < UInt32.max { requestID += 1 }
            do {
                let data = try JSONSerialization.data(withJSONObject: ["schema_version": 1, "request_id": requestID, "operation": operation, "payload": [:]] as [String: Any])
                usedRequests.insert(requestID); requests[requestID] = operation; configuration.append(try packet(data, characteristic: controlID))
            } catch { close("Configuration encoding failed"); return }
        }; pump()
    }
    private func packet(_ bytes: Data, characteristic: CBUUID, producedAt: TimeInterval? = nil) throws -> Packet {
        guard let p = peripheral, characteristics[characteristic] != nil else { throw BluetoothPolicyError.unavailable }
        let occupied = Set(([active, pendingDrive, priority].compactMap { $0?.id }) + configuration.map { $0.id })
        repeat { messageID &+= 1 } while occupied.contains(messageID)
        let chunks = try actuatorFragment(messageId: messageID, payload: bytes, maximumWriteLength: UInt32(p.maximumWriteValueLength(for: .withResponse)))
        return Packet(chunks: chunks, characteristic: characteristic, id: messageID, producedAt: producedAt)
    }
    private func pump() {
        guard writeAt == nil, let p = peripheral else { return }
        // Disarm cancels remaining drive fragments at the next ATT boundary.
        if priority != nil { active = priority; priority = nil }
        if active == nil {
            if let drive = pendingDrive { active = drive; pendingDrive = nil }
            else if !configuration.isEmpty { active = configuration.removeFirst() }
        }
        guard var packet = active else { return }
        if let produced = packet.producedAt, now < produced || now - produced >= 0.1 { active = nil; stop(); return }
        guard let characteristic = characteristics[packet.characteristic], !packet.chunks.isEmpty else { active = nil; return }
        let bytes = packet.chunks.removeFirst(); active = packet; writeAt = now
        p.writeValue(bytes, for: characteristic, type: .withResponse)
    }
    private func tick() {
        guard peripheral != nil, !setup else { return }
        if let started = awaitingSafeAt, now - started >= 0.3 { stop(); return }
        if let started = writeAt, now - started >= 0.1 { close("Bluetooth write stalled; reconnect required"); return }
        if policy.stale(now: now) { close("Bluetooth status stalled; reconnect required"); return }
        if !requests.isEmpty, now - replyReadAt >= 0.2, let replies = characteristics[repliesID], replies.isNotifying {
            replyReadAt = now; peripheral?.readValue(for: replies)
        }
        pump()
    }
    func centralManagerDidUpdateState(_ central: CBCentralManager) { if central.state != .poweredOn { close("Bluetooth unavailable") } }
    func centralManager(_ central: CBCentralManager, didDiscover p: CBPeripheral, advertisementData: [String: Any], rssi RSSI: NSNumber) {
        found[p.identifier] = p
        let items = found.values.map { ["identifier": $0.identifier.uuidString, "name": $0.name ?? "Terra peripheral"] }
        if let data = try? JSONSerialization.data(withJSONObject: items) { DispatchQueue.main.async { self.peripheralsJSON = String(decoding: data, as: UTF8.self) } }
    }
    func centralManager(_ central: CBCentralManager, didConnect p: CBPeripheral) { guard current(p) else { return }; p.discoverServices([serviceID]) }
    func centralManager(_ central: CBCentralManager, didFailToConnect p: CBPeripheral, error: Error?) { guard current(p) else { return }; close("Connection failed") }
    func centralManager(_ central: CBCentralManager, didDisconnectPeripheral p: CBPeripheral, error: Error?) { if retiring === p { retiring = nil; return }; guard current(p) else { return }; close("Disconnected; explicit reconnect and arm required") }
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
        admitting = true; p.readValue(for: status); publish(setup ? "Confirm pairing code on phone and Pi" : "Authenticating owner")
    }
    func peripheral(_ p: CBPeripheral, didUpdateNotificationStateFor c: CBCharacteristic, error: Error?) {
        guard current(p), characteristics[c.uuid] === c else { return }
        guard error == nil, c.isNotifying else { close("Status subscription failed"); return }; synchronize()
    }
    func peripheral(_ p: CBPeripheral, didWriteValueFor c: CBCharacteristic, error: Error?) {
        guard current(p), characteristics[c.uuid] === c, writeAt != nil, active?.characteristic == c.uuid else { return }
        writeAt = nil
        guard error == nil else { close("Bluetooth write failed; reconnect required"); return }
        if active?.chunks.isEmpty == true { active = nil }
        // ATT acknowledgement does not update armed/applied/accepted state.
        pump()
    }
    func peripheral(_ p: CBPeripheral, didUpdateValueFor c: CBCharacteristic, error: Error?) {
        guard current(p), characteristics[c.uuid] === c else { return }
        guard error == nil, let bytes = c.value else { close("Encrypted read/notification failed"); return }
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
                close("Pairing saved. Start normal Pi service, then scan and reconnect explicitly."); return
            }
            if c.uuid == statusID {
                if object["type"] as? String == "status" {
                    for effect in policy.status(object, now: now) { if case .state(let text) = effect { publish(text) } }
                    if hasCapabilities, let revision = synchronizedRevision { policy.synchronized(revision: revision) }
                    let ready = policy.ready, armed = policy.armed, arming = policy.arming
                    DispatchQueue.main.async { self.isReady = ready; self.armed = armed; self.arming = arming }
                    DispatchQueue.main.async { self.statusJSON = text; self.connectedIdentifier = p.identifier }
                    if admitting {
                        admitting = false
                        guard let replies = characteristics[repliesID] else { throw BluetoothPolicyError.unavailable }
                        p.setNotifyValue(true, for: c); p.setNotifyValue(true, for: replies)
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
                guard let id = object["request_id"] as? UInt32, let operation = requests.removeValue(forKey: id) else { return }
                DispatchQueue.main.async { self.replyJSON = text }
                if object["result"] as? String == "ok", let payload = object["payload"], let data = try? JSONSerialization.data(withJSONObject: payload) {
                    let json = String(decoding: data, as: UTF8.self)
                    if operation == "capabilities" { hasCapabilities = true; DispatchQueue.main.async { self.capabilitiesJSON = json } }
                    if operation == "read_layout", let layout = payload as? [String: Any], let revision = layout["revision"] as? UInt32 {
                        guard let actuators = layout["actuators"] as? [[String: Any]] else { throw BluetoothPolicyError.malformed }
                        safeValues = try actuators.map { actuator in
                            guard let id = actuator["id"], let safe = actuator["safe"] as? [String: Any], let limits = actuator["limits"] as? [String: Any], let min = limits["min"] as? Double, let max = limits["max"] as? Double else { throw BluetoothPolicyError.malformed }
                            let target = (safe["value"] as? Double) ?? 0
                            return ["id": id, "value": Swift.min(max, Swift.max(min, target))]
                        }
                        synchronizedRevision = revision; if hasCapabilities { policy.synchronized(revision: revision) }; DispatchQueue.main.async { self.layoutJSON = json }
                    }
                }
            }
        } catch { close("Malformed Bluetooth message; reconnect required") }
    }
}
