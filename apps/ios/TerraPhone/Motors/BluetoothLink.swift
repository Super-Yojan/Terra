import Combine
import CoreBluetooth
import Foundation

/// Execution output: sends wheel commands to the rover's Raspberry Pi over
/// Bluetooth. Handles pairing, reading the actuator layout, and streaming
/// drive frames. All state lives on `queue`; published values on main.
final class BluetoothLink: NSObject, ObservableObject, CBCentralManagerDelegate, CBPeripheralDelegate {
    struct Rover: Identifiable, Equatable { let id: UUID; let name: String; let pairing: Bool }

    @Published private(set) var status = "Disconnected"
    @Published private(set) var rovers: [Rover] = []
    @Published private(set) var connected: UUID?
    /// Layout read and relay connected: drive frames will be accepted.
    @Published private(set) var ready = false
    /// Ports the rover advertises (`capabilities`), and pins already in use.
    @Published private(set) var ports: [RoverPort] = []
    @Published private(set) var occupied: Set<String> = []
    /// Vehicle geometry, wiring, and phone mount stored on the connected Pi.
    @Published private(set) var vehicleSetup: VehicleSetup?
    /// Whether the phone remembers a rover to auto-reconnect to.
    @Published private(set) var hasRememberedRover: Bool
    /// Progress of `applyLayout`.
    @Published private(set) var layoutApply: LayoutApply = .idle

    enum LayoutApply: Equatable { case idle, working(String), failed(String), done(UInt32) }

    private static func uuid(_ suffix: String) -> CBUUID { CBUUID(string: "7e5a00\(suffix)-4c2b-4f91-9e3a-1d8c6b2a0f10") }
    private let serviceID = BluetoothLink.uuid("10"), driveID = BluetoothLink.uuid("11"), statusID = BluetoothLink.uuid("12"),
        controlID = BluetoothLink.uuid("13"), repliesID = BluetoothLink.uuid("14")
    private let savedKey = "terra.rover.id"

    private let queue = DispatchQueue(label: "terra.bluetooth")
    private var central: CBCentralManager!
    private var peripheral: CBPeripheral?
    private var chars: [CBUUID: CBCharacteristic] = [:]
    private var found: [UUID: CBPeripheral] = [:]
    private var wanted: UUID? = UserDefaults.standard.string(forKey: "terra.rover.id").flatMap(UUID.init)
    private var pairingMode = false
    private var authenticated = false

    // Relay state from status notifications.
    private var relayConnected = false
    private var revision: UInt32 = 0
    private var profiles: ActuatorDriveProfileSet?
    private var sync = ActuatorProfileSync()
    private var piSupportsVehicleSetup = false

    // Request/reply exchange for layout reads (one at a time).
    private var requestID = UInt32(clamping: max(0x8000_0000, UserDefaults.standard.integer(forKey: "terra.rover.requestID")))
    private var waiting: (id: UInt32, operation: String, payload: [String: Any])?
    private var messageID: UInt16 = 0
    private var statusAssembly = FragmentAssembler()
    private var replyAssembly = FragmentAssembler(timeout: 2, limit: 4080)

    // Layout edit transaction (begin → stage actuators → stage vehicle setup → validate → commit).
    private struct LayoutEdit {
        var token = "", version: UInt32 = 0, base: UInt32 = 0
        var remaining: [[String: Any]]
        let names: [Int: String]
        let vehicle: [String: Any]?
        let setup: VehicleSetup?
        var vehicleStaged = false
        var committing = false
    }
    private var edit: LayoutEdit?
    private static let layoutOperations: Set<String> = ["begin_layout_edit", "stage_actuator", "stage_vehicle_setup",
                                                        "validate_layout_edit", "commit_layout_edit", "discard_layout_edit"]

    // Drive writes: one in flight, newest pending wins.
    private var writing = false
    private var pending: [Data] = []
    private var queuedWrites: [(Data, CBUUID)] = []

    override init() {
        hasRememberedRover = UserDefaults.standard.string(forKey: savedKey) != nil
        super.init()
        central = CBCentralManager(delegate: self, queue: queue)
    }

    // MARK: API

    func scan() { queue.async { self.startScan() } }

    func connect(_ id: UUID) { queue.async {
        self.wanted = id
        UserDefaults.standard.set(id.uuidString, forKey: self.savedKey)
        DispatchQueue.main.async { self.hasRememberedRover = true }
        self.connectWantedIfFound()
    } }

    func disconnect() { queue.async {
        self.wanted = nil
        UserDefaults.standard.removeObject(forKey: self.savedKey)
        DispatchQueue.main.async { self.hasRememberedRover = false }
        self.close("Disconnected")
    } }

    /// Replace the rover's actuator layout and saved vehicle setup on the Pi.
    /// Progress and errors are published in `layoutApply`; the new drive
    /// profiles and vehicle setup are read automatically once the rover commits.
    func applyLayout(_ actuators: [[String: Any]], vehicleSetup: VehicleSetup? = nil) { queue.async {
        guard self.peripheral != nil, self.authenticated, self.chars[self.controlID] != nil else {
            self.setApply(.failed("Not connected to a rover")); return
        }
        let names = Dictionary(actuators.compactMap { a in (a["id"] as? Int).map { ($0, a["name"] as? String ?? "#\($0)") } },
                               uniquingKeysWith: { a, _ in a })
        self.edit = LayoutEdit(base: self.revision, remaining: actuators, names: names,
                               vehicle: vehicleSetup?.jsonObject(), setup: vehicleSetup)
        self.setApply(.working("Starting"))
        self.request("begin_layout_edit", payload: ["expected_revision": self.revision, "mode": "replace"])
    } }

    func resetLayoutApply() { DispatchQueue.main.async { self.layoutApply = .idle } }

    private func setApply(_ state: LayoutApply) { DispatchQueue.main.async { self.layoutApply = state } }

    /// Send one wheel command; `.stop` sends every actuator's safe value.
    func send(_ command: WheelCommand) { queue.async {
        guard let profiles = self.profiles, self.relayConnected, let p = self.peripheral else { return }
        // The rover rejects drive writes while it handles a control request
        // (and stops its outputs anyway); a rejected write would drop the link.
        guard !self.controlBusy else { self.pending = []; return }
        let values = command == .stop ? profiles.neutral : profiles.values(for: command)
        guard let bytes = try? profiles.encode(values) else { return }
        let capacity = p.maximumWriteValueLength(for: .withResponse) / 2 * 2
        guard capacity >= 2 else { return }
        self.pending = stride(from: 0, to: bytes.count, by: capacity).map { Data(bytes[$0..<min($0 + capacity, bytes.count)]) }
        self.pump()
    } }

    // MARK: Connection

    private func publish(_ text: String) { DispatchQueue.main.async { self.status = text } }

    private func startScan() {
        guard central.state == .poweredOn else { publish("Waiting for Bluetooth"); return }
        found.removeAll()
        DispatchQueue.main.async { self.rovers = [] }
        central.scanForPeripherals(withServices: [serviceID], options: [CBCentralManagerScanOptionAllowDuplicatesKey: false])
        publish("Scanning")
    }

    private func connectWantedIfFound() {
        guard peripheral == nil, let id = wanted, let p = found[id] else { return }
        central.stopScan()
        peripheral = p; p.delegate = self
        central.connect(p)
        publish("Connecting")
    }

    private func close(_ reason: String) {
        if let p = peripheral { central.cancelPeripheralConnection(p) }
        peripheral = nil; chars.removeAll(); pairingMode = false; authenticated = false
        relayConnected = false; revision = 0; profiles = nil; sync.clear(); piSupportsVehicleSetup = false; waiting = nil
        writing = false; pending = []; queuedWrites = []; statusAssembly.clear(); replyAssembly.clear()
        if edit != nil { edit = nil; setApply(.failed("Disconnected before the layout was saved")) }
        DispatchQueue.main.async { self.connected = nil; self.ready = false; self.ports = []; self.occupied = []; self.vehicleSetup = nil }
        publish(reason)
        // Reconnect to the remembered rover when it shows up again.
        if wanted != nil { queue.asyncAfter(deadline: .now() + 2) { if self.peripheral == nil { self.startScan() } } }
    }

    func centralManagerDidUpdateState(_ central: CBCentralManager) {
        if central.state == .poweredOn { startScan() } else { close("Bluetooth unavailable") }
    }

    func centralManager(_ central: CBCentralManager, didDiscover p: CBPeripheral, advertisementData: [String: Any], rssi: NSNumber) {
        let name = advertisementData[CBAdvertisementDataLocalNameKey] as? String ?? p.name ?? "Terra rover"
        let pairing = name.hasPrefix("terra-") && name.hasSuffix("-pair")
        found[p.identifier] = p
        let rover = Rover(id: p.identifier, name: pairing ? String(name.dropLast(5)) : name, pairing: pairing)
        DispatchQueue.main.async {
            if let i = self.rovers.firstIndex(where: { $0.id == rover.id }) { self.rovers[i] = rover } else { self.rovers.append(rover) }
        }
        connectWantedIfFound()
    }

    private func describe(_ error: Error) -> String {
        let ns = error as NSError
        return "\(ns.localizedDescription) (\(ns.domain) \(ns.code))"
    }

    func centralManager(_ central: CBCentralManager, didConnect p: CBPeripheral) {
        guard p === peripheral else { return }
        p.discoverServices([serviceID])
    }
    func centralManager(_ central: CBCentralManager, didFailToConnect p: CBPeripheral, error: Error?) {
        guard p === peripheral else { return }
        if let error { print("[Bluetooth] didFailToConnect: \(describe(error))") }
        close(error.map { "Connection failed: \(describe($0))" } ?? "Connection failed")
    }
    func centralManager(_ central: CBCentralManager, didDisconnectPeripheral p: CBPeripheral, error: Error?) {
        guard p === peripheral else { return }
        if let error { print("[Bluetooth] didDisconnect: \(describe(error))") }
        close("Disconnected · reconnecting")
    }

    func peripheral(_ p: CBPeripheral, didModifyServices invalidatedServices: [CBService]) {
        guard p === peripheral else { return }
        guard invalidatedServices.contains(where: { $0.uuid == serviceID }) else { return }
        print("[Bluetooth] didModifyServices: Terra service invalidated, re-discovering")
        chars.removeAll(); pairingMode = false; authenticated = false
        relayConnected = false; revision = 0; profiles = nil; sync.clear(); piSupportsVehicleSetup = false; waiting = nil
        writing = false; pending = []; queuedWrites = []; statusAssembly.clear(); replyAssembly.clear()
        if edit != nil { edit = nil; setApply(.failed("Rover restarted before the layout was saved")) }
        DispatchQueue.main.async { self.ready = false; self.ports = []; self.occupied = []; self.vehicleSetup = nil }
        publish("Refreshing Bluetooth services")
        p.discoverServices([serviceID])
    }

    func peripheral(_ p: CBPeripheral, didDiscoverServices error: Error?) {
        guard p === peripheral else { return }
        if let error {
            print("[Bluetooth] didDiscoverServices: \(describe(error))")
            close("Service discovery failed: \(describe(error))")
            return
        }
        guard let service = p.services?.first(where: { $0.uuid == serviceID }) else { close("Terra service unavailable"); return }
        p.discoverCharacteristics([driveID, statusID, controlID, repliesID], for: service)
    }

    func peripheral(_ p: CBPeripheral, didDiscoverCharacteristicsFor service: CBService, error: Error?) {
        guard p === peripheral else { return }
        if let error {
            print("[Bluetooth] didDiscoverCharacteristics: \(describe(error))")
            close("Characteristic discovery failed: \(describe(error))")
            return
        }
        for c in service.characteristics ?? [] { chars[c.uuid] = c }
        guard let status = chars[statusID] else { close("Status unavailable"); return }
        // A rover in pairing mode exposes only status; reading it triggers the iOS pairing prompt.
        pairingMode = chars[driveID] == nil && chars[controlID] == nil
        p.readValue(for: status)
        publish(pairingMode ? "Accept Bluetooth pairing on your phone" : "Authenticating")
    }

    // MARK: Messages

    func peripheral(_ p: CBPeripheral, didUpdateValueFor c: CBCharacteristic, error: Error?) {
        guard p === peripheral else { return }
        if let error {
            let detail = describe(error)
            print("[Bluetooth] didUpdateValueFor \(c.uuid): \(detail)")
            if (error as? CBATTError)?.code == .invalidHandle {
                close("Stale Bluetooth cache (handle is invalid) · toggle Bluetooth off/on in Settings")
            } else {
                close("Bluetooth read failed: \(detail)")
            }
            return
        }
        guard let bytes = c.value else {
            print("[Bluetooth] didUpdateValueFor \(c.uuid): empty value")
            close("Bluetooth read failed: empty value")
            return
        }
        let now = ProcessInfo.processInfo.systemUptime
        do {
            let document: Data?
            if (try? JSONSerialization.jsonObject(with: bytes)) is [String: Any] { document = bytes }
            else if c.uuid == statusID { document = try statusAssembly.push(bytes, now: now) }
            else if c.uuid == repliesID { document = try replyAssembly.push(bytes, now: now) }
            else { return }
            guard let document, let object = try JSONSerialization.jsonObject(with: document) as? [String: Any],
                  object["schema_version"] as? Int == 1 else { return }
            if pairingMode {
                guard object["type"] as? String == "setup", object["pairing_confirmed"] as? Bool == true else { return }
                close("Paired · waiting for rover to restart"); return
            }
            if c.uuid == statusID { try handleStatus(object, peripheral: p, status: c) }
            else if c.uuid == repliesID { try handleReply(object) }
        } catch {
            close("Malformed Bluetooth message · reconnecting")
        }
    }

    private func handleStatus(_ object: [String: Any], peripheral p: CBPeripheral, status c: CBCharacteristic) throws {
        guard object["type"] as? String == "status",
              let connected = object["connected"] as? Bool,
              let revision = object["active_revision"] as? UInt32 else { throw BluetoothPolicyError.malformed }
        let firstStatus = !authenticated
        authenticated = true
        relayConnected = connected && !(object["fault"] is String)
        if revision != self.revision {
            self.revision = revision; profiles = nil
            // The status can overtake the commit reply: a new revision while
            // committing means the rover saved the layout.
            if let committed = edit, committed.committing {
                edit = nil
                if let setup = committed.setup { DispatchQueue.main.async { self.vehicleSetup = setup } }
                setApply(.done(revision))
            }
            if !firstStatus { request("layout_index") }
        }
        let ready = relayConnected && profiles != nil
        DispatchQueue.main.async { self.connected = p.identifier; self.ready = ready }
        if let fault = object["fault"] as? String { publish("Rover fault: \(fault)") }
        else { publish(ready ? "Ready" : (connected ? "Reading actuator layout" : "Waiting for motor relay")) }
        if firstStatus, let replies = chars[repliesID] {
            p.setNotifyValue(true, for: c); p.setNotifyValue(true, for: replies)
            request("capabilities")
        }
    }

    func peripheral(_ p: CBPeripheral, didUpdateNotificationStateFor c: CBCharacteristic, error: Error?) {
        guard p === peripheral else { return }
        if let error {
            let detail = describe(error)
            print("[Bluetooth] didUpdateNotificationState \(c.uuid): \(detail)")
            close("Bluetooth subscribe failed: \(detail)")
            return
        }
        // The rover marks the relay connected right after our first read,
        // before this subscription exists, so that change is never notified.
        // Read status again now that later changes will reach us.
        if c.uuid == statusID, c.isNotifying { p.readValue(for: c) }
    }

    private func handleReply(_ object: [String: Any]) throws {
        guard let id = object["request_id"] as? UInt32, let w = waiting, id == w.id else { return }
        waiting = nil
        if Self.layoutOperations.contains(w.operation) { handleLayoutReply(w.operation, object); return }
        guard object["result"] as? String == "ok", let payload = object["payload"] as? [String: Any] else {
            publish("Rover rejected \(w.operation)"); return
        }
        switch w.operation {
        case "capabilities":
            let advertised = (payload["ports"] as? [String: Any] ?? [:]).compactMap { name, value -> RoverPort? in
                guard let port = value as? [String: Any], let kinds = port["kinds"] as? [String],
                      let resources = port["resources"] as? [String] else { return nil }
                return RoverPort(name: name, kinds: kinds, resources: resources)
            }.sorted { VehicleSetup.portOrder($0.name) < VehicleSetup.portOrder($1.name) }
            let occupied = Set(payload["occupied_resources"] as? [String] ?? [])
            let features = Set(payload["features"] as? [String] ?? [])
            piSupportsVehicleSetup = features.contains("vehicle_setup_v1")
            DispatchQueue.main.async { self.ports = advertised; self.occupied = occupied }
            request("layout_index")
        case "layout_index":
            guard let rev = payload["revision"] as? UInt32, let entries = payload["actuators"] as? [[String: Any]] else { throw BluetoothPolicyError.malformed }
            sync.begin(revision: rev, ids: entries.compactMap { $0["id"] as? Int })
            if payload["layout_available"] as? Bool == true { try nextProfile() }
            else {
                DispatchQueue.main.async { self.vehicleSetup = nil }
                publish("No actuator layout on the rover")
            }
        case "read_drive_profile":
            guard let rev = payload["revision"] as? UInt32, let entry = payload["actuator"] else { throw BluetoothPolicyError.malformed }
            let actuator = try JSONDecoder().decode(ActuatorDriveProfile.self, from: JSONSerialization.data(withJSONObject: entry))
            try sync.accept(revision: rev, actuator: actuator)
            try nextProfile()
        case "read_vehicle_setup":
            let decoded: VehicleSetup? = if let raw = payload["vehicle"], !(raw is NSNull) {
                VehicleSetup.from(jsonObject: raw)
            } else {
                nil
            }
            DispatchQueue.main.async { self.vehicleSetup = decoded }
        default: break
        }
    }

    private func handleLayoutReply(_ operation: String, _ object: [String: Any]) {
        guard var e = edit, operation != "discard_layout_edit" else { return }
        guard object["result"] as? String == "ok", let payload = object["payload"] as? [String: Any] else {
            let errors = (object["errors"] as? [[String: Any]] ?? []).map { error -> String in
                let message = error["message"] as? String ?? "rejected"
                guard let id = error["actuator_id"] as? Int else { return message }
                return "\(e.names[id] ?? "#\(id)"): \(message)"
            }
            edit = nil
            setApply(.failed(errors.isEmpty ? "Rover rejected \(operation)" : errors.joined(separator: "\n")))
            if !e.token.isEmpty { request("discard_layout_edit", payload: ["edit_token": e.token]) }
            return
        }
        if let token = payload["edit_token"] as? String { e.token = token }
        if let version = payload["edit_version"] as? UInt32 { e.version = version }
        if let base = payload["base_revision"] as? UInt32 { e.base = base }
        switch operation {
        case "begin_layout_edit", "stage_actuator", "stage_vehicle_setup":
            if !e.remaining.isEmpty {
                let actuator = e.remaining.removeFirst()
                setApply(.working("Sending \(actuator["name"] as? String ?? "actuator")"))
                request("stage_actuator", payload: ["edit_token": e.token, "edit_version": e.version, "actuator": actuator])
            } else if piSupportsVehicleSetup, !e.vehicleStaged, let vehicle = e.vehicle {
                e.vehicleStaged = true
                setApply(.working("Saving vehicle setup"))
                request("stage_vehicle_setup", payload: ["edit_token": e.token, "edit_version": e.version, "vehicle": vehicle])
            } else {
                setApply(.working("Checking the layout"))
                request("validate_layout_edit", payload: ["edit_token": e.token, "edit_version": e.version])
            }
        case "validate_layout_edit":
            e.committing = true
            setApply(.working("Saving on the rover"))
            request("commit_layout_edit", payload: ["edit_token": e.token, "edit_version": e.version, "base_revision": e.base])
        case "commit_layout_edit":
            let committedSetup = e.setup
            edit = nil
            if let committedSetup { DispatchQueue.main.async { self.vehicleSetup = committedSetup } }
            setApply(.done(payload["revision"] as? UInt32 ?? revision))
            return
        default: break
        }
        edit = e
    }

    private func nextProfile() throws {
        guard let rev = sync.revision else { throw BluetoothPolicyError.malformed }
        if let id = sync.remaining.first {
            request("read_drive_profile", payload: ["expected_revision": rev, "actuator_id": id])
        } else if let set = sync.completed, set.revision == revision {
            profiles = set
            let ready = relayConnected
            DispatchQueue.main.async { self.ready = ready }
            publish(ready ? "Ready" : "Waiting for motor relay")
            if piSupportsVehicleSetup { request("read_vehicle_setup") }
        } else {
            request("layout_index")
        }
    }

    private func request(_ operation: String, payload: [String: Any] = [:]) {
        guard let p = peripheral, chars[controlID] != nil else { return }
        requestID &+= 1
        UserDefaults.standard.set(Int(requestID), forKey: "terra.rover.requestID")
        let envelope: [String: Any] = ["schema_version": 1, "request_id": requestID, "operation": operation, "payload": payload]
        guard let data = try? JSONSerialization.data(withJSONObject: envelope) else { return }
        messageID &+= 1
        // Cap each fragment to <= 180 bytes (and <= ATT_MTU - 3) so every
        // chunk fits in a single ATT Write Request well below BlueZ's 512-byte
        // attribute limit and never triggers a BLE Long Write.
        let mtuWrite = min(p.maximumWriteValueLength(for: .withResponse),
                           p.maximumWriteValueLength(for: .withoutResponse))
        let writeSize = max(20, min(180, mtuWrite))
        guard let chunks = try? fragmentMessage(data, id: messageID, writeSize: writeSize) else {
            print("[Bluetooth] failed to fragment \(operation) (\(data.count)B, writeSize=\(writeSize))")
            return
        }
        print("[Bluetooth] -> \(operation) (\(data.count)B, \(chunks.count) chunk(s), writeSize=\(writeSize), mtuWrite=\(mtuWrite))")
        waiting = (requestID, operation, payload)
        queuedWrites += chunks.map { ($0, controlID) }
        pump()
    }

    // MARK: Writes

    /// A layout edit or control request is in progress on the rover.
    private var controlBusy: Bool { edit != nil || waiting != nil }

    /// Control messages go first; then the newest drive frame, unless a
    /// control request is still being handled by the rover.
    private func pump() {
        guard !writing, let p = peripheral else { return }
        let next: (Data, CBUUID)
        if !queuedWrites.isEmpty { next = queuedWrites.removeFirst() }
        else if controlBusy { pending = []; return }
        else if !pending.isEmpty { next = (pending.removeFirst(), driveID) }
        else { return }
        guard let c = chars[next.1] else { return }
        writing = true
        p.writeValue(next.0, for: c, type: .withResponse)
    }

    func peripheral(_ p: CBPeripheral, didWriteValueFor c: CBCharacteristic, error: Error?) {
        guard p === peripheral else { return }
        writing = false
        if let error {
            let detail = describe(error)
            print("[Bluetooth] didWriteValueFor \(c.uuid) (op=\(waiting?.operation ?? "none"), remainingChunks=\(queuedWrites.count)): \(detail)")
            close("Bluetooth write failed: \(detail)")
            return
        }
        pump()
    }
}
