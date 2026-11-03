import Foundation

enum BluetoothPolicyError: Error { case unavailable, malformed }

struct BluetoothJSONAssembler {
    let timeout: TimeInterval
    let limit: Int
    init(timeout: TimeInterval = 0.1, limit: Int = 16384) {
        precondition(timeout.isFinite && timeout > 0 && limit > 0 && limit <= 16384)
        self.timeout = timeout; self.limit = limit
    }
    private var id: UInt16?
    private var count: UInt8 = 0
    private var next: UInt8 = 0
    private var started: TimeInterval = 0
    private var data = Data()
    mutating func clear() { id = nil; data.removeAll(); next = 0 }
    mutating func push(_ bytes: Data, now: TimeInterval) throws -> Data? {
        if id != nil && (now < started || now - started >= timeout) { clear() }
        guard bytes.count >= 5, bytes[3] > 0, bytes[2] < bytes[3] else { clear(); throw BluetoothPolicyError.malformed }
        let incoming = UInt16(bytes[0]) | UInt16(bytes[1]) << 8
        if id == nil { guard bytes[2] == 0 else { throw BluetoothPolicyError.malformed }; id = incoming; count = bytes[3]; started = now }
        guard id == incoming, count == bytes[3], next == bytes[2], data.count + bytes.count - 4 <= limit else { clear(); throw BluetoothPolicyError.malformed }
        data.append(bytes.dropFirst(4))
        if next == count - 1 { let result = data; clear(); return result }
        next += 1; return nil
    }
}
