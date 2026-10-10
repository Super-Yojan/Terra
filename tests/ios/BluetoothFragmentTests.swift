import Foundation

@main struct BluetoothFragmentTests {
    static func main() throws {
        let first = Data([1, 0, 0, 2, 65])
        let second = Data([1, 0, 1, 2, 66])
        var config = BluetoothJSONAssembler(timeout: 2, limit: 4096)
        let initial = try config.push(first, now: 0)
        precondition(initial == nil)
        let complete = try config.push(second, now: 0.3)
        precondition(complete == Data([65, 66]))
        var motion = BluetoothJSONAssembler()
        _ = try motion.push(first, now: 0)
        do { _ = try motion.push(second, now: 0.1); preconditionFailure("Motion assembler accepted expired data") } catch { }
        _ = try config.push(first, now: 0)
        do { _ = try config.push(second, now: 2); preconditionFailure("Configuration assembler accepted expired data") } catch { }
        _ = try config.push(first, now: 10)
        do { _ = try config.push(second, now: 9); preconditionFailure("Clock regression accepted") } catch { }
        print("Bluetooth fragments: separate deadlines and clock regression passed")
    }
}
