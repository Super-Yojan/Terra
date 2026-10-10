import OSLog

/// Unified logs appear in Xcode's debug console and macOS Console for a USB-connected iPhone.
/// Keep messages to operational state; do not log credentials or full sensor payloads.
enum TerraLog {
    static let subsystem = "com.terra.phone"
    static let bluetooth = Logger(subsystem: subsystem, category: "Bluetooth")
    static let control = Logger(subsystem: subsystem, category: "Control")
    static let tracking = Logger(subsystem: subsystem, category: "Tracking")
    static let configuration = Logger(subsystem: subsystem, category: "Configuration")
}
