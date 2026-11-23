import Foundation
import Security

/// The fleet router password, kept in the Keychain so it never ships in
/// terra-av.toml. terra-av uses it for `[comms]` (see crates/terra-av/src/config.rs).
enum RouterPassword {
    private static let query: [String: Any] = [
        kSecClass as String: kSecClassGenericPassword,
        kSecAttrService as String: "org.terra.robotics.phone.router",
        kSecAttrAccount as String: "terra",
    ]

    static func load() -> String? {
        var item: CFTypeRef?
        var search = query
        search[kSecReturnData as String] = true
        search[kSecMatchLimit as String] = kSecMatchLimitOne
        guard SecItemCopyMatching(search as CFDictionary, &item) == errSecSuccess,
              let data = item as? Data else { return nil }
        return String(data: data, encoding: .utf8)
    }

    /// Save, or delete when `password` is empty. Returns false if the Keychain refused.
    @discardableResult
    static func save(_ password: String) -> Bool {
        SecItemDelete(query as CFDictionary)
        if password.isEmpty { return true }
        var item = query
        item[kSecValueData as String] = Data(password.utf8)
        item[kSecAttrAccessible as String] = kSecAttrAccessibleAfterFirstUnlockThisDeviceOnly
        return SecItemAdd(item as CFDictionary, nil) == errSecSuccess
    }
}

/// Base station Zenoh router settings (`[comms]` in terra-av.toml), persisted
/// on the phone in UserDefaults so the operator can switch routers or LAN IPs
/// from the UI without editing terra-av.toml or rebuilding the app.
struct BaseStationConfig: Codable, Equatable {
    var enabled = true
    var endpoint = ""
    var prefix = "terra"
    var user = ""
    /// Fallback rover ID when no Raspberry Pi setup is loaded. When connected
    /// to a Pi, `VehicleSetup.roverId` (stored on the Pi) takes precedence.
    var roverId = 0

    private static let key = "terra.baseStationConfig"

    init(enabled: Bool = true, endpoint: String = "", prefix: String = "terra", user: String = "", roverId: Int = 0) {
        self.enabled = enabled
        self.endpoint = endpoint
        self.prefix = prefix
        self.user = user
        self.roverId = roverId
    }

    init(from decoder: Decoder) throws {
        let c = try decoder.container(keyedBy: CodingKeys.self)
        enabled = try c.decodeIfPresent(Bool.self, forKey: .enabled) ?? true
        endpoint = try c.decodeIfPresent(String.self, forKey: .endpoint) ?? ""
        prefix = try c.decodeIfPresent(String.self, forKey: .prefix) ?? "terra"
        user = try c.decodeIfPresent(String.self, forKey: .user) ?? ""
        roverId = try c.decodeIfPresent(Int.self, forKey: .roverId) ?? 0
    }

    static func load(bundledToml: String? = nil) -> BaseStationConfig {
        if let data = UserDefaults.standard.data(forKey: key),
           let decoded = try? JSONDecoder().decode(BaseStationConfig.self, from: data) {
            return decoded
        }
        let toml = bundledToml ?? (Bundle.main.url(forResource: "terra-av", withExtension: "toml")
            .flatMap { try? String(contentsOf: $0, encoding: .utf8) } ?? "")
        return defaults(from: toml)
    }

    func save() {
        if let data = try? JSONEncoder().encode(self) {
            UserDefaults.standard.set(data, forKey: Self.key)
        }
    }

    /// Seed initial defaults from the bundled `[comms]` table if the user has
    /// never saved base station settings yet.
    static func defaults(from toml: String) -> BaseStationConfig {
        var config = BaseStationConfig()
        var inComms = false
        for raw in toml.components(separatedBy: "\n") {
            let line = raw.trimmingCharacters(in: .whitespaces)
            if line.isEmpty || line.hasPrefix("#") { continue }
            if line.hasPrefix("[") {
                inComms = (line == "[comms]")
                continue
            }
            guard inComms, let eq = line.firstIndex(of: "=") else { continue }
            let k = line[..<eq].trimmingCharacters(in: .whitespaces)
            let v = line[line.index(after: eq)...].trimmingCharacters(in: .whitespaces)
            switch k {
            case "endpoint": config.endpoint = unquote(v)
            case "prefix": config.prefix = unquote(v)
            case "user": config.user = unquote(v)
            case "rover_id": if let id = Int(v) { config.roverId = id }
            default: break
            }
        }
        return config
    }

    /// Replace (or remove, when disabled/empty) the `[comms]` table in `toml`.
    func applying(to toml: String, roverIdOverride: Int? = nil) -> String {
        var out: [String] = []
        var inComms = false
        for line in toml.components(separatedBy: "\n") {
            let trimmed = line.trimmingCharacters(in: .whitespaces)
            if trimmed.hasPrefix("[") {
                inComms = (trimmed == "[comms]")
                if inComms { continue }
            }
            if inComms { continue }
            out.append(line)
        }

        let cleanEndpoint = endpoint.trimmingCharacters(in: .whitespacesAndNewlines)
        guard enabled, !cleanEndpoint.isEmpty else {
            return out.joined(separator: "\n")
        }

        let cleanPrefix = prefix.trimmingCharacters(in: .whitespacesAndNewlines)
        let cleanUser = user.trimmingCharacters(in: .whitespacesAndNewlines)
        let effectiveRoverId = max(0, roverIdOverride ?? roverId)

        var block = [
            "",
            "[comms]",
            "endpoint = \"\(escape(cleanEndpoint))\"",
            "prefix = \"\(escape(cleanPrefix.isEmpty ? "terra" : cleanPrefix))\"",
            "rover_id = \(effectiveRoverId)",
        ]
        if !cleanUser.isEmpty {
            block.append("user = \"\(escape(cleanUser))\"")
        }
        out.append(contentsOf: block)
        return out.joined(separator: "\n")
    }

    private static func unquote(_ value: String) -> String {
        var s = value
        if let hash = s.firstIndex(of: "#") {
            s = String(s[..<hash]).trimmingCharacters(in: .whitespaces)
        }
        if s.hasPrefix("\""), s.hasSuffix("\""), s.count >= 2 {
            s = String(s.dropFirst().dropLast())
        }
        return s
    }

    private func escape(_ value: String) -> String {
        value.replacingOccurrences(of: "\\", with: "\\\\")
             .replacingOccurrences(of: "\"", with: "\\\"")
    }
}
