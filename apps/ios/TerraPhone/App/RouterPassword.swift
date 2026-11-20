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
