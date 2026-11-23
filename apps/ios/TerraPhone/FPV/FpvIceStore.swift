import Foundation
import Security

/// UserDefaults for ICE URLs and the Keychain for the TURN credential.
/// Nothing here is written into the repository.
enum FpvIceStore {
    static func load() -> FpvIceSettings {
        let defaults = UserDefaults.standard
        return FpvIceSettings(
            stunURL: defaults.string(forKey: FpvIceSettings.stunDefaultsKey) ?? FpvIceSettings.defaultStunURL,
            turnURL: defaults.string(forKey: FpvIceSettings.turnURLDefaultsKey) ?? "",
            turnUsername: defaults.string(forKey: FpvIceSettings.turnUsernameDefaultsKey) ?? "",
            turnCredential: credential()
        )
    }

    static func save(stunURL: String, turnURL: String, turnUsername: String, turnCredential: String) -> Bool {
        let defaults = UserDefaults.standard
        defaults.set(stunURL.trimmingCharacters(in: .whitespacesAndNewlines), forKey: FpvIceSettings.stunDefaultsKey)
        defaults.set(turnURL.trimmingCharacters(in: .whitespacesAndNewlines), forKey: FpvIceSettings.turnURLDefaultsKey)
        defaults.set(turnUsername, forKey: FpvIceSettings.turnUsernameDefaultsKey)
        return saveCredential(turnCredential)
    }

    private static func credential() -> String {
        var result: CFTypeRef?
        let query: [String: Any] = [
            kSecClass as String: kSecClassGenericPassword,
            kSecAttrService as String: "Terra.ARGOS",
            kSecAttrAccount as String: FpvIceSettings.turnCredentialAccount,
            kSecReturnData as String: true,
        ]
        guard SecItemCopyMatching(query as CFDictionary, &result) == errSecSuccess,
              let data = result as? Data else { return "" }
        return String(data: data, encoding: .utf8) ?? ""
    }

    private static func saveCredential(_ value: String) -> Bool {
        let query: [String: Any] = [
            kSecClass as String: kSecClassGenericPassword,
            kSecAttrService as String: "Terra.ARGOS",
            kSecAttrAccount as String: FpvIceSettings.turnCredentialAccount,
        ]
        if value.isEmpty {
            let status = SecItemDelete(query as CFDictionary)
            return status == errSecSuccess || status == errSecItemNotFound
        }
        let data = Data(value.utf8)
        let updated = SecItemUpdate(query as CFDictionary, [kSecValueData as String: data] as CFDictionary)
        if updated == errSecSuccess { return true }
        guard updated == errSecItemNotFound else { return false }
        var item = query
        item[kSecValueData as String] = data
        item[kSecAttrAccessible as String] = kSecAttrAccessibleAfterFirstUnlockThisDeviceOnly
        return SecItemAdd(item as CFDictionary, nil) == errSecSuccess
    }
}
