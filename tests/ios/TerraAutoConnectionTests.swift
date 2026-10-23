import Foundation
@main struct AutoConnectionTests {
    static func main() {
        precondition(TerraRouterSettings.parse(endpoint: TerraFleetDefaults.endpoint, prefix: "terra/phone", roverID: "7")?.endpoint == TerraFleetDefaults.endpoint)
        precondition(TerraRouterSettings.parse(endpoint: "tls/host:0", prefix: "terra/phone", roverID: "7") == nil)
        precondition(TerraRouterSettings.parse(endpoint: "tls/host:7448#verify=false", prefix: "terra/phone", roverID: "7") == nil)
        let first = UUID(), second = UUID()
        precondition(TerraAutoConnectionPolicy.select(candidates: [], preferred: nil) == nil)
        precondition(TerraAutoConnectionPolicy.select(candidates: [first], preferred: nil) == nil)
        precondition(TerraAutoConnectionPolicy.select(candidates: [first, second], preferred: nil) == nil)
        precondition(TerraAutoConnectionPolicy.select(candidates: [first, second], preferred: second) == second)
        precondition(TerraAutoConnectionPolicy.select(candidates: [first], preferred: second) == nil)
        precondition(TerraAutoConnectionPolicy.select(candidates: [first, first], preferred: nil) == nil)
        let suite = "TerraRoverTests-" + UUID().uuidString
        let defaults = UserDefaults(suiteName: suite)!
        defer { defaults.removePersistentDomain(forName: suite) }
        defaults.set(first.uuidString, forKey: "preferredHardwareRover")
        defaults.set("terra-A", forKey: "preferredHardwareRoverName")
        var rovers = TerraSavedRovers(defaults: defaults)
        precondition(rovers.rovers.map(\.id) == [first], "Migrate the existing rover")
        rovers.remember(id: second, name: "terra-B")
        rovers.remember(id: first, name: "terra-A renamed")
        precondition(rovers.rovers.count == 2)
        precondition(rovers.rovers.first?.name == "terra-A renamed")
        precondition(rovers.select(first))
        precondition(TerraSavedRovers(defaults: defaults).selected == first)
        precondition(!rovers.select(UUID()), "Unknown rovers cannot be selected as owned")
        rovers.clearSelection()
        precondition(rovers.selected == nil && rovers.rovers.count == 2)
        rovers.remember(id: second, name: "terra-B")
        rovers.forget(first)
        precondition(rovers.selected == second && rovers.rovers.map(\.id) == [second])
        rovers.forget(second)
        precondition(TerraSavedRovers(defaults: defaults).rovers.isEmpty)
        precondition(defaults.string(forKey: "preferredHardwareRover") == nil)
        print("Auto-connection selection and saved-rover checks passed")
    }
}
