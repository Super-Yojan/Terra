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
        print("Auto-connection selection: 6 checks passed")
    }
}
