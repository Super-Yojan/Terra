import Foundation
@main struct AutoConnectionTests {
    static func main() {
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
