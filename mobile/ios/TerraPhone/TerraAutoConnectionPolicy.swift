import Foundation

/// Prefer the previously selected rover. Never fall back to another owner's rover.
enum TerraAutoConnectionPolicy {
    static func select(candidates: [UUID], preferred: UUID?) -> UUID? {
        let unique = Set(candidates)
        if let preferred { return unique.contains(preferred) ? preferred : nil }
        return unique.count == 1 ? unique.first : nil
    }
}
