import SwiftUI

enum TerraBevySessionStore {
    static let endpointKey = "zenohEndpoint"
    static let roverIDKey = "zenohRoverID"

    static func discardOnDevice() {
        #if !targetEnvironment(simulator)
        let defaults = UserDefaults.standard
        defaults.removeObject(forKey: endpointKey)
        defaults.removeObject(forKey: roverIDKey)
        #endif
    }
}
