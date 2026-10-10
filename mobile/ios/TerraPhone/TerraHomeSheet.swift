import SwiftUI

enum TerraHomeSheet: String, Identifiable {
    case rover, fleet, fleetSettings, roverSetup
    var id: String { rawValue }
}
