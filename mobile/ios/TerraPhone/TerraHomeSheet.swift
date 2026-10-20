import SwiftUI

enum TerraHomeSheet: String, Identifiable {
    case rover, rovers, fleet, fleetSettings, roverSetup
    var id: String { rawValue }
}
