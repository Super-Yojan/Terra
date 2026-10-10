import SwiftUI
import UIKit

enum TerraDestination: String, Hashable {
    case drive, missions, live, telemetry, settings, robots
    var title: String {
        switch self {
        case .drive: return "Debug drive"
        case .missions: return "Navigation tools"
        case .live: return "Map inspection"
        case .telemetry: return "Diagnostics"
        case .settings: return "Controller tools"
        case .robots: return "Rover setup"
        }
    }
}
