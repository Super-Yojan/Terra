import SwiftUI
import UIKit

enum TerraStyle {
    static let forest = Color(uiColor: UIColor { traits in
        traits.userInterfaceStyle == .dark
            ? UIColor(red: 0.63, green: 0.83, blue: 0.70, alpha: 1)
            : UIColor(red: 0.09, green: 0.23, blue: 0.17, alpha: 1)
    })
    static let buttonForest = Color(red: 0.09, green: 0.23, blue: 0.17)
    static let background = Color(uiColor: .systemGroupedBackground)
    static let surface = Color(uiColor: .secondarySystemGroupedBackground)
}
