import SwiftUI

@main
struct TerraPhoneApp: App {
    init() { TerraBevySessionStore.discardOnDevice() }
    var body: some Scene { WindowGroup { ContentView() } }
}
