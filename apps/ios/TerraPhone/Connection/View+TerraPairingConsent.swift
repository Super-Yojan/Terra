import SwiftUI

extension View {
    func terraPairingConsent(brain: PhoneController, enabled: Bool = true) -> some View {
        alert("Connect to this rover?", isPresented: Binding(
            get: { enabled && brain.pairingCandidate != nil }, set: { _ in }
        )) {
            Button("Connect") { brain.respondToPairing(connect: true) }
            Button("Not now", role: .cancel) { brain.respondToPairing(connect: false) }
        } message: { Text("\(brain.pairingCandidate?.name ?? "Terra rover") is ready to pair.") }
    }
}
