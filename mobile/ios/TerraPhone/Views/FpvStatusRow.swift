import SwiftUI

/// Shows whether the rear camera is streaming to the dashboard. Stopping the stream leaves arming unchanged.
struct FpvStatusRow: View {
    @ObservedObject var brain: PhoneController
    var body: some View {
        VStack(alignment: .leading, spacing: 8) {
            HStack(alignment: .center, spacing: 14) {
                Image(systemName: brain.fpvLive ? "video.fill" : "video")
                    .font(.title2)
                    .foregroundStyle(brain.fpvLive ? Color.red : TerraStyle.forest)
                    .frame(width: 44, height: 44)
                    .accessibilityHidden(true)
                VStack(alignment: .leading, spacing: 4) {
                    Text("FPV").font(.title2.weight(.semibold))
                    Text(brain.fpvStatus).font(.subheadline.weight(.medium))
                }
                Spacer(minLength: 0)
                if brain.fpvLive {
                    Circle().fill(Color.red).frame(width: 10, height: 10).accessibilityHidden(true)
                }
            }
            Text(brain.fpvLive ? "The rear camera is streaming to the dashboard. Drive commands still use the fleet link." : "The dashboard can request a live rear-camera view. This phone sends video only.")
                .font(.subheadline).foregroundStyle(.secondary).fixedSize(horizontal: false, vertical: true)
            if brain.fpvActive {
                Button("Stop FPV", role: .destructive) { brain.stopFpv() }
                    .buttonStyle(.bordered)
            }
        }
        .accessibilityElement(children: .combine)
    }
}
