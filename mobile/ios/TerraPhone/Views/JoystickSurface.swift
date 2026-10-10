import SwiftUI

struct JoystickSurface: View {
    let diameter: CGFloat
    let offset: CGSize
    var body: some View {
        ZStack {
            Circle().fill(Color(uiColor: .tertiarySystemGroupedBackground))
            guides
            arrows
            thumb
        }
        .font(.caption.weight(.semibold)).foregroundStyle(.secondary)
        .frame(width: diameter, height: diameter)
    }
    private var guides: some View {
        ZStack {
            Circle().stroke(Color.primary.opacity(0.08), lineWidth: 1)
            Circle().stroke(Color.primary.opacity(0.07), style: StrokeStyle(lineWidth: 1, dash: [3, 5])).padding(42)
            Rectangle().fill(Color.primary.opacity(0.06)).frame(width: 1).padding(.vertical, 24)
            Rectangle().fill(Color.primary.opacity(0.06)).frame(height: 1).padding(.horizontal, 24)
        }
    }
    private var arrows: some View {
        ZStack {
            VStack { Image(systemName: "chevron.up"); Spacer(); Image(systemName: "chevron.down") }.padding(15)
            HStack { Image(systemName: "chevron.left"); Spacer(); Image(systemName: "chevron.right") }.padding(15)
        }
    }
    private var thumb: some View {
        Circle().fill(Color(red: 0.09, green: 0.23, blue: 0.17))
            .frame(width: 78, height: 78)
            .overlay {
                Image(systemName: "plus").font(.title2.weight(.medium)).foregroundStyle(.white.opacity(0.8))
            }
            .shadow(color: .black.opacity(0.14), radius: 8, y: 4)
            .offset(offset)
    }
}
