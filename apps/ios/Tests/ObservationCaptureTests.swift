import Foundation
@main struct CaptureTests {
 static func main() {
  let a = ObservationCapturePolicy()
  precondition(a.shouldCapture(exposure: 1, frameID: "A"))
  precondition(!a.shouldCapture(exposure: 1.1, frameID: "A"))
  precondition(a.shouldCapture(exposure: 2, frameID: "A"))
  precondition(a.shouldCapture(exposure: 0.1, frameID: "B"), "a reset starts a separate exposure frame")
  precondition(!a.shouldCapture(exposure: .nan, frameID: "B"))
 }
}
