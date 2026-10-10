import Foundation
@main struct LocalizationTests {
 static func main() {
  var policy = PhoneLocalizationPolicy()
  func input(_ now: Double, accuracy: Double = 4, tracked: Bool = true) -> PhoneLocalizationInput {
   PhoneLocalizationInput(now: now, latitude: 38, longitude: -77, fixTime: now, accuracy: accuracy, heading: 90, headingAccuracy: 5, headingTime: now, localX: 3, localY: -2, topYaw: 0, tracked: tracked)
  }
  precondition(policy.update(input(0)).mode == "local")
  precondition(policy.update(input(7)).mode == "local")
  let aligned = policy.update(input(8))
  precondition(aligned.mode == "geographic")
  precondition(abs((aligned.rotation ?? 0) + Double.pi/2) < 1e-9)
  precondition(policy.update(input(9, accuracy: 50)).mode == "local")
  precondition(policy.update(input(10)).mode == "local")
  precondition(policy.update(input(18)).mode == "geographic")
  precondition(policy.update(input(19, tracked: false)).mode == "local")
  policy.reset()
  precondition(policy.update(input(30)).originLatitude == nil)
  print("Phone localization hysteresis, heading alignment, tracking loss and reset: passed")
 }
}
