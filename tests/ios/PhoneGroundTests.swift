import Foundation
@main struct GroundTests {
 static func main() {
  var policy = PhoneGroundPolicy()
  precondition(policy.observe(height: nil, cameraHeight: 0, area: 0, now: 0) == nil)
  precondition(policy.observe(height: -0.28, cameraHeight: 0, area: 1, now: 0) == nil)
  precondition(policy.observe(height: -0.28, cameraHeight: 0, area: 1, now: 0.5) == nil)
  let offset = policy.observe(height: -0.28, cameraHeight: 0, area: 1, now: 1.1)!
  precondition(abs(offset - 0.28) < 0.0001)
  precondition(abs(-0.28 + offset) < 0.0001, "floor must map to zero, not assumed camera height")
  precondition(policy.observe(height: -0.8, cameraHeight: 0, area: 1, now: 2) == offset, "one noisy plane cannot shift an established map")
  policy.reset()
  precondition(policy.observe(height: 0.2, cameraHeight: 0, area: 2, now: 3) == nil, "surface above camera is not the floor")
  precondition(policy.observe(height: -0.3, cameraHeight: 0, area: 0.05, now: 4) == nil, "small patches cannot calibrate ground")
  print("Ground calibration: measured height, stability, invalid plane and reset passed")
 }
}
