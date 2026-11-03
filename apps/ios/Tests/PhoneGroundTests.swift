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
  // The plane stays level whatever body up reads, so a mount pitch/roll error
  // cannot tilt it into the floor. It follows camera elevation, while world
  // coordinates remain fixed.
  let angle = 16.0 * Double.pi / 180
  for bodyUp in [SIMD3(-sin(angle), 0, cos(angle)), SIMD3(sin(angle), 0, cos(angle)), SIMD3(0, -sin(angle), cos(angle))] {
   for elevation in [0.0, 0.3] {
    let camera = SIMD3<Double>(2, 3, 0.28 - offset + elevation)
    let plane = policy.groundPlane(cameraPosition: camera, bodyUp: bodyUp)!
    func height(_ point: SIMD3<Double>) -> Double {
     plane.normal.x * point.x + plane.normal.y * point.y + plane.normal.z * point.z - plane.offset
    }
    for reach in [-3.5, 0, 3.5] {
     let floor = SIMD3(2 + reach, 3 + reach, elevation)
     precondition(abs(height(floor)) < 0.000001, "level floor must stay ground at every distance")
     precondition(abs(height(floor + SIMD3(0, 0, 0.3)) - 0.3) < 0.000001, "objects above the floor remain obstacles")
    }
    precondition(policy.offset == offset, "terrain must not move the mapping world frame")
   }
  }
  precondition(policy.groundPlane(cameraPosition: SIMD3(0, 0, .nan), bodyUp: SIMD3(0, 0, 1)) == nil)
  precondition(policy.groundPlane(cameraPosition: .zero, bodyUp: .zero) == nil)
  precondition(policy.groundPlane(cameraPosition: .zero, bodyUp: SIMD3(1, 0, 0)) == nil)
  policy.reset()
  precondition(policy.groundPlane(cameraPosition: .zero, bodyUp: SIMD3(0, 0, 1)) == nil, "reset requires measured ground before mapping")
  precondition(policy.observe(height: 0.2, cameraHeight: 0, area: 2, now: 3) == nil, "surface above camera is not the floor")
  precondition(policy.observe(height: -0.3, cameraHeight: 0, area: 0.05, now: 4) == nil, "small patches cannot calibrate ground")
  policy.reset()
  precondition(policy.mount(clearance: 0.4, cameraHeight: 0.65) != nil, "a measured mount height fixes ground without a floor anchor")
  precondition(abs(policy.offset! - (0.4 - 0.65)) < 0.000001)
  print("Ground calibration: mount height, stable world frame, level plane under tilted body, raised obstacles, invalid plane and reset passed")
 }
}
