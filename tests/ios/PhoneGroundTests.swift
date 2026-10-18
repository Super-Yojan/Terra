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
  // Elevation and attitude change the supporting plane, while world coordinates
  // remain fixed. Exercise uphill, downhill, and a slope along world +Y.
  let angle = 20.0 * Double.pi / 180
  for normal in [SIMD3(-sin(angle), 0, cos(angle)), SIMD3(sin(angle), 0, cos(angle)), SIMD3(0, -sin(angle), cos(angle))] {
   let ground = SIMD3<Double>(2, 3, 1)
   let camera = ground + normal * 0.28 - SIMD3(0, 0, 0.28)
   let plane = policy.groundPlane(cameraPosition: camera, bodyUp: normal)!
   let tangent = normal.x != 0 ? SIMD3(normal.z, 0, -normal.x) : SIMD3(0, normal.z, -normal.y)
   let rampPoint = ground + tangent * 2
   let obstacle = rampPoint + normal * 0.3
   func height(_ point: SIMD3<Double>) -> Double {
    plane.normal.x * point.x + plane.normal.y * point.y + plane.normal.z * point.z - plane.offset
   }
   precondition(abs(height(rampPoint)) < 0.000001, "elevated ramp must stay ground")
   precondition(abs(height(obstacle) - 0.3) < 0.000001, "objects above the ramp remain obstacles")
   precondition(policy.offset == offset, "terrain must not move the mapping world frame")
  }
  precondition(policy.groundPlane(cameraPosition: SIMD3(0, 0, .nan), bodyUp: SIMD3(0, 0, 1)) == nil)
  precondition(policy.groundPlane(cameraPosition: .zero, bodyUp: .zero) == nil)
  precondition(policy.groundPlane(cameraPosition: .zero, bodyUp: SIMD3(1, 0, 0)) == nil)
  policy.reset()
  precondition(policy.groundPlane(cameraPosition: .zero, bodyUp: SIMD3(0, 0, 1)) == nil, "reset requires measured ground before mapping")
  precondition(policy.observe(height: 0.2, cameraHeight: 0, area: 2, now: 3) == nil, "surface above camera is not the floor")
  precondition(policy.observe(height: -0.3, cameraHeight: 0, area: 0.05, now: 4) == nil, "small patches cannot calibrate ground")
  print("Ground calibration: stable world frame, uphill/downhill terrain, raised obstacles, invalid plane and reset passed")
 }
}
