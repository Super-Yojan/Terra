import Foundation
import simd
@main struct PersonDepthGeometryTests {
 static func main() {
  let k = simd_float3x3(columns: (SIMD3(100,0,0), SIMD3(0,100,0), SIMD3(50,40,1)))
  let p = PersonDepthGeometry.worldPoint(pixel: SIMD2(50,40), depth: 2, intrinsics: k, cameraTransform: matrix_identity_float4x4, worldFromAR: matrix_identity_float3x3)!
  precondition(p == SIMD3(0,0,-2))
  precondition(PersonDepthGeometry.worldPoint(pixel: SIMD2(50,40), depth: .nan, intrinsics: k, cameraTransform: matrix_identity_float4x4, worldFromAR: matrix_identity_float3x3) == nil)
  precondition(PersonDepthGeometry.measuredDepth([2,2,2,2,2,10]) == 2)
  precondition(PersonDepthGeometry.measuredDepth([2,2]) == nil)
  for (o, expected) in [(PersonImageOrientation.up, SIMD2<Float>(0.2,0.3)), (.right, SIMD2<Float>(0.3,0.8)), (.down, SIMD2<Float>(0.8,0.7)), (.left, SIMD2<Float>(0.7,0.2))] {
   let raw = PersonDepthGeometry.rawPoint(x: 0.2, y: 0.7, orientation: o)
   precondition(simd_length(raw-expected) < 0.00001)
  }
  let offset = PersonDepthGeometry.worldPoint(pixel: SIMD2(75,65), depth: 2, intrinsics: k, cameraTransform: matrix_identity_float4x4, worldFromAR: matrix_identity_float3x3)!
  precondition(offset == SIMD3(0.5,-0.5,-2))
  precondition(PersonDepthGeometry.measuredDepth([1,2,3,4,5,6]) == nil)
  for o in [PersonImageOrientation.up, .right, .down, .left] {
   let q = PersonDepthGeometry.rawPoint(x: 0.5, y: 0.5, orientation: o)
   precondition(q == SIMD2(0.5,0.5))
  }
  var transform = matrix_identity_float4x4; transform.columns.3.x = 3
  let shifted = PersonDepthGeometry.worldPoint(pixel: SIMD2(50,40), depth: 2, intrinsics: k, cameraTransform: transform, worldFromAR: matrix_identity_float3x3)!
  precondition(shifted.x == 3)
  var gate = PersonDetectorFrameGate()
  precondition(gate.begin(timestamp: 1) == 1)
  precondition(gate.begin(timestamp: 2) == nil, "No frame backlog while inference runs")
  gate.finish()
  precondition(gate.begin(timestamp: 1.1) == nil)
  precondition(gate.begin(timestamp: 0.5) == nil)
  precondition(gate.begin(timestamp: .nan) == nil)
  precondition(gate.begin(timestamp: 1.3) == 2)
  // Portrait (home button down) puts world up along camera -X; Terra's landscape mount along ±Y.
  precondition(PersonDepthGeometry.orientation(upX: -1, upY: 0) == .right)
  precondition(PersonDepthGeometry.orientation(upX: 1, upY: 0.1) == .left)
  precondition(PersonDepthGeometry.orientation(upX: 0.2, upY: 0.9) == .up)
  precondition(PersonDepthGeometry.orientation(upX: 0, upY: -1) == .down)
  precondition(PersonDepthGeometry.orientation(upX: 0.1, upY: 0.1) == nil)
  // 1920x1440 landscape letterboxed into 640: 640x480 content, 80 px bars top and bottom.
  let top = PersonDepthGeometry.visionPoint(modelX: 0, modelY: 80, input: 640, imageWidth: 1920, imageHeight: 1440)
  precondition(simd_length(top - SIMD2(0, 1)) < 0.00001)
  let centre = PersonDepthGeometry.visionPoint(modelX: 320, modelY: 320, input: 640, imageWidth: 1440, imageHeight: 1920)
  precondition(simd_length(centre - SIMD2(0.5, 0.5)) < 0.00001)
  let corner = PersonDepthGeometry.visionPoint(modelX: 560, modelY: 640, input: 640, imageWidth: 1440, imageHeight: 1920)
  precondition(simd_length(corner - SIMD2(1, 0)) < 0.00001)
  print("Person depth: projection, camera orientation, measured depth and frame transform passed")
 }
}
