import Foundation
struct PhoneLocalizationInput {
 var now: Double
 var latitude: Double?
 var longitude: Double?
 var fixTime: Double?
 var accuracy: Double?
 var heading: Double?
 var headingAccuracy: Double?
 var headingTime: Double?
 var localX: Double
 var localY: Double
 var topYaw: Double?
 var tracked: Bool
 var permissionReason: String? = nil
}
struct PhoneLocalizationReport: Codable {
 var version = 1
 var frameID: String
 var mode: String
 var reason: String
 var tracking: String
 var gpsAccuracy: Double?
 var gpsLatitude: Double?
 var gpsLongitude: Double?
 var gpsFixAge: Double?
 var headingAccuracy: Double?
 var originLatitude: Double?
 var originLongitude: Double?
 var anchorX: Double?
 var anchorY: Double?
 var rotation: Double?
}
struct PhoneLocalizationPolicy {
 private var frameID = UUID().uuidString
 var identifier: String { frameID }
 private var goodSince: Double?
 private var anchor: (Double, Double, Double, Double, Double)?
 mutating func reset() { frameID = UUID().uuidString; goodSince = nil; anchor = nil }
 mutating func update(_ input: PhoneLocalizationInput) -> PhoneLocalizationReport {
  let freshFix = input.fixTime.map { input.now - $0 >= -1 && input.now - $0 <= 3 } ?? false
  let freshHeading = input.headingTime.map { input.now - $0 >= -1 && input.now - $0 <= 2 } ?? false
  let gpsGood = freshFix && input.accuracy.map { $0.isFinite && $0 >= 0 && $0 <= 10 } == true
   && input.latitude.map { $0.isFinite && abs($0) <= 85 } == true && input.longitude.map { $0.isFinite && abs($0) <= 180 } == true
  let headingGood = freshHeading && input.headingAccuracy.map { $0.isFinite && $0 >= 0 && $0 <= 15 } == true
   && input.heading.map { $0.isFinite && $0 >= 0 && $0 < 360 } == true && input.topYaw?.isFinite == true
  let usable = input.permissionReason == nil && gpsGood && headingGood && input.tracked && input.localX.isFinite && input.localY.isFinite
  if usable { if goodSince == nil { goodSince = input.now } } else { goodSince = nil }
  let ready = goodSince.map { input.now - $0 >= 8 } ?? false
  if ready && anchor == nil {
   anchor = (input.latitude!, input.longitude!, input.localX, input.localY, -input.heading! * .pi / 180 - input.topYaw!)
  }
  let gpsReason: String
  if let permission = input.permissionReason { gpsReason = permission }
  else if input.fixTime == nil { gpsReason = "Waiting for GPS fix" }
  else if !freshFix { gpsReason = "GPS fix stale · waiting for an update" }
  else if !gpsGood { gpsReason = "GPS accuracy insufficient · requires 10 m or better" }
  else { gpsReason = "GPS available" }
  let reason = !input.tracked ? "AR tracking unavailable" : !gpsGood || input.permissionReason != nil ? gpsReason : !headingGood ? "Heading uncertain · requires fresh compass accuracy of 15° or better" : !ready ? "Checking geographic alignment" : "GPS and heading reliable"
  return PhoneLocalizationReport(frameID: frameID, mode: ready ? "geographic" : "local", reason: reason,
   tracking: input.tracked ? "normal" : "limited", gpsAccuracy: input.accuracy,
   gpsLatitude: input.latitude.flatMap { $0.isFinite && abs($0) <= 90 ? $0 : nil },
   gpsLongitude: input.longitude.flatMap { $0.isFinite && abs($0) <= 180 ? $0 : nil },
   gpsFixAge: input.fixTime.flatMap { $0.isFinite && input.now.isFinite ? max(0, input.now-$0) : nil }, headingAccuracy: input.headingAccuracy,
   originLatitude: anchor?.0, originLongitude: anchor?.1, anchorX: anchor?.2, anchorY: anchor?.3, rotation: anchor?.4)
 }
}

/// A local supporting plane in the fixed, Z-up mapping world frame.
/// Signed height above ground is `normal · point - offset`, in metres.
struct PhoneGroundPlane {
 var normal: SIMD3<Double>
 var offset: Double
}

/// Calibrate the world origin and camera clearance from a stable measured floor.
/// Keep the world offset fixed, but follow terrain elevation and rover pitch/roll
/// with a local supporting plane. Never move world points to flatten the terrain.
struct PhoneGroundPolicy {
 private(set) var offset: Double?
 private var clearance: Double?
 private var candidate: Double?
 private var since: Double?
 mutating func reset() { offset = nil; clearance = nil; candidate = nil; since = nil }
 mutating func observe(height: Double?, cameraHeight: Double, area: Double, now: Double, upZ: Double = 1) -> Double? {
  if let offset { return offset }
  guard let height, height.isFinite, cameraHeight.isFinite, now.isFinite,
        upZ.isFinite, upZ >= 0.5, upZ <= 1.000001,
        area.isFinite, area >= 0.4, cameraHeight-height >= 0.1, cameraHeight-height <= 3 else {
   candidate = nil; since = nil; return nil
  }
  if candidate.map({ abs($0-height) <= 0.03 }) != true { candidate = height; since = now }
  if let since, now-since >= 1 {
   offset = -height
   clearance = (cameraHeight-height) * upZ
  }
  return offset
 }

 /// Use the calibrated rigid camera-to-ground distance and the current body up
 /// axis. Translation up/down a ramp changes this plane, not the world origin.
 /// The rover must remain supported on the terrain; this is not a depth plane fitter.
 func groundPlane(cameraPosition: SIMD3<Double>, bodyUp: SIMD3<Double>) -> PhoneGroundPlane? {
  guard let offset, let clearance,
        cameraPosition.x.isFinite, cameraPosition.y.isFinite, cameraPosition.z.isFinite,
        bodyUp.x.isFinite, bodyUp.y.isFinite, bodyUp.z.isFinite else { return nil }
  let length = hypot(hypot(bodyUp.x, bodyUp.y), bodyUp.z)
  guard length.isFinite, length >= 1e-12 else { return nil }
  let normal = bodyUp / length
  guard normal.z >= 0.5 else { return nil }
  let camera = cameraPosition + SIMD3(0, 0, offset)
  return PhoneGroundPlane(normal: normal,
   offset: normal.x * camera.x + normal.y * camera.y + normal.z * camera.z - clearance)
 }
}
