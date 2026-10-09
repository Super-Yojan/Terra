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
}
struct PhoneLocalizationReport: Codable {
 var version = 1
 var frameID: String
 var mode: String
 var reason: String
 var tracking: String
 var gpsAccuracy: Double?
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
  let usable = gpsGood && headingGood && input.tracked && input.localX.isFinite && input.localY.isFinite
  if usable { if goodSince == nil { goodSince = input.now } } else { goodSince = nil }
  let ready = goodSince.map { input.now - $0 >= 8 } ?? false
  if ready && anchor == nil {
   anchor = (input.latitude!, input.longitude!, input.localX, input.localY, -input.heading! * .pi / 180 - input.topYaw!)
  }
  let reason = !input.tracked ? "AR tracking unavailable" : !gpsGood ? "GPS unavailable or inaccurate" : !headingGood ? "Heading uncertain" : !ready ? "Checking geographic alignment" : "GPS and heading reliable"
  return PhoneLocalizationReport(frameID: frameID, mode: ready ? "geographic" : "local", reason: reason,
   tracking: input.tracked ? "normal" : "limited", gpsAccuracy: input.accuracy, headingAccuracy: input.headingAccuracy,
   originLatitude: anchor?.0, originLongitude: anchor?.1, anchorX: anchor?.2, anchorY: anchor?.3, rotation: anchor?.4)
 }
}

/// Establish a fixed world ground plane from a stable ARKit-classified floor.
/// Camera altitude is measured relative to this plane, never guessed from mount height.
struct PhoneGroundPolicy {
 private(set) var offset: Double?
 private var candidate: Double?
 private var since: Double?
 mutating func reset() { offset = nil; candidate = nil; since = nil }
 mutating func observe(height: Double?, cameraHeight: Double, area: Double, now: Double) -> Double? {
  if let offset { return offset }
  guard let height, height.isFinite, cameraHeight.isFinite, now.isFinite,
        area.isFinite, area >= 0.4, cameraHeight-height >= 0.1, cameraHeight-height <= 3 else {
   candidate = nil; since = nil; return nil
  }
  if candidate.map({ abs($0-height) <= 0.03 }) != true { candidate = height; since = now }
  if let since, now-since >= 1 { offset = -height }
  return offset
 }
}
