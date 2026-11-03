import Foundation
struct ActuatorIndexEntry: Codable, Equatable, Identifiable { var id: Int; var name: String; var kind: String }
struct ActuatorLayoutIndex: Codable, Equatable {
 var revision: UInt32; var layout_available: Bool; var actuator_count: Int; var actuators: [ActuatorIndexEntry]
}
struct ActuatorPaginationState {
 var revision: UInt32 = 0
 var generation: UInt64 = 0
 var entries: [ActuatorIndexEntry] = []
 var selectedID: Int?
 var selected: ActuatorDraft?
 var dirty = false
 var token: String?
 var version: UInt32 = 0
 var validatedVersion: UInt32?
 var cache: [Int: ActuatorDraft] = [:]
 var canCommit: Bool { token != nil && validatedVersion == version && !dirty }
 mutating func invalidate(generation: UInt64) {
  let unsent = dirty ? selected : nil
  self = Self(); self.generation = generation
  if let unsent { selected = unsent; selectedID = unsent.id; dirty = true }
 }
 mutating func synchronize(index: ActuatorLayoutIndex, generation: UInt64) {
  if self.generation != generation || revision != index.revision {
   invalidate(generation: generation); revision = index.revision; entries = index.actuators
  } else if token == nil { entries = index.actuators }
 }
 mutating func select(_ id: Int) -> Bool {
  guard !dirty else { return false }; selectedID = id; selected = cache[id]; return true
 }
 mutating func acceptPage(_ actuator: ActuatorDraft, revision: UInt32, generation: UInt64) -> Bool {
  guard self.revision == revision, self.generation == generation, selectedID == actuator.id, !dirty else { return false }
  cache[actuator.id] = actuator; selected = actuator; return true
 }
 mutating func change(_ actuator: ActuatorDraft) { selectedID = actuator.id; selected = actuator; dirty = true; validatedVersion = nil }
 mutating func discardLocal() { dirty = false; selected = selectedID.flatMap { cache[$0] } }
 mutating func begin(token: String, base: UInt32, version: UInt32, replacement: Bool) {
  self.token = token; revision = base; self.version = version; validatedVersion = nil
  if replacement { entries = []; cache = [:]; selected = nil; selectedID = nil; dirty = false }
 }
 mutating func acknowledge(_ actuator: ActuatorDraft, version: UInt32) {
  self.version = version; validatedVersion = nil; cache[actuator.id] = actuator
  let entry = ActuatorIndexEntry(id: actuator.id, name: actuator.name, kind: actuator.kind)
  if let i = entries.firstIndex(where: { $0.id == actuator.id }) { entries[i] = entry } else { entries.append(entry) }
  if selectedID == actuator.id { selected = actuator; dirty = false }
 }
 mutating func removed(_ id: Int, version: UInt32) {
  self.version = version; validatedVersion = nil; entries.removeAll { $0.id == id }; cache[id] = nil
  if selectedID == id { selectedID = nil; selected = nil; dirty = false }
 }
}

/// Advances only after the current actuator has been acknowledged by the rover.
struct ActuatorPresetUpload {
 private var remaining: [ActuatorDraft]
 init(actuators: [ActuatorDraft]) { remaining = actuators }
 var current: ActuatorDraft? { remaining.first }
 var needsPortSelection: Bool { current?.port.isEmpty == true }
 mutating func updateCurrent(_ actuator: ActuatorDraft) {
  guard !remaining.isEmpty else { return }; remaining[0] = actuator
 }
 @discardableResult mutating func acknowledge(id: Int) -> Bool {
  guard current?.id == id else { return false }
  remaining.removeFirst(); return true
 }
}
