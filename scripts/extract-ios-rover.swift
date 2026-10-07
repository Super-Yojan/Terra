// Extract triangle meshes and material colors for the mobile preview conversion.
import SceneKit
import AppKit
let scene = try SCNScene(url: URL(fileURLWithPath:CommandLine.arguments[1]), options:nil)
let output = URL(fileURLWithPath:CommandLine.arguments[2],isDirectory:true)
try FileManager.default.createDirectory(at:output,withIntermediateDirectories:true)
var items:[[String:Any]]=[]
var meshIndex=0
scene.rootNode.enumerateChildNodes { node, _ in
 guard let geometry=node.geometry, let source=geometry.sources(for:.vertex).first else {return}
 do {
  let basename="mesh\(meshIndex)";meshIndex+=1
  try source.data.write(to:output.appendingPathComponent(basename+".vertices"))
  var elements:[[String:Any]]=[]
  for (index,element) in geometry.elements.enumerated() {
   let name=basename+"-\(index).indices"
   try element.data.write(to:output.appendingPathComponent(name))
   let material=geometry.materials[index % geometry.materials.count]
   let color=(material.diffuse.contents as? NSColor)?.usingColorSpace(.sRGB) ?? .gray
   let linear = [color.redComponent, color.greenComponent, color.blueComponent].map { value in value <= 0.04045 ? value / 12.92 : pow((value + 0.055) / 1.055, 2.4) }
   elements.append(["file":name,"bytesPerIndex":element.bytesPerIndex,"primitiveCount":element.primitiveCount,"type":element.primitiveType.rawValue,"color":linear])
  }
  let m=node.simdWorldTransform
  items.append(["vertices":basename+".vertices","count":source.vectorCount,"offset":source.dataOffset,"stride":source.dataStride,"bytesPerComponent":source.bytesPerComponent,"float":source.usesFloatComponents,"matrix":[[m.columns.0.x,m.columns.0.y,m.columns.0.z,m.columns.0.w],[m.columns.1.x,m.columns.1.y,m.columns.1.z,m.columns.1.w],[m.columns.2.x,m.columns.2.y,m.columns.2.z,m.columns.2.w],[m.columns.3.x,m.columns.3.y,m.columns.3.z,m.columns.3.w]],"elements":elements])
 } catch {fatalError("Extraction failed: \(error)")}
}
try JSONSerialization.data(withJSONObject:items,options:.prettyPrinted).write(to:output.appendingPathComponent("meshes.json"))
print("Extracted \(items.count) meshes")
