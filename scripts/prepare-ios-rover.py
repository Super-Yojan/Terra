#!/usr/bin/env python3
"""Build the iOS preview asset from a CAD USD or OBJ and its neighboring MTL.

Requires numpy, trimesh, fast-simplification and Apple's usdcat/usdzip tools.
Example: python3 scripts/prepare-ios-rover.py ~/Downloads/robot.usdc
The input files are never modified. Small material groups stay intact.
"""
import argparse
import json
from pathlib import Path
import subprocess
import tempfile
import numpy as np
import trimesh


def load_model(source: Path) -> trimesh.Scene:
    if source.suffix.lower() not in {".usd", ".usda", ".usdc", ".usdz"}:
        return trimesh.load(source, force="scene", process=False)
    scene = trimesh.Scene()
    with tempfile.TemporaryDirectory(prefix="terra-usd-") as temporary:
        work = Path(temporary)
        extractor = Path(__file__).with_name("extract-ios-rover.swift")
        subprocess.run(["swift", "-module-cache-path", str(work / "cache"),
                        str(extractor), str(source), str(work)], check=True)
        for item in json.loads((work / "meshes.json").read_text()):
            if not item["float"] or item["bytesPerComponent"] not in (4, 8):
                raise ValueError("Unsupported vertex encoding")
            vertices = np.ndarray(
                shape=(item["count"], 3), dtype=f'<f{item["bytesPerComponent"]}',
                buffer=(work / item["vertices"]).read_bytes(), offset=item["offset"],
                strides=(item["stride"], item["bytesPerComponent"])).copy()
            transform = np.asarray(item["matrix"]).T
            vertices = vertices @ transform[:3, :3].T + transform[:3, 3]
            for element in item["elements"]:
                if element["type"] != 0:
                    raise ValueError("Export the CAD model as a triangle mesh")
                indices = np.fromfile(work / element["file"], dtype=f'<u{element["bytesPerIndex"]}')
                channels = len(indices) // (element["primitiveCount"] * 3)
                # Imported USD can interleave position, normal and UV indices.
                faces = indices.reshape(-1, channels)[:, 0].reshape(-1, 3)
                mesh = trimesh.Trimesh(vertices=vertices, faces=faces, process=False)
                mesh.remove_unreferenced_vertices()
                color = np.append(np.asarray(element["color"]) * 255, 255).astype(np.uint8)
                mesh.visual = trimesh.visual.ColorVisuals(mesh=mesh, vertex_colors=color)
                scene.add_geometry(mesh)
    return scene


def prepare(source: Path, output: Path, target: int) -> None:
    scene = load_model(source)
    before = sum(len(mesh.faces) for mesh in scene.geometry.values())
    if before == 0:
        raise ValueError("The model contains no triangle faces")
    output.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix="terra-rover-") as temporary:
        work = Path(temporary)
        materials, colors = [], {}
        offset = 0
        with (work / "rover.obj").open("w") as obj:
            obj.write("mtllib rover.mtl\n")
            for index, original in enumerate(scene.geometry.values()):
                diffuse = (original.visual.material.diffuse if hasattr(original.visual, "material")
                           else original.visual.main_color)
                color = np.asarray(diffuse[:3], dtype=float) / 255
                mesh = original.copy()
                if len(mesh.faces) > 400:
                    mesh.merge_vertices(merge_tex=True, merge_norm=True)
                    mesh = mesh.simplify_quadric_decimation(
                        face_count=max(200, round(len(mesh.faces) * target / before)))
                # Preserve CAD Z-up here. The iOS viewer adapts it to SceneKit Y-up.
                name = f"material{index}"
                colors[name] = color.tolist()
                materials.extend([f"newmtl {name}", "Kd " + " ".join(map(str, color)), ""])
                obj.write(f"o part{index}\nusemtl {name}\n")
                for vertex in mesh.vertices:
                    obj.write("v %.7f %.7f %.7f\n" % tuple(vertex))
                for face in mesh.faces:
                    obj.write("f %d %d %d\n" % tuple(face + offset + 1))
                offset += len(mesh.vertices)
        (work / "rover.mtl").write_text("\n".join(materials))
        subprocess.run(["usdcat", str(work / "rover.obj"), "-o", str(work / "rover.usdc")], check=True)
        subprocess.run(["usdzip", "--arkitAsset", str(work / "rover.usdc"), str(output / "rover.usdz")], check=True)
        (output / "materials.json").write_text(json.dumps(colors, indent=2) + "\n")
    print(f"Prepared {output / 'rover.usdz'} from {before:,} source triangles")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", type=Path)
    parser.add_argument("--target-faces", type=int, default=120000,
                        help="Simplification target; disconnected CAD parts can require more faces")
    parser.add_argument("--output", type=Path,
                        default=Path(__file__).resolve().parents[1] / "mobile/ios/TerraPhone/Models")
    args = parser.parse_args()
    if args.target_faces <= 0:
        parser.error("--target-faces must be positive")
    prepare(args.source.resolve(), args.output.resolve(), args.target_faces)
