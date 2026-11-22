"""Convert a full field STEP to a self-contained, metre-scale glTF for a game pack.

Uses the same OpenCascade STEP reader as the robot importer, then instances each
distinct part. The input datum is the centre of the field at carpet height.
"""
import argparse
import hashlib
import json
from pathlib import Path
import re
import struct
import subprocess
import sys
import tempfile


def export(pack, source, args):
    doc = json.loads((pack / "parts.json").read_text(encoding="utf-8"))
    binary = (pack / "meshes.bin").read_bytes()
    data = bytearray()
    gltf = {
        "asset": {"version": "2.0", "generator": "FRC Prototype Simulator field_cad",
                  "extras": {"step": source.name, "sha256": hashlib.sha256(source.read_bytes()).hexdigest()}},
        "scene": 0, "scenes": [{"nodes": [0]}], "nodes": [], "meshes": [],
        "materials": [], "bufferViews": [], "accessors": [],
    }
    def append(table, value):
        result = len(gltf[table])
        gltf[table].append(value)
        return result

    def accessor(offset, count, index=False, bounds=None):
        size = count * (4 if index else 12)
        view = append("bufferViews", {"buffer": 0, "byteOffset": len(data), "byteLength": size,
                                      "target": 34963 if index else 34962})
        data.extend(binary[offset:offset+size])
        acc = {"bufferView": view, "componentType": 5125 if index else 5126,
               "type": "SCALAR" if index else "VEC3", "count": count}
        if bounds:
            acc["min"], acc["max"] = bounds
        return append("accessors", acc)

    # glTF is Y-up. Its +X points from blue toward red, and its -Z is blue's left.
    sign = 1 if args.blue_at == "negative-x" else -1
    orientation = ([sign,0,0,0, 0,1,0,0, 0,0,sign,0, 0,0,0,1] if args.up == "y"
                   else [sign,0,0,0, 0,0,-sign,0, 0,1,0,0, 0,0,0,1])
    origin = append("nodes", {"name": "Field CAD", "matrix": orientation, "children": []})
    omit = re.compile(args.omit, re.I) if args.omit else None
    geometry, materials, meshes = {}, {}, {}
    for part in doc["parts"]:
        key = part["shape"]
        shape = doc["shapes"][key]
        mesh = shape["mesh"]
        name = shape["name"]
        extent = max(b - a for a, b in zip(*shape["bbox"]))
        if not mesh or extent < args.min_size or (omit and omit.search(name)):
            continue
        if key not in geometry:
            off, count = mesh["offset"], mesh["vertices"]
            # Actual float32 bounds are required by glTF, even for shapes whose CAD bounds are looser.
            xyz = list(struct.iter_unpack("<fff", binary[off:off+12*count]))
            bounds = [list(map(min, zip(*xyz))), list(map(max, zip(*xyz)))]
            pos = accessor(off, count, bounds=bounds)
            normal = accessor(off+12*count, count)
            indices = accessor(off+24*count, mesh["indices"], index=True)
            geometry[key] = {"attributes": {"POSITION": pos, "NORMAL": normal}, "indices": indices}
        color, alpha = part["color"] or "#999999", part.get("opacity", 1.0)
        if name == "FE-2024-01" or "playing field carpet" in name.lower():
            color, alpha = "#34373a", 1.0
        # The 2025 STEP assigns opaque colours to the clear guardrail sheets
        # and the upper driver station window. Their shape and placement are intact.
        if name in {"FE-00008", "FE-00009", "FE-00010", "FE-00044", "FE-00004-01"}:
            color, alpha = "#a6c1ce", 0.19
        surface = color, alpha
        if surface not in materials:
            rgba = [(int(color[i:i+2], 16)/255)**2.2 for i in (1,3,5)] + [alpha]
            material = {"pbrMetallicRoughness": {"baseColorFactor": rgba, "metallicFactor": 0.0, "roughnessFactor": 0.84}}
            if alpha < 1.0:
                material.update(alphaMode="BLEND", doubleSided=True)
            materials[surface] = append("materials", material)
        appearance = key, surface
        if appearance not in meshes:
            prim = dict(geometry[key], material=materials[surface])
            meshes[appearance] = append("meshes", {"name": name, "primitives": [prim]})
        matrix = part["m"] + [0, 0, 0, 1]
        node = append("nodes", {"name": name, "mesh": meshes[appearance],
                                "matrix": [matrix[4*r+c] for c in range(4) for r in range(4)]})
        gltf["nodes"][origin]["children"].append(node)
    if not meshes:
        raise ValueError("No field geometry matched the filters")
    gltf["buffers"] = [{"byteLength": len(data)}]
    meta = json.dumps(gltf, separators=(",", ":")).encode()
    meta += b" " * (-len(meta) % 4)
    size = 28 + len(meta) + len(data)
    with args.output.open("wb") as out:
        out.write(struct.pack("<4sII", b"glTF", 2, size))
        out.write(struct.pack("<I4s", len(meta), b"JSON"))
        out.write(meta)
        out.write(struct.pack("<I4s", len(data), b"BIN\0"))
        out.write(data)
    print(f"{args.output}: {len(gltf['nodes'])-1} placed parts, {len(geometry)} distinct shapes, {size:,} bytes")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("step", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--up", choices=["z","y"], default="z")
    parser.add_argument("--blue-at", choices=["negative-x","positive-x"], default="negative-x")
    parser.add_argument("--omit", help="Regex of part names to leave out, e.g. game pieces")
    parser.add_argument("--min-size", type=float, default=0.09, help="Omit parts smaller than this in every dimension (m)")
    parser.add_argument("--cadpack", type=Path, help="Reuse an existing cadread conversion of this STEP")
    args = parser.parse_args()
    if args.cadpack:
        export(args.cadpack, args.step, args)
    else:
        with tempfile.TemporaryDirectory() as tmp:
            reader = Path(__file__).parent / "cadread" / "cadread.py"
            subprocess.run([sys.executable, reader, args.step, tmp, "--lin", "0.003", "--ang", "0.65"], check=True)
            export(Path(tmp), args.step, args)
