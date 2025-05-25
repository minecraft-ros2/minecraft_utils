#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
URDF → Geckolib JSON 変換

対応:
- mesh ファイル (OBJ/STL/etc.)
- box, cylinder, sphere のプリミティブ形状
- package:// パスの基本サポート
- URDF のリンク階層を考慮した relative transforms
"""

import argparse
import json
from pathlib import Path

import numpy as np
import trimesh
from urdfpy import URDF

# NumPy >=1.20 対策: np.float のエイリアスを復活
if not hasattr(np, 'float'):
    np.float = float


def voxel_to_cubes(matrix: np.ndarray, pitch: float, atlas_u: int, atlas_v: int):
    cubes = []
    for x, y, z in np.argwhere(matrix):
        origin = [float(x) * pitch * 16,
                  float(z) * pitch * 16,
                  float(y) * pitch * 16]
        size   = [pitch * 16, pitch * 16, pitch * 16]
        cubes.append({
            "origin": origin,
            "size":   size,
            "uv":     [atlas_u, atlas_v],
            "inflate": 0.0
        })
    return cubes


def resolve_path(filename: str, base: Path) -> Path:
    if filename.startswith("package://"):
        rel = filename.replace("package://", "")
        return base.parent / rel
    return base.parent / filename


def load_urdf_mesh(urdf_path: Path) -> trimesh.Trimesh:
    robot = URDF.load(str(urdf_path))
    joint_map = {j.child: j for j in robot.joints}
    meshes = []

    for link in robot.links:
        for visual in link.visuals:
            geom = visual.geometry
            mesh_obj = None

            # mesh ファイル
            if geom.mesh is not None and geom.mesh.filename:
                mesh_file = resolve_path(geom.mesh.filename, urdf_path)
                mesh_obj = trimesh.load(mesh_file, force='mesh')

            # box プリミティブ
            elif geom.box is not None:
                mesh_obj = trimesh.creation.box(extents=geom.box.size)

            # cylinder プリミティブ
            elif geom.cylinder is not None:
                mesh_obj = trimesh.creation.cylinder(radius=geom.cylinder.radius, height=geom.cylinder.length, sections=32)

            # sphere プリミティブ
            elif geom.sphere is not None:
                mesh_obj = trimesh.creation.icosphere(radius=geom.sphere.radius, subdivisions=3)

            if mesh_obj is None:
                continue

            # URDF hierarchy transforms: origin may be ndarray or Pose
            origin = visual.origin
            if isinstance(origin, np.ndarray):
                T = origin
            else:
                T = origin.matrix
            # accumulate parent joint origins
            curr = link.name
            while curr in joint_map:
                joint = joint_map[curr]
                jorig = joint.origin
                if isinstance(jorig, np.ndarray):
                    T = jorig @ T
                else:
                    T = jorig.matrix @ T
                curr = joint.parent

            mesh_obj.apply_transform(T)
            meshes.append(mesh_obj)

    if not meshes:
        raise RuntimeError(f"URDF 内に mesh/primitive が見つかりませんでした: {urdf_path}")
    return trimesh.util.concatenate(meshes)


def main():
    p = argparse.ArgumentParser(description="URDF → Geckolib .geo.json 変換")
    p.add_argument("--urdf",       type=Path, required=True, help="入力 URDF ファイル")
    p.add_argument("--output",     type=Path, required=True, help="出力 .geo.json ファイル")
    p.add_argument("--pitch",      type=float, default=0.05,     help="Voxel サイズ (m)")
    p.add_argument("--atlas-u",    type=int,   default=0,       help="UV 左上 U 座標")
    p.add_argument("--atlas-v",    type=int,   default=0,       help="UV 左上 V 座標")
    p.add_argument("--identifier", type=str,   default="model", help="geometry.identifier")
    p.add_argument("--tex-size",   type=int, nargs=2, metavar=('W','H'), default=[16,16], help="texture_width, texture_height")
    args = p.parse_args()

    print("Loading and merging meshes from URDF…")
    mesh = load_urdf_mesh(args.urdf)
    # 一旦objファイルに保存
    mesh.export("output.obj", file_type='obj')
    extents = mesh.extents
    min_dim = float(extents.min())
    print(f"Mesh extents (x,y,z): {extents}. Minimum dimension: {min_dim:.3f} m.")
    rec_pitch = min_dim / 16
    print(f"Current pitch: {args.pitch}. 推奨 pitch <= {rec_pitch:.4f} for capturing all parts.")

    print(f"Voxelizing mesh at pitch={args.pitch} m…")
    vox = mesh.voxelized(args.pitch)
    matrix = vox.matrix

    print("Building JSON data…")
    cubes = voxel_to_cubes(matrix, args.pitch, args.atlas_u, args.atlas_v)
    geo = {
        "format_version": "1.12.0",
        "minecraft:geometry": [{
            "description": {"identifier": args.identifier,
                              "texture_width": args.tex_size[0],
                              "texture_height": args.tex_size[1]},
            "bones": [{"name": "root","pivot": [0,0,0],"cubes": cubes}]
        }]
    }

    args.output.parent.mkdir(parents=True, exist_ok=True)
    with open(args.output, 'w', encoding='utf-8') as f:
        json.dump(geo, f, ensure_ascii=False, indent=2)
    print(f"Generated: {args.output}")

if __name__ == "__main__":
    main()
