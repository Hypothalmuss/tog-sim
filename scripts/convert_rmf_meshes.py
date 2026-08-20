#!/usr/bin/env python3
"""Convert Open-RMF glTF (.glb) workcell models into Gazebo-Fortress-friendly models.

Fortress (ign-rendering 6 / Ogre-Next 2.2) cannot load glTF, and multi-object OBJ exports crash its mesh importer.
We therefore write ONE single-object OBJ (with normals) PER MATERIAL and build an SDF with one <visual> per
material carrying the PBR base colour, keeping the original collision geometry from the Fuel model.sdf.

Usage: convert_rmf_meshes.py <fuel_cache_model_dir> <out_model_dir> <model_name>
"""

import glob
import os
import re
import sys

import numpy as np
import trimesh


def main():
    src_dir, out_dir, name = sys.argv[1:4]
    glb = glob.glob(os.path.join(src_dir, "meshes", "*.glb"))[0]
    scene = trimesh.load(glb, force="scene")
    # group geometry instances by material
    by_mat = {}
    for node in scene.graph.nodes_geometry:
        tf, gname = scene.graph[node]
        g = scene.geometry[gname].copy()
        g.apply_transform(tf)
        mat = getattr(getattr(g, "visual", None), "material", None)
        key = getattr(mat, "name", None) or "material"
        color = None
        if mat is not None:
            bc = getattr(mat, "baseColorFactor", None)
            if bc is None and getattr(mat, "main_color", None) is not None:
                bc = mat.main_color
            if bc is not None:
                color = (np.array(bc[:4], dtype=float) / (255.0 if np.max(bc) > 1.0 else 1.0)).tolist()
        by_mat.setdefault(key, {"meshes": [], "color": color})
        by_mat[key]["meshes"].append(g)
    os.makedirs(os.path.join(out_dir, "meshes"), exist_ok=True)
    visuals = []
    for i, (key, d) in enumerate(by_mat.items()):
        m = trimesh.util.concatenate(d["meshes"])
        m.merge_vertices()
        m.visual = trimesh.visual.ColorVisuals(m)
        fname = f"{name}_{i}.obj"
        m.export(os.path.join(out_dir, "meshes", fname), include_normals=True, include_texture=False)
        # strip any mtllib/usemtl lines: the colour comes from the SDF material
        p = os.path.join(out_dir, "meshes", fname)
        lines = [ln for ln in open(p).read().splitlines() if not ln.startswith(("mtllib", "usemtl"))]
        open(p, "w").write("\n".join(lines) + "\n")
        for mtl in glob.glob(os.path.join(out_dir, "meshes", "*.mtl")):
            os.remove(mtl)
        c = d["color"] or [0.6, 0.6, 0.62, 1.0]
        if len(c) == 3:
            c.append(1.0)
        rgba = " ".join(f"{v:.3f}" for v in c)
        visuals.append(
            f"""            <visual name="visual_{i}">
                <geometry><mesh><uri>model://{name}/meshes/{fname}</uri></mesh></geometry>
                <material><ambient>{rgba}</ambient><diffuse>{rgba}</diffuse>
                  <specular>0.2 0.2 0.2 1</specular></material>
            </visual>"""
        )
        print(f"  {fname}: material '{key}' {len(m.vertices)} verts {len(m.faces)} faces color {rgba}")
    # original SDF: keep collisions, replace the visual block
    sdf = open(glob.glob(os.path.join(src_dir, "model.sdf"))[0]).read()
    sdf = re.sub(r"<visual name=\"[^\"]+\">.*?</visual>", "\n".join(visuals), sdf, count=1, flags=re.S)
    sdf = re.sub(r'<model name="[^"]+">', f'<model name="{name}">', sdf, count=1)
    if name.startswith("enclosure"):  # scenery only: conveyors run through the walls
        sdf = re.sub(r"\s*<collision name=\"[^\"]+\">.*?</collision>", "", sdf, flags=re.S)
    if "<static>" not in sdf and name != "tray_rmf":
        sdf = sdf.replace(f'<model name="{name}">', f'<model name="{name}">\n        <static>true</static>', 1)
    if name == "tray_rmf" and "<inertial>" not in sdf:
        sdf = re.sub(
            r"(<link name=\"[^\"]+\">)",
            r"\1\n            <inertial><mass>0.4</mass>"
            r"<inertia><ixx>0.01</ixx><iyy>0.01</iyy><izz>0.02</izz></inertia></inertial>",
            sdf,
            count=1,
        )
    open(os.path.join(out_dir, "model.sdf"), "w").write(sdf)
    open(os.path.join(out_dir, "model.config"), "w").write(
        f"""<?xml version="1.0"?>
<model>
  <name>{name}</name>
  <version>1.0</version>
  <sdf version="1.8">model.sdf</sdf>
  <author><name>Open-RMF (converted by tog-sim/scripts/convert_rmf_meshes.py)</name></author>
  <description>Open-RMF workcell model (CC BY 4.0), glTF converted to per-material OBJ (Fortress).</description>
</model>
"""
    )


if __name__ == "__main__":
    main()
