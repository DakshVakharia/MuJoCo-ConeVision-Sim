"""Procedural Formula Student cone meshes (square base plate + truncated cone + stripes).

All meshes are inline (vertex/face attributes), so no files are needed. Every cone is one body
`cone_<class>_<index>`; plate, cone and stripe bands are geoms of that body.
"""
import math
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

SMALL_CLASSES = ("blue", "yellow", "orange")
STRIPE_DARK_ON = ("yellow",)          # black stripe on yellow cones, white on the rest


def size_key(cls):
    return "big" if cls == "orange_big" else "small"


def revolve_solid(profile, n):
    """Closed solid of revolution about z from a profile [(r, z), ...] bottom -> top.

    The solid is capped at the first and last profile point. Returns (verts (V,3), faces (F,3)),
    faces counter-clockwise seen from outside.
    """
    profile = list(profile)
    ang = np.linspace(0, 2 * math.pi, n, endpoint=False)
    verts, rings = [], []
    for r, z in profile:
        rings.append(len(verts))
        verts += [(r * math.cos(a), r * math.sin(a), z) for a in ang]
    faces = []
    for k in range(len(profile) - 1):
        b, t = rings[k], rings[k + 1]
        for i in range(n):
            j = (i + 1) % n
            faces.append((b + i, b + j, t + j))
            faces.append((b + i, t + j, t + i))
    zb, zt = profile[0][1], profile[-1][1]
    cb = len(verts); verts.append((0.0, 0.0, zb))
    ct = len(verts); verts.append((0.0, 0.0, zt))
    for i in range(n):
        j = (i + 1) % n
        faces.append((cb, rings[0] + j, rings[0] + i))             # bottom cap faces down
        faces.append((ct, rings[-1] + i, rings[-1] + j))           # top cap faces up
    return np.array(verts), np.array(faces)


def cone_radius_at(z, h, t, r0, r1):
    return r0 + (r1 - r0) * (z - t) / (h - t)


def box_solid(hx, hy, z0, z1):
    """Closed axis-aligned box (verts, faces) with unshared vertices so it is shaded flat."""
    c = np.array([(-hx, -hy, z0), (hx, -hy, z0), (hx, hy, z0), (-hx, hy, z0),
                  (-hx, -hy, z1), (hx, -hy, z1), (hx, hy, z1), (-hx, hy, z1)])
    quads = [(0, 3, 2, 1), (4, 5, 6, 7), (0, 1, 5, 4), (1, 2, 6, 5), (2, 3, 7, 6), (3, 0, 4, 7)]
    v = np.vstack([c[list(q)] for q in quads])
    f = np.array([(4 * k + a, 4 * k + b, 4 * k + d) for k in range(6)
                  for a, b, d in ((0, 1, 2), (0, 2, 3))])
    return v, f


def _merge(parts):
    """parts: [(verts, faces, palette_index)] -> verts, faces, per-vertex palette index."""
    V, F, K, off = [], [], [], 0
    for v, f, k in parts:
        V.append(v)
        F.append(f + off)
        K += [k] * len(v)
        off += len(v)
    return np.vstack(V), np.vstack(F), np.array(K)


def add_cone_assets(asset, scene_cfg, tex_dir):
    """Add one mesh per cone class (plate + cone + stripes merged, colours via a palette texture).

    One geom per cone keeps the geom count (and render time) low. Returns the spec used by add_cone_body.
    """
    from .textures import write_palette
    cc = scene_cfg["cones"]
    n = int(cc.get("segments", 28))
    t = float(cc.get("plate_thickness", 0.015))
    col = cc["colors"]
    names = ["blue", "yellow", "orange", "orange_big", "stripe_light", "stripe_dark"]
    pal = {nm: k for k, nm in enumerate(names)}
    write_palette([col[nm] for nm in names], Path(tex_dir) / "cone_palette.png")
    ET.SubElement(asset, "texture", name="cone_palette", type="2d", file="cone_palette.png")
    ET.SubElement(asset, "material", name="cone_mat", texture="cone_palette", specular="0.15", shininess="0.25")
    spec = {}
    for cls in cone_classes():
        key = size_key(cls)
        bw, h = cc[key]["base_width"], cc[key]["height"]
        r0, r1 = bw * cc["bottom_radius_frac"], bw * cc["top_radius_frac"]
        parts = [(*box_solid(bw / 2, bw / 2, 0.0, t), pal[cls]),
                 (*revolve_solid([(r0, t * 0.5), (r0, t), (r1, h)], n), pal[cls])]
        stripe = pal["stripe_dark" if cls in STRIPE_DARK_ON else "stripe_light"]
        for a, b in cc["stripes"][key]:
            za, zb, eps = max(a * h, t + 0.002), b * h, 0.0025
            ra, rb = cone_radius_at(za, h, t, r0, r1), cone_radius_at(zb, h, t, r0, r1)
            parts.append((*revolve_solid([(ra * 0.5, za), (ra + eps, za + 0.0005),
                                          (rb + eps, zb - 0.0005), (rb * 0.5, zb)], n), stripe))
        v, f, k = _merge(parts)
        uv = np.stack([(k + 0.5) / len(names), np.full(len(k), 0.5)], axis=1)
        ET.SubElement(asset, "mesh", name=f"cone_{cls}_mesh", inertia="shell",
                      vertex=" ".join(f"{x:.5f}" for x in v.ravel()),
                      face=" ".join(str(int(i)) for i in f.ravel()),
                      texcoord=" ".join(f"{x:.4f}" for x in uv.ravel()))
        spec[cls] = f"cone_{cls}_mesh"
    return spec


def cone_classes():
    return ("blue", "yellow", "orange", "orange_big")


def add_cone_body(worldbody, spec, cls, idx, x, y, name=None):
    """Add one cone as a body with its (single) geom as child."""
    body = ET.SubElement(worldbody, "body", name=name or f"cone_{cls}_{idx}", pos=f"{x:.4f} {y:.4f} 0")
    ET.SubElement(body, "geom", type="mesh", mesh=spec[cls], material="cone_mat",
                  contype="0", conaffinity="0")
    return body
