"""Roadside scenery: placement (seeded, collision free) and MJCF emission.

Uses the converted CC0 meshes in assets/scenery (catalog.json); falls back to procedural
primitives (cylinder trunks + ellipsoid canopies, tyre stacks, hay bales, boxes) when the
assets are missing or `scenery.use_assets` is false.
"""
import json
import math
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
from scipy.spatial import cKDTree

REPO_ROOT = Path(__file__).resolve().parents[3]

# --- procedural fallbacks: name -> category, footprint radius, track-aligned?, geoms -----------
# geom = (type, size, pos, euler_deg, rgba)
_G, _B = (0.2, 0.45, 0.15, 1), (0.36, 0.25, 0.16, 1)
PROCEDURAL = {
    "proc_tree": dict(category="tree", radius=1.6, align=False, height=5.0, geoms=[
        ("cylinder", (0.18, 1.3), (0, 0, 1.3), (0, 0, 0), _B),
        ("ellipsoid", (1.7, 1.7, 2.0), (0, 0, 3.6), (0, 0, 0), _G)]),
    "proc_tree_dark": dict(category="tree", radius=1.4, align=False, height=5.5, geoms=[
        ("cylinder", (0.16, 1.2), (0, 0, 1.2), (0, 0, 0), _B),
        ("ellipsoid", (1.4, 1.4, 2.4), (0, 0, 3.8), (0, 0, 0), (0.12, 0.32, 0.12, 1))]),
    "proc_pine": dict(category="tree", radius=1.2, align=False, height=8.0, geoms=[
        ("cylinder", (0.15, 1.0), (0, 0, 1.0), (0, 0, 0), _B),
        ("ellipsoid", (1.1, 1.1, 3.0), (0, 0, 4.5), (0, 0, 0), (0.10, 0.30, 0.14, 1))]),
    "tyre_stack": dict(category="object", radius=0.55, align=False, height=0.8, geoms=[
        ("cylinder", (0.33, 0.1), (0, 0, 0.1), (0, 0, 0), (0.10, 0.10, 0.10, 1)),
        ("cylinder", (0.33, 0.1), (0.02, 0, 0.3), (0, 0, 0), (0.12, 0.12, 0.12, 1)),
        ("cylinder", (0.33, 0.1), (-0.01, 0.02, 0.5), (0, 0, 0), (0.10, 0.10, 0.10, 1)),
        ("cylinder", (0.33, 0.1), (0.0, 0, 0.7), (0, 0, 0), (0.12, 0.12, 0.12, 1))]),
    "hay_bale": dict(category="object", radius=0.9, align=True, height=1.2, geoms=[
        ("cylinder", (0.6, 0.6), (0, 0, 0.6), (90, 0, 0), (0.82, 0.68, 0.30, 1))]),
    "crate": dict(category="object", radius=0.6, align=False, height=0.8, geoms=[
        ("box", (0.4, 0.4, 0.4), (0, 0, 0.4), (0, 0, 0), (0.5, 0.36, 0.2, 1))]),
    "traffic_barrel": dict(category="object", radius=0.4, align=False, height=0.9, geoms=[
        ("cylinder", (0.28, 0.45), (0, 0, 0.45), (0, 0, 0), (0.1, 0.25, 0.7, 1)),
        ("cylinder", (0.285, 0.06), (0, 0, 0.55), (0, 0, 0), (0.9, 0.9, 0.9, 1))]),
    "barrier_block": dict(category="object", radius=1.1, align=True, height=0.8, geoms=[
        ("box", (1.0, 0.25, 0.4), (0, 0, 0.4), (0, 0, 0), (0.85, 0.85, 0.85, 1))]),
    "gazebo": dict(category="structure", radius=2.2, align=False, height=3.0, geoms=[
        ("box", (2.0, 2.0, 0.05), (0, 0, 2.7), (0, 0, 0), (0.9, 0.9, 0.9, 1)),
        ("cylinder", (0.06, 1.3), (1.8, 1.8, 1.3), (0, 0, 0), (0.8, 0.8, 0.8, 1)),
        ("cylinder", (0.06, 1.3), (-1.8, 1.8, 1.3), (0, 0, 0), (0.8, 0.8, 0.8, 1)),
        ("cylinder", (0.06, 1.3), (1.8, -1.8, 1.3), (0, 0, 0), (0.8, 0.8, 0.8, 1)),
        ("cylinder", (0.06, 1.3), (-1.8, -1.8, 1.3), (0, 0, 0), (0.8, 0.8, 0.8, 1))]),
    "rock": dict(category="rock", radius=0.9, align=False, height=0.8, geoms=[
        ("ellipsoid", (0.9, 0.7, 0.4), (0, 0, 0.25), (0, 0, 0), (0.45, 0.45, 0.45, 1))]),
    "bush": dict(category="bush", radius=0.8, align=False, height=1.0, geoms=[
        ("ellipsoid", (0.8, 0.8, 0.55), (0, 0, 0.45), (0, 0, 0), (0.18, 0.40, 0.14, 1))]),
}
CATEGORY_WEIGHT = {"object": 1.0, "bush": 0.6, "rock": 0.5, "structure": 0.12}
STRUCTURE_MAX_PER_NAME = 1


def load_catalog(scfg):
    """Return (catalog dict, mesh dir, palette png path); ({}, None, None) if unavailable / disabled."""
    if not scfg.get("use_assets", True):
        return {}, None, None
    d = Path(scfg.get("asset_dir", "assets/scenery"))
    d = d if d.is_absolute() else REPO_ROOT / d
    f = d / "catalog.json"
    if not f.exists():
        return {}, None, None
    cat = json.loads(f.read_text())
    pal = d / cat.pop("_palette", {}).get("file", "palette.png")
    if not pal.exists():
        return {}, None, None
    cat = {k: v for k, v in cat.items() if (d / "meshes" / v["file"]).exists()}
    return cat, d / "meshes", pal


def _dense_centerline(c, step=0.5):
    closed = np.vstack([c, c[:1]])
    seg = np.linalg.norm(np.diff(closed, axis=0), axis=1)
    s = np.concatenate([[0], np.cumsum(seg)])
    t = np.arange(0, s[-1], step)
    return np.stack([np.interp(t, s, closed[:, 0]), np.interp(t, s, closed[:, 1])], axis=1)


def _definitions(catalog):
    defs = {k: dict(v, kind="mesh") for k, v in catalog.items()}
    for k, v in PROCEDURAL.items():
        # procedural trees only when no mesh trees exist; extras (tyres, hay, ...) always
        if v["category"] == "tree" and any(d["category"] == "tree" for d in defs.values()):
            continue
        if v["category"] in ("bush", "rock", "structure") and any(d["category"] == v["category"] for d in defs.values()):
            continue
        defs[k] = dict(v, kind="proc")
    return defs


def place_scenery(track, scfg, catalog):
    """Return a list of placement dicts {name, category, x, y, yaw (rad), scale, fp}."""
    if not scfg.get("enabled", True):
        return []
    rng = np.random.default_rng(int(scfg.get("seed", 0)))
    defs = _definitions(catalog)
    dense = _dense_centerline(track.centerline)
    ctree = cKDTree(dense)
    allc = np.array([[x, y] for _, _, x, y in track.all_cones()]).reshape(-1, 2)
    cone_tree = cKDTree(allc) if len(allc) else None
    nxt = np.roll(dense, -1, axis=0)
    tang = nxt - np.roll(dense, 1, axis=0)
    tang /= np.linalg.norm(tang, axis=1, keepdims=True) + 1e-9
    normals = np.stack([-tang[:, 1], tang[:, 0]], axis=1)
    sx, sy, _ = track.start_pose
    lo, hi = scfg.get("scale_range", [0.8, 1.25])
    scales = np.linspace(lo, hi, 3)
    spacing = float(scfg.get("spacing_m", 0.4))
    clearance = float(scfg.get("cone_clearance_m", 2.0))
    placed, counts = [], {}

    def fp_of(d, scale):
        r = d["radius"] * scale
        return r * (0.45 if d["category"] == "tree" else 0.8)

    def try_place(name, dmin, dmax, forced=None):
        d = defs[name]
        scale = float(rng.choice(scales))
        fp = fp_of(d, scale)
        for _ in range(250):
            if forced is None:
                i = int(rng.integers(len(dense)))
                dist = rng.uniform(dmin, dmax)
                p = dense[i] + rng.choice([-1, 1]) * dist * normals[i]
            else:
                p = forced
            dc, j = ctree.query(p)
            if dc < max(dmin, 0.0) + (0.5 * fp if d["category"] != "tree" else 0.0) or dc > dmax:
                if forced is None:
                    continue
                return None
            if cone_tree is not None and cone_tree.query(p)[0] < clearance + fp:
                if forced is None:
                    continue
                return None
            if math.hypot(p[0] - sx, p[1] - sy) < clearance + fp:
                continue
            if any(math.hypot(p[0] - o["x"], p[1] - o["y"]) < fp + o["fp"] + spacing for o in placed):
                if forced is None:
                    continue
                return None
            yaw = math.atan2(tang[j][1], tang[j][0]) + rng.normal(0, 0.05) if d["align"] else rng.uniform(0, 2 * math.pi)
            item = dict(name=name, category=d["category"], x=float(p[0]), y=float(p[1]),
                        yaw=float(yaw), scale=scale, fp=fp, tang=tang[j].copy())
            placed.append(item)
            counts[name] = counts.get(name, 0) + 1
            return item
        return None

    def pick(cats, weights=None):
        names = [n for n, d in defs.items() if d["category"] in cats
                 and not (d["category"] == "structure" and counts.get(n, 0) >= STRUCTURE_MAX_PER_NAME)]
        if not names:
            return None
        w = np.array([CATEGORY_WEIGHT.get(defs[n]["category"], 1.0) for n in names])
        return names[int(rng.choice(len(names), p=w / w.sum()))]

    dmin, dmax = float(scfg["min_dist_from_track_m"]), float(scfg["max_dist_from_track_m"])
    # big structures first (hardest to fit)
    for _ in range(min(2, int(scfg.get("num_objects", 0)) // 12)):
        n = pick(("structure",))
        if n:
            try_place(n, dmin + 2, dmax)
    for _ in range(int(scfg.get("num_trees", 0))):
        n = pick(("tree",))
        if n:
            try_place(n, dmin, dmax)
    n_obj = int(scfg.get("num_objects", 0))
    made = len([p for p in placed if p["category"] == "structure"])
    while made < n_obj:
        n = pick(("object", "bush", "rock"))
        if n is None:
            break
        item = try_place(n, dmin, dmax)
        made += 1
        if item and defs[n]["align"]:        # barriers / bales form short rows along the track
            for k in range(int(rng.integers(0, 4))):
                step = 2.0 * defs[n]["radius"] * item["scale"] * 0.95
                q = np.array([item["x"], item["y"]]) + item["tang"] * step * (k + 1)
                if try_place(n, dmin, dmax, forced=q):
                    made += 1
    for _ in range(int(scfg.get("num_far_trees", 0))):
        n = pick(("tree",))
        if n:
            try_place(n, float(scfg.get("far_min_dist_m", 60)), float(scfg.get("far_max_dist_m", 140)))
    for p in placed:
        p.pop("tang", None)
    return placed


def _material(asset, cache, rgba):
    key = tuple(round(float(v), 3) for v in rgba)
    if key not in cache:
        name = f"scn_mat_{len(cache)}"
        ET.SubElement(asset, "material", name=name, rgba=" ".join(str(v) for v in key),
                      specular="0.05", shininess="0.1")
        cache[key] = name
    return cache[key]


def emit_scenery(asset, worldbody, placed, catalog, mesh_src, palette_tex="scenery_palette"):
    """Write scenery meshes/bodies into the MJCF. Returns the mesh file names used.

    Mesh assets use one geom per instance; colours come from the shared palette texture
    (material `scn_palette`, declared by the caller as texture `palette_tex`).
    """
    defs = _definitions(catalog)
    matcache, meshes, used_files = {}, set(), set()
    vis = dict(contype="0", conaffinity="0")
    pal_made = False
    for k, it in enumerate(placed):
        d, s = defs[it["name"]], it["scale"]
        body = ET.SubElement(worldbody, "body", name=f"scenery_{k}_{it['name']}",
                             pos=f"{it['x']:.3f} {it['y']:.3f} 0", euler=f"0 0 {math.degrees(it['yaw']):.2f}")
        if d["kind"] == "mesh":
            if not pal_made:
                ET.SubElement(asset, "material", name="scn_palette", texture=palette_tex,
                              specular="0.05", shininess="0.1")
                pal_made = True
            mname = f"{it['name']}_s{s:.2f}"
            if mname not in meshes:
                ET.SubElement(asset, "mesh", name=mname, file=d["file"], scale=f"{s} {s} {s}",
                              inertia="shell")
                meshes.add(mname)
                used_files.add(d["file"])
            ET.SubElement(body, "geom", type="mesh", mesh=mname, material="scn_palette", **vis)
        else:
            for typ, size, pos, euler, rgba in d["geoms"]:
                ET.SubElement(body, "geom", type=typ, size=" ".join(f"{v * s:.4f}" for v in size),
                              pos=" ".join(f"{v * s:.4f}" for v in pos), euler=" ".join(str(v) for v in euler),
                              material=_material(asset, matcache, rgba), **vis)
    return sorted(used_files)
