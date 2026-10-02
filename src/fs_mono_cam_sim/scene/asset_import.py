"""One-off converter: Kenney (CC0) OBJ packs -> MuJoCo-friendly per-colour OBJ parts.

MuJoCo's OBJ loader ignores materials, so every source model is split by colour into
one mesh per colour ("part"); the colour is stored as an rgba in assets/scenery/catalog.json
and applied to the geom. Vertices are converted Y-up -> Z-up, centred on x/y, base at z=0
and uniformly scaled to a nominal size in metres.

Usage (after downloading/unzipping the Kenney packs, see assets/scenery/ATTRIBUTION.md):
    PYTHONPATH=src python -m fs_mono_cam_sim.scene.asset_import <dir containing nature/ racing/ survival/>
"""
import json
import sys
from pathlib import Path

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[3]
OUT_DIR = REPO_ROOT / "assets" / "scenery"

# The nature-kit material colours look teal when used raw; replace with natural ones.
COLOR_REMAP = {
    "grass": (0.25, 0.48, 0.15), "leafsGreen": (0.20, 0.44, 0.13), "leafsDark": (0.12, 0.32, 0.11),
    "leafsFall": (0.72, 0.38, 0.10), "dirt": (0.42, 0.30, 0.18), "dirtDark": (0.34, 0.24, 0.15),
    "woodBark": (0.36, 0.25, 0.16), "woodBarkDark": (0.28, 0.19, 0.12), "wood": (0.48, 0.34, 0.21),
    "woodDark": (0.28, 0.19, 0.13), "woodInner": (0.68, 0.54, 0.38), "woodBirch": (0.85, 0.82, 0.75),
    "stone": (0.50, 0.50, 0.49), "stoneDark": (0.38, 0.38, 0.40), "colorRed": (0.7, 0.15, 0.12),
    "colorYellow": (0.85, 0.65, 0.15), "colorWhite": (0.9, 0.9, 0.9), "colorTan": (0.7, 0.55, 0.38),
}

# (pack, file stem, output name, category, dim, size_m, align)
#   dim "h": scale so height == size_m;  "w": scale so max horizontal extent == size_m
#   align: yaw follows the track direction (barriers/fences)
CATALOG = [
    ("nature", "tree_default", "tree_default", "tree", "h", 8.0, False),
    ("nature", "tree_oak", "tree_oak", "tree", "h", 9.0, False),
    ("nature", "tree_tall", "tree_tall", "tree", "h", 11.0, False),
    ("nature", "tree_detailed", "tree_detailed", "tree", "h", 8.0, False),
    ("nature", "tree_fat", "tree_fat", "tree", "h", 7.0, False),
    ("nature", "tree_simple", "tree_simple", "tree", "h", 6.5, False),
    ("nature", "tree_thin", "tree_thin", "tree", "h", 9.0, False),
    ("nature", "tree_cone", "tree_cone", "tree", "h", 8.0, False),
    ("nature", "tree_pineTallA", "pine_tall_a", "tree", "h", 12.0, False),
    ("nature", "tree_pineTallB", "pine_tall_b", "tree", "h", 11.0, False),
    ("nature", "tree_pineDefaultA", "pine_default_a", "tree", "h", 9.0, False),
    ("nature", "tree_pineRoundC", "pine_round_c", "tree", "h", 8.0, False),
    ("nature", "tree_default_fall", "tree_default_fall", "tree", "h", 8.0, False),
    ("nature", "plant_bushLarge", "bush_large", "bush", "h", 1.3, False),
    ("nature", "plant_bush", "bush", "bush", "h", 1.0, False),
    ("nature", "plant_bushDetailed", "bush_detailed", "bush", "h", 1.2, False),
    ("survival", "rock-a", "rock_a", "rock", "w", 1.4, False),
    ("survival", "rock-b", "rock_b", "rock", "w", 1.1, False),
    ("survival", "rock-c", "rock_c", "rock", "w", 1.7, False),
    ("nature", "stump_round", "stump_round", "object", "h", 0.5, False),
    ("nature", "log_stack", "log_stack", "object", "h", 0.9, False),
    ("racing", "barrierRed", "barrier_red", "object", "w", 2.0, True),
    ("racing", "barrierWhite", "barrier_white", "object", "w", 2.0, True),
    ("racing", "barrierWall", "barrier_wall", "object", "w", 3.0, True),
    ("racing", "fenceStraight", "fence_straight", "object", "w", 3.0, True),
    ("racing", "billboard", "billboard", "structure", "h", 5.0, False),
    ("racing", "lightPostLarge", "light_post", "structure", "h", 7.0, False),
    ("racing", "flagRed", "flag_red", "structure", "h", 4.0, False),
    ("racing", "grandStand", "grandstand", "structure", "w", 12.0, True),
    ("survival", "barrel", "barrel", "object", "h", 0.9, False),
    ("survival", "tent", "tent", "structure", "h", 2.6, False),
    ("survival", "signpost", "signpost", "object", "h", 1.8, False),
    ("survival", "bucket", "bucket", "object", "h", 0.4, False),
]

PACK_LICENSE = {
    "nature": ("Kenney Nature Kit 2.1", "https://kenney.nl/assets/nature-kit"),
    "racing": ("Kenney Racing Kit", "https://kenney.nl/assets/racing-kit"),
    "survival": ("Kenney Survival Kit", "https://kenney.nl/assets/survival-kit"),
}


def _parse_mtl(path):
    mats, cur = {}, None
    if not path.exists():
        return mats
    for line in path.read_text(errors="ignore").splitlines():
        t = line.split()
        if not t:
            continue
        if t[0] == "newmtl":
            cur = t[1]
            mats[cur] = {"kd": (0.8, 0.8, 0.8), "map": None}
        elif cur and t[0] == "Kd":
            mats[cur]["kd"] = tuple(float(v) for v in t[1:4])
        elif cur and t[0] == "map_Kd":
            mats[cur]["map"] = t[-1]
    return mats


def _load_obj(path):
    """Return verts (n,3), list of (material, tri vertex idx, uv-centroid or None), mtl dict."""
    obj_dir = path.parent
    verts, uvs, faces, mats, mat = [], [], [], {}, None
    for line in path.read_text(errors="ignore").splitlines():
        t = line.split()
        if not t:
            continue
        if t[0] == "mtllib":
            mats.update(_parse_mtl(obj_dir / t[1]))
        elif t[0] == "usemtl":
            mat = t[1]
        elif t[0] == "v":
            verts.append([float(v) for v in t[1:4]])
        elif t[0] == "vt":
            uvs.append([float(v) for v in t[1:3]])
        elif t[0] == "f":
            idx = []
            for tok in t[1:]:
                p = tok.split("/")
                vi = int(p[0])
                vi = vi - 1 if vi > 0 else len(verts) + vi
                ti = None
                if len(p) > 1 and p[1]:
                    ti = int(p[1])
                    ti = ti - 1 if ti > 0 else len(uvs) + ti
                idx.append((vi, ti))
            for k in range(1, len(idx) - 1):          # fan triangulation
                tri = [idx[0], idx[k], idx[k + 1]]
                uv = None
                if all(i[1] is not None for i in tri):
                    uv = np.mean([uvs[i[1]] for i in tri], axis=0)
                faces.append((mat, [i[0] for i in tri], uv))
    return np.array(verts, float), faces, mats, obj_dir


def _face_color(mat, uv, mats, obj_dir, cache):
    m = mats.get(mat, {"kd": (0.7, 0.7, 0.7), "map": None})
    if m["map"] and uv is not None:
        import cv2
        p = obj_dir / m["map"]
        if p not in cache:
            cache[p] = cv2.imread(str(p))
        img = cache[p]
        if img is not None:
            h, w = img.shape[:2]
            x = int(np.clip(uv[0] * w, 0, w - 1))
            y = int(np.clip((1 - uv[1]) * h, 0, h - 1))
            b, g, r = img[y, x]
            return (r / 255.0, g / 255.0, b / 255.0)
    return COLOR_REMAP.get(mat, m["kd"])


def convert(src_root, out_dir=OUT_DIR):
    """Convert all CATALOG models. Every model becomes ONE mesh (one geom per instance): colours are
    encoded as per-vertex texcoords into a shared palette texture (palette.png, one swatch per colour)."""
    src_root, out_dir = Path(src_root), Path(out_dir)
    (out_dir / "meshes").mkdir(parents=True, exist_ok=True)
    for old in (out_dir / "meshes").glob("*.obj"):
        old.unlink()
    models, cache = {}, {}
    for pack, stem, name, cat, dim, size, align in CATALOG:
        path = next(iter((src_root / pack / "Models").glob(f"OBJ*/{stem}.obj")), None)
        if path is None:
            print("missing", pack, stem)
            continue
        v, faces, mats, obj_dir = _load_obj(path)
        v = np.stack([v[:, 0], -v[:, 2], v[:, 1]], axis=1)             # Y-up -> Z-up (rotation)
        lo, hi = v.min(0), v.max(0)
        ext = hi - lo
        s = size / (ext[2] if dim == "h" else max(ext[0], ext[1]))
        v = (v - np.array([(lo[0] + hi[0]) / 2, (lo[1] + hi[1]) / 2, lo[2]])) * s
        tris = []                                                      # (vertex ids, colour)
        for mat, tri, uv in faces:
            a = np.linalg.norm(np.cross(v[tri[1]] - v[tri[0]], v[tri[2]] - v[tri[0]])) / 2
            if a <= 1e-8:                                              # degenerate triangle
                continue
            col = tuple(round(min(1.0, round(float(c) / 0.06) * 0.06), 3)
                        for c in _face_color(mat, uv, mats, obj_dir, cache))
            tris.append((tri, col))
        models[name] = dict(v=v, tris=tris, cat=cat, align=align, pack=pack)
    palette = sorted({c for m in models.values() for _, c in m["tris"]})
    index = {c: k for k, c in enumerate(palette)}
    import cv2
    sw = 8
    img = np.zeros((sw, len(palette) * sw, 3), np.uint8)
    for k, c in enumerate(palette):
        img[:, k * sw:(k + 1) * sw] = (np.array(c[::-1]) * 255).round().astype(np.uint8)
    cv2.imwrite(str(out_dir / "palette.png"), img)
    catalog = {"_palette": {"file": "palette.png", "count": len(palette)}}
    for name, m in models.items():
        v, tris = m["v"], m["tris"]
        used = sorted({i for t, _ in tris for i in t})
        # flat shading (Kenney models are faceted): every triangle gets its own 3 vertices
        with open(out_dir / "meshes" / f"{name}.obj", "w") as f:
            f.write(f"# {PACK_LICENSE[m['pack']][0]} (CC0), converted; colours via palette.png texcoords\n")
            for t, c in tris:
                for i in t:
                    f.write("v %.4f %.4f %.4f\n" % tuple(v[i]))
            for t, c in tris:
                for _ in t:
                    f.write("vt %.5f 0.5\n" % ((index[c] + 0.5) / len(palette)))
            for k in range(len(tris)):
                f.write("f " + " ".join(f"{3 * k + j + 1}/{3 * k + j + 1}" for j in range(3)) + "\n")
        ext = v[used].max(0) - v[used].min(0)
        catalog[name] = {"category": m["cat"], "align": m["align"], "file": f"{name}.obj",
                         "height": round(float(ext[2]), 3),
                         "radius": round(float(np.hypot(ext[0], ext[1]) / 2), 3),
                         "source": PACK_LICENSE[m["pack"]][0]}
        print(f"{name:20s} tris={len(tris):5d} h={ext[2]:.2f} r={catalog[name]['radius']:.2f}")
    (out_dir / "catalog.json").write_text(json.dumps(catalog, indent=1))
    print("palette colours:", len(palette))
    return catalog


if __name__ == "__main__":
    convert(sys.argv[1])
