"""MuJoCo scene generator: ground, sky, lighting, FS cones, roadside scenery and the car.

Contract (see CLAUDE.md):
    build_scene_xml(track, cfg) -> str          cfg = config.load_all()
    Every cone is a body `cone_<class>_<index>` holding all its geoms; visual-only geoms use
    contype=0/conaffinity=0; <statistic extent="1"> so znear/zfar are in metres.
    The car is added through car.car_model.add_car.

`build_scene_xml` returns a string whose meshdir/texturedir are absolute (works with
MjModel.from_xml_string). `write_scene(track, cfg, path)` writes a self-contained folder
(XML + meshes/ + textures/ next to it, relative paths) so the XML loads from its own location.
"""
import shutil
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
from scipy.spatial import cKDTree

from ..car.car_model import add_car
from . import cones as cone_mod
from . import scenery as scn
from .textures import make_texture

REPO_ROOT = Path(__file__).resolve().parents[3]
DEFAULT_TEX_DIR = REPO_ROOT / "generated" / "scene_assets" / "textures"


def cone_body_name(cls, idx):
    return f"cone_{cls}_{idx}"


def _fmt(v):
    return " ".join(f"{float(x):.5g}" for x in v)


# ------------------------------------------------------------------ ground ---------------------
def _add_ground_texture(asset, name, kind, rgb1, rgb2, tex_dir, seed):
    """Add texture `name`; returns the PNG file name (None for the builtin checker)."""
    if kind == "checker":
        ET.SubElement(asset, "texture", name=name, type="2d", builtin="checker",
                      rgb1=_fmt(rgb1), rgb2=_fmt(rgb2), width="512", height="512")
        return None
    fname = f"{kind}_{seed}.png"
    path = Path(tex_dir) / fname
    if not path.exists():
        make_texture(kind, path, seed=seed)
    ET.SubElement(asset, "texture", name=name, type="2d", file=fname)
    return fname


def _strip_halfwidths(track, centers):
    """Left/right half widths of the racing surface at `centers` from the nearest blue/yellow cones."""
    out = []
    for cls in ("blue", "yellow"):
        pts = track.cones[cls]
        out.append(cKDTree(pts).query(centers)[0] if len(pts) else np.full(len(centers), np.nan))
    L, R = out
    both = np.concatenate([L, R])
    fallback = np.nanmedian(both) if not np.isnan(both).all() else 2.0
    L = np.where(np.isnan(L), fallback, L)
    R = np.where(np.isnan(R), fallback, R)
    k = 25
    ker = np.ones(k) / k

    def sm(a):
        return np.convolve(np.concatenate([a[-k:], a, a[:k]]), ker, mode="same")[k:-k]
    return sm(L), sm(R)


def _add_track_strip(asset, wb, track, gcfg, tex_dir, tile):
    """Closed slab mesh following the centerline, textured with a separate asphalt texture."""
    sc = gcfg["track_strip"]
    c = scn._dense_centerline(track.centerline, 1.0)
    n = len(c)
    tang = np.roll(c, -1, axis=0) - np.roll(c, 1, axis=0)
    tang /= np.linalg.norm(tang, axis=1, keepdims=True) + 1e-9
    nrm = np.stack([-tang[:, 1], tang[:, 0]], axis=1)
    hl, hr = _strip_halfwidths(track, c)
    hl, hr = hl + sc["extra_width_m"], hr + sc["extra_width_m"]
    h = float(sc["height_m"])
    Lp, Rp = c + nrm * hl[:, None], c - nrm * hr[:, None]
    s = np.concatenate([[0], np.cumsum(np.linalg.norm(np.diff(np.vstack([c, c[:1]]), axis=0), axis=1))])
    V, UV, F, FI = [], [], [], []

    def addv(p, z, uv):
        V.append((p[0], p[1], z))
        UV.append(uv)
        return len(V) - 1

    top = []                                   # N+1 pairs so the texture does not wrap across one quad
    for i in range(n + 1):
        k = i % n
        top.append((addv(Lp[k], 0.0, (0.0, s[i] / tile)),
                    addv(Rp[k], 0.0, ((hl[k] + hr[k]) / tile, s[i] / tile))))
    for i in range(n):
        (l0, r0), (l1, r1) = top[i], top[i + 1]
        F += [(r0, r1, l1), (r0, l1, l0)]
        FI += [i, i]

    def ring(P, z):
        return [addv(P[i], z, (0, 0)) for i in range(n)]
    lt, lb, rt, rb = ring(Lp, 0.0), ring(Lp, -1.5 * h), ring(Rp, 0.0), ring(Rp, -1.5 * h)
    for i in range(n):
        j = (i + 1) % n
        F += [(lb[i], lb[j], lt[j]), (lb[i], lt[j], lt[i]),
              (rb[j], rb[i], rt[i]), (rb[j], rt[i], rt[j]),
              (lb[i], rb[i], rb[j]), (lb[i], rb[j], lb[j])]
        FI += [i] * 6
    V, UV = np.array(V), np.array(UV)
    mids = np.concatenate([(Lp + Rp) / 2, np.full((n, 1), -0.75 * h)], axis=1)
    fixed = []
    for f, i in zip(F, FI):                    # make every face point away from the slab's core
        a, b, cc = V[list(f)]
        nv = np.cross(b - a, cc - a)
        cen = (a + b + cc) / 3
        fixed.append(f if np.dot(nv, cen - mids[i]) >= 0 else (f[0], f[2], f[1]))
    ET.SubElement(asset, "mesh", name="track_strip", vertex=" ".join(f"{x:.4f}" for x in V.ravel()),
                  face=" ".join(str(i) for f in fixed for i in f),
                  texcoord=" ".join(f"{x:.4f}" for x in UV.ravel()))
    tex = _add_ground_texture(asset, "strip_tex", sc.get("texture", "asphalt"),
                              gcfg["rgb1"], gcfg["rgb2"], tex_dir, seed=2)
    ET.SubElement(asset, "material", name="strip_mat", texture="strip_tex", texrepeat="1 1",
                  rgba="0.85 0.85 0.85 1", specular="0.1", shininess="0.2")
    ET.SubElement(wb, "geom", name="track_strip", type="mesh", mesh="track_strip", material="strip_mat",
                  contype="0", conaffinity="0")
    return tex


# ------------------------------------------------------------------ main -----------------------
def _shadow_clip(lcfg, track):
    """Half-size (m) of the directional-light shadow map. With statistic extent=1 the default (1 m) would
    leave the whole scene outside the map; 'auto' covers the track plus the near scenery band."""
    clip = lcfg.get("shadow_clip", "auto")
    if clip == "auto":
        pts = np.vstack([track.centerline] + [track.cones[c] for c in track.cones if len(track.cones[c])])
        ctr = (pts.min(0) + pts.max(0)) / 2
        clip = float(np.linalg.norm(pts - ctr, axis=1).max()) + 25.0
    return str(round(float(clip), 1))


def build_scene_xml(track, cfg, out_dir=None, _info=None):
    """Return the MJCF string. With `out_dir`, meshdir/texturedir are relative ("meshes", "textures")."""
    scfg, cam = cfg["scene"], cfg["camera"]
    gcfg, lcfg = scfg["ground"], scfg["lighting"]
    catalog, mesh_src, pal_png = scn.load_catalog(scfg["scenery"])
    tex_dir = Path(out_dir) / "textures" if out_dir else DEFAULT_TEX_DIR

    root = ET.Element("mujoco", model="fs_track")
    comp = ET.SubElement(root, "compiler", angle="degree")
    if out_dir:
        comp.set("meshdir", "meshes")
        comp.set("texturedir", "textures")
    else:
        comp.set("texturedir", tex_dir.as_posix())
        if mesh_src:
            comp.set("meshdir", Path(mesh_src).as_posix())
    ET.SubElement(root, "option", timestep="0.01", gravity="0 0 -9.81")
    pts_all = np.vstack([track.centerline] + [track.cones[c] for c in track.cones if len(track.cones[c])])
    c0 = (pts_all.min(0) + pts_all.max(0)) / 2     # shadow map is centred here (statistic center)
    ET.SubElement(root, "statistic", extent="1", center=f"{c0[0]:.2f} {c0[1]:.2f} 0")

    visual = ET.SubElement(root, "visual")     # znear/zfar are metres because extent = 1
    ET.SubElement(visual, "global", offwidth=str(max(int(cam["width"]), 640)),
                  offheight=str(max(int(cam["height"]), 480)))
    ET.SubElement(visual, "quality", shadowsize=str(int(lcfg.get("shadow_size", 4096))),
                  offsamples=str(int(lcfg.get("multisample", 4))))
    ET.SubElement(visual, "headlight", ambient=_fmt(lcfg["ambient"]), diffuse="0.15 0.15 0.15",
                  specular="0 0 0")
    mp = dict(znear=str(cam["near_clip"]), zfar=str(cam["far_clip"]),
              shadowclip=_shadow_clip(lcfg, track))
    hz = scfg.get("haze", {})
    if hz.get("enabled"):
        mp.update(fogstart=str(hz["start_m"]), fogend=str(hz["end_m"]))
        ET.SubElement(visual, "rgba", fog=_fmt([*hz["rgb"], 1.0]), haze=_fmt([*hz["rgb"], 1.0]))
    ET.SubElement(visual, "map", **mp)

    asset = ET.SubElement(root, "asset")
    wb = ET.SubElement(root, "worldbody")

    sky = scfg["sky"]
    ET.SubElement(asset, "texture", name="sky", type="skybox", builtin="gradient",
                  rgb1=_fmt(sky["rgb1"]), rgb2=_fmt(sky["rgb2"]), width="512", height="3072")

    # ground plane sized from track bounds + margin
    pts = np.vstack([track.centerline] + [track.cones[c] for c in track.cones if len(track.cones[c])])
    lo, hi = pts.min(0), pts.max(0)
    ctr, half = (lo + hi) / 2, (hi - lo) / 2 + float(gcfg["margin_m"])
    tile = float(gcfg.get("tile_m", 6.0))
    tex = _add_ground_texture(asset, "ground_tex", gcfg["texture"], gcfg["rgb1"], gcfg["rgb2"], tex_dir, seed=1)
    ET.SubElement(asset, "material", name="ground_mat", texture="ground_tex", texuniform="true",
                  texrepeat=f"{1.0 / tile:.5f} {1.0 / tile:.5f}", specular="0.05", shininess="0.1")
    strip_on = gcfg.get("track_strip", {}).get("enabled", False)
    gz = -float(gcfg["track_strip"]["height_m"]) if strip_on else 0.0   # ground sits below the strip top (z=0)
    ET.SubElement(wb, "geom", name="ground", type="plane", pos=f"{ctr[0]:.3f} {ctr[1]:.3f} {gz:.3f}",
                  size=f"{half[0]:.2f} {half[1]:.2f} 0.1", material="ground_mat",
                  contype="0", conaffinity="0")
    textures_used = {tex} if tex else set()
    if strip_on:
        t2 = _add_track_strip(asset, wb, track, gcfg, tex_dir, tile)
        if t2:
            textures_used.add(t2)

    ET.SubElement(wb, "light", name="sun", directional="true", pos=f"{ctr[0]:.1f} {ctr[1]:.1f} 60",
                  dir=_fmt(lcfg["sun_direction"]), diffuse=_fmt(lcfg["sun_diffuse"]),
                  specular=_fmt(lcfg.get("sun_specular", [0.1] * 3)),
                  castshadow="true" if lcfg.get("shadows", True) else "false")

    spec = cone_mod.add_cone_assets(asset, scfg, tex_dir)
    textures_used.add("cone_palette.png")
    for cls, i, x, y in track.all_cones():
        cone_mod.add_cone_body(wb, spec, cls, i, x, y, name=cone_body_name(cls, i))

    placed = scn.place_scenery(track, scfg["scenery"], catalog)
    if catalog and placed:
        Path(tex_dir).mkdir(parents=True, exist_ok=True)
        shutil.copy2(pal_png, Path(tex_dir) / "scenery_palette.png")
        ET.SubElement(asset, "texture", name="scenery_palette", type="2d", file="scenery_palette.png")
        textures_used.add("scenery_palette.png")
    used_meshes = scn.emit_scenery(asset, wb, placed, catalog, mesh_src)
    if _info is not None:
        _info.update(meshes=used_meshes, textures=sorted(textures_used), mesh_src=mesh_src, placed=placed)

    add_car(wb, asset, cfg["car"], cam)        # mocap body "car" with the camera
    ET.indent(root)
    return ET.tostring(root, encoding="unicode")


def write_scene(track, cfg, out_path):
    """Write the scene XML plus meshes/ and textures/ next to it. Returns the XML path."""
    out_path = Path(out_path)
    out_dir = out_path.parent
    out_dir.mkdir(parents=True, exist_ok=True)
    info = {}
    xml = build_scene_xml(track, cfg, out_dir=out_dir, _info=info)
    if info["meshes"]:
        (out_dir / "meshes").mkdir(exist_ok=True)
        for f in info["meshes"]:
            shutil.copy2(Path(info["mesh_src"]) / f, out_dir / "meshes" / f)
    out_path.write_text(xml, encoding="utf-8")
    return out_path
