"""Procedural open-wheel Formula-Student-style car + its single pinhole camera.

Contract:
    add_car(worldbody, asset, car_cfg, cam_cfg) -> ET.Element
Adds a mocap body named CAR_BODY (origin = base_link: ground under the car centre, x fwd, y left,
z up) containing the visual-only car geoms and the camera named cam_cfg["name"].

Why procedural: no download licensing/size risk, no file paths to resolve from the generated scene
XML (meshes are written inline as vertex/face lists), dimensions follow car.yaml, and the car
is built from a few lofted meshes (nose/monocoque, sidepods, engine cover) plus primitives
(wings, wheels, wishbones, halo, hoop, helmet).
"""
import math
import xml.etree.ElementTree as ET

import numpy as np

from .camera_geometry import R_base_cam, intrinsics

CAR_BODY = "car"
_VIS = {"contype": "0", "conaffinity": "0"}   # every car geom is visual only


# ------------------------------------------------------------------ camera
def camera_xyaxes(roll_deg, pitch_deg, yaw_deg):
    """MuJoCo camera x and y axes (x right, y up; it looks along -z) expressed in base_link."""
    R = R_base_cam({"roll_deg": roll_deg, "pitch_deg": pitch_deg, "yaw_deg": yaw_deg})
    return R[:, 0], -R[:, 1]          # optical x (right), up = -optical y (down)


def camera_attributes(cam_cfg):
    """XML attributes for a <camera> with exact pinhole intrinsics (MuJoCo >= 3.2).

    MuJoCo: `resolution` (px), `focalpixel` (fx fy px), `principalpixel` = offset of the image centre
    from the principal point in px (verified empirically: sign is opposite to cx - w/2), `sensorsize` (length units; only the ratio to the pixel size
    matters, we use an arbitrary 1e-3 per pixel).
    """
    fx, fy, cx, cy, w, h = intrinsics(cam_cfg)
    pitch = 1e-3
    return {
        "resolution": f"{w} {h}",
        "sensorsize": f"{w * pitch:.6f} {h * pitch:.6f}",
        "focalpixel": f"{fx:.6f} {fy:.6f}",
        "principalpixel": f"{w / 2.0 - cx:.6f} {h / 2.0 - cy:.6f}",
    }


# ------------------------------------------------------------------ mesh helpers
def _ring(x, y0, y1, z0, z1, chamfer):
    """Octagon-ish cross-section of a box y0..y1, z0..z1 with chamfered corners."""
    cy = min(chamfer, (y1 - y0) * 0.45)
    cz = min(chamfer, (z1 - z0) * 0.45)
    return [(x, y0 + cy, z0), (x, y1 - cy, z0), (x, y1, z0 + cz), (x, y1, z1 - cz),
            (x, y1 - cy, z1), (x, y0 + cy, z1), (x, y0, z1 - cz), (x, y0, z0 + cz)]


def loft(sections, chamfer=0.06):
    """Loft rectangular sections (x, y0, y1, z0, z1), listed front-to-back or back-to-front,
    into a closed triangle mesh (verts, faces) with outward winding."""
    rings = [_ring(*s, chamfer) for s in sections]
    n = len(rings[0])
    verts = np.array([p for r in rings for p in r], dtype=float)
    faces = []
    for i in range(len(rings) - 1):
        for j in range(n):
            a, b = i * n + j, i * n + (j + 1) % n
            c, d = (i + 1) * n + j, (i + 1) * n + (j + 1) % n
            faces += [(a, b, d), (a, d, c)]
    for base in (0, (len(rings) - 1) * n):                       # end caps (fans)
        for j in range(1, n - 1):
            faces.append((base, base + j, base + j + 1))
    faces = np.array(faces, dtype=int)
    cen = verts.mean(axis=0)
    for k, (a, b, c) in enumerate(faces):                          # force outward winding
        nrm = np.cross(verts[b] - verts[a], verts[c] - verts[a])
        if np.dot(nrm, verts[[a, b, c]].mean(axis=0) - cen) < 0:
            faces[k] = (a, c, b)
    return verts, faces


def _add_mesh(asset, name, verts, faces):
    ET.SubElement(asset, "mesh", name=name,
                  vertex=" ".join(f"{v:.5f}" for v in verts.ravel()),
                  face=" ".join(str(int(i)) for i in faces.ravel()))


def _fmt(*v):
    return " ".join(f"{float(a):.5f}" for a in v)


def _rgba(c):
    return " ".join(f"{v:.3f}" for v in c)


# ------------------------------------------------------------------ car
def add_car(worldbody, asset, car_cfg, cam_cfg):
    L = car_cfg.get("length", 2.9)
    W = car_cfg.get("width", 1.4)
    WB = car_cfg.get("wheelbase", 1.55)
    TW = car_cfg.get("track_width", 1.2)
    R = car_cfg.get("wheel_radius", 0.23)
    WW = car_cfg.get("wheel_width", 0.20)
    red = [float(v) for v in car_cfg.get("body_color", [0.85, 0.1, 0.1, 1.0])]
    accent = [float(v) for v in car_cfg.get("accent_color", [0.95, 0.95, 0.95, 1.0])]
    helmet_rgba = [float(v) for v in car_cfg.get("helmet_color", [0.95, 0.8, 0.1, 1.0])]
    carbon = [0.07, 0.07, 0.08, 1.0]
    xf, xr = WB / 2, -WB / 2                  # front / rear axle x
    yw = TW / 2                               # tyre centre-line |y|
    front_x, rear_x = L / 2, -L / 2
    s = L / 2.9                               # longitudinal layout scales with car length

    body = ET.SubElement(worldbody, "body", name=CAR_BODY, mocap="true", pos="0 0 0")

    def geom(name, typ, **kw):
        return ET.SubElement(body, "geom", {"name": f"car_{name}", "type": typ, **_VIS, **kw})

    # ---- lofted bodywork: nose cone + monocoque (tapered, rising towards the cockpit) ----
    mono = [(front_x - 0.02, -0.045, 0.045, 0.085, 0.16),
            (front_x - 0.35 * s, -0.09, 0.09, 0.08, 0.20),
            (xf - 0.05, -0.17, 0.17, 0.08, 0.30),
            (0.40 * s, -0.22, 0.22, 0.07, 0.36),
            (0.05 * s, -0.24, 0.24, 0.07, 0.40),
            (-0.30 * s, -0.24, 0.24, 0.09, 0.44),
            (-0.60 * s, -0.20, 0.20, 0.10, 0.47)]
    _add_mesh(asset, "car_monocoque", *loft(mono, 0.07))
    geom("monocoque", "mesh", mesh="car_monocoque", rgba=_rgba(red))

    eng = [(-0.45 * s, -0.17, 0.17, 0.10, 0.50),
           (-0.75 * s, -0.15, 0.15, 0.12, 0.60),
           (-1.05 * s, -0.10, 0.10, 0.14, 0.52),
           (rear_x + 0.20, -0.07, 0.07, 0.16, 0.40)]
    _add_mesh(asset, "car_engine", *loft(eng, 0.06))
    geom("engine", "mesh", mesh="car_engine", rgba=_rgba(red))

    for side, nm in ((1, "L"), (-1, "R")):                          # sidepods
        y0, y1 = sorted((side * 0.26, side * (yw - WW / 2 - 0.03)))
        sp = [(0.38 * s, y0, y1, 0.09, 0.26),
              (0.12 * s, y0, y1, 0.07, 0.38),
              (-0.30 * s, y0, y1, 0.07, 0.38),
              (-0.70 * s, y0 + 0.05, y1 - 0.06, 0.09, 0.30)]
        _add_mesh(asset, f"car_sidepod_{nm}", *loft(sp, 0.07))
        geom(f"sidepod_{nm}", "mesh", mesh=f"car_sidepod_{nm}", rgba=_rgba(red))
        geom(f"inlet_{nm}", "box", size=_fmt(0.015, abs(y1 - y0) / 2 - 0.02, 0.09),
             pos=_fmt(0.37 * s, (y0 + y1) / 2, 0.24), rgba=_rgba(carbon))

    geom("floor", "box", size=_fmt(0.95 * s, 0.34, 0.012), pos=_fmt(-0.05, 0, 0.06), rgba=_rgba(carbon))

    # ---- front wing ----
    fw_half = min(W / 2 + 0.1, 0.85)
    fx = front_x - 0.14
    geom("fw_main", "box", size=_fmt(0.12, fw_half, 0.006), pos=_fmt(fx, 0, 0.07), rgba=_rgba(carbon))
    geom("fw_flap", "box", size=_fmt(0.07, fw_half * 0.95, 0.005), pos=_fmt(fx - 0.04, 0, 0.12),
         euler="0 -11 0", rgba=_rgba(accent))
    for sgn, nm in ((1, "L"), (-1, "R")):
        geom(f"fw_endplate_{nm}", "box", size=_fmt(0.15, 0.006, 0.07),
             pos=_fmt(fx - 0.02, sgn * fw_half, 0.12), rgba=_rgba(red))
    geom("fw_pylon", "box", size=_fmt(0.08, 0.012, 0.03), pos=_fmt(fx + 0.02, 0, 0.10), rgba=_rgba(carbon))

    # ---- rear wing ----
    rw_half = 0.45
    rx = rear_x + 0.20
    rz = 0.78
    geom("rw_main", "box", size=_fmt(0.13, rw_half, 0.008), pos=_fmt(rx, 0, rz), euler="0 -10 0",
         rgba=_rgba(carbon))
    geom("rw_flap", "box", size=_fmt(0.09, rw_half, 0.007), pos=_fmt(rx - 0.09, 0, rz + 0.11),
         euler="0 -26 0", rgba=_rgba(accent))
    for sgn, nm in ((1, "L"), (-1, "R")):
        geom(f"rw_endplate_{nm}", "box", size=_fmt(0.24, 0.007, 0.20),
             pos=_fmt(rx - 0.04, sgn * rw_half, rz + 0.05), rgba=_rgba(red))
        geom(f"rw_strut_{nm}", "capsule", size="0.012",
             fromto=_fmt(rx + 0.02, sgn * 0.06, 0.42, rx + 0.02, sgn * 0.20, rz - 0.02), rgba=_rgba(carbon))
    geom("rw_beam", "box", size=_fmt(0.07, 0.30, 0.005), pos=_fmt(rx + 0.03, 0, 0.32), rgba=_rgba(carbon))

    # ---- wheels (tyre + rim + hub) and double-wishbone suspension; static (mocap body) ----
    for ax, wn in ((xf, "F"), (xr, "R")):
        for sgn, sn in ((1, "L"), (-1, "R")):
            nm, wy = wn + sn, sgn * yw
            kw = dict(euler="90 0 0", pos=_fmt(ax, wy, R))
            geom(f"tyre_{nm}", "cylinder", size=_fmt(R, WW / 2), rgba="0.04 0.04 0.045 1", **kw)
            geom(f"rim_{nm}", "cylinder", size=_fmt(R * 0.62, WW / 2 + 0.004), rgba="0.75 0.75 0.78 1", **kw)
            geom(f"hub_{nm}", "cylinder", size=_fmt(R * 0.22, WW / 2 + 0.01), rgba="0.15 0.15 0.15 1", **kw)
            hub_y = sgn * (yw - WW / 2)
            for zc, leg, dz in ((0.09, "lo", -0.06), (0.30, "up", 0.07)):
                for dx, ln in ((0.14, "a"), (-0.14, "b")):
                    geom(f"arm_{nm}_{leg}{ln}", "capsule", size="0.009",
                         fromto=_fmt(ax + dx, sgn * 0.22, zc, ax, hub_y, R + dz), rgba=_rgba(carbon))

    # ---- driver, halo, main hoop ----
    hx, hz = 0.0, 0.57
    geom("helmet", "sphere", size="0.13", pos=_fmt(hx, 0, hz), rgba=_rgba(helmet_rgba))
    geom("visor", "box", size=_fmt(0.03, 0.09, 0.03), pos=_fmt(hx + 0.10, 0, hz + 0.01), rgba="0.02 0.02 0.05 1")
    geom("shoulders", "box", size=_fmt(0.1, 0.20, 0.07), pos=_fmt(hx - 0.15, 0, 0.45), rgba=_rgba(accent))

    hoop_x = -0.30 * s
    for sgn, nm in ((1, "L"), (-1, "R")):
        geom(f"hoop_leg_{nm}", "capsule", size="0.017",
             fromto=_fmt(hoop_x + 0.04, sgn * 0.20, 0.38, hoop_x, sgn * 0.12, 0.98), rgba=_rgba(carbon))
    geom("hoop_top", "capsule", size="0.017", fromto=_fmt(hoop_x, -0.12, 0.98, hoop_x, 0.12, 0.98),
         rgba=_rgba(carbon))
    hp, ht = (0.30 * s, 0.0, 0.40), (0.24 * s, 0.0, 0.74)          # halo centre pillar
    geom("halo_pillar", "capsule", size="0.014", fromto=_fmt(*hp, *ht), rgba=_rgba(carbon))
    for sgn, nm in ((1, "L"), (-1, "R")):
        geom(f"halo_arm_{nm}", "capsule", size="0.014",
             fromto=_fmt(*ht, hoop_x + 0.06, sgn * 0.22, 0.86), rgba=_rgba(carbon))
        geom(f"halo_arm2_{nm}", "capsule", size="0.014",
             fromto=_fmt(hoop_x + 0.06, sgn * 0.22, 0.86, hoop_x, sgn * 0.12, 0.98), rgba=_rgba(carbon))

    # ---- camera + housing. The housing sits BEHIND the camera plane, so it never shows in the image.
    R_bc = R_base_cam(cam_cfg)
    cam_pos = np.array([cam_cfg["x"], cam_cfg["y"], cam_cfg["z"]], dtype=float)
    if car_cfg.get("camera_housing", True):
        hc = cam_pos - R_bc[:, 2] * 0.055
        geom("cam_housing", "box", size=_fmt(0.045, 0.04, 0.03), pos=_fmt(*hc),
             xyaxes=_fmt(*R_bc[:, 2], *R_bc[:, 0]), rgba="0.1 0.1 0.12 1")
        if cam_pos[2] > 0.55:
            mx = hc[0] - 0.03
            geom("cam_mast", "capsule", size="0.012",
                 fromto=_fmt(mx, cam_pos[1], 0.34, mx, cam_pos[1], hc[2] - 0.03), rgba=_rgba(carbon))

    xa, ya = camera_xyaxes(cam_cfg.get("roll_deg", 0.0), cam_cfg.get("pitch_deg", 0.0),
                           cam_cfg.get("yaw_deg", 0.0))
    ET.SubElement(body, "camera", {"name": cam_cfg["name"], "pos": _fmt(*cam_pos),
                                   "xyaxes": " ".join(f"{v:.6f}" for v in (*xa, *ya)),
                                   **camera_attributes(cam_cfg)})
    return body
