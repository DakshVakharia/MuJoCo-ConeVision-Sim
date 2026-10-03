"""Fake-YOLO bounding boxes computed exactly from a segmentation render."""
from dataclasses import dataclass

import mujoco
import numpy as np

from ..track.track_types import CONE_CLASSES


@dataclass
class Detection:
    cls: str
    index: int
    body: str
    x_min: float
    y_min: float
    x_max: float
    y_max: float
    visible_pixels: int
    range_m: float


_CACHE = {}


def _tables(model):
    """Cached per-model lookup tables.

    cls_idx[body]  : index into CONE_CLASSES, -1 if not a cone body
    cone_idx[body] : cone index parsed from `cone_<class>_<index>`
    geom_to_cone   : (ngeom+1,) geom id -> cone body id or -1; last slot (index -1) is always -1
    """
    hit = _CACHE.get("m")
    if hit is not None and hit[0] is model:
        return hit[1]
    nb = model.nbody
    cls_idx = np.full(nb, -1, np.int32)
    cone_idx = np.full(nb, -1, np.int32)
    for b in range(nb):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, b) or ""
        if not name.startswith("cone_"):
            continue
        c, _, i = name[5:].rpartition("_")
        if c in CONE_CLASSES and i.isdigit():
            cls_idx[b] = CONE_CLASSES.index(c)
            cone_idx[b] = int(i)
    gb = np.asarray(model.geom_bodyid, np.int32)
    geom_to_cone = np.append(np.where(cls_idx[gb] >= 0, gb, -1), -1).astype(np.int32)
    tabs = (cls_idx, cone_idx, geom_to_cone, np.flatnonzero(cls_idx >= 0))
    _CACHE["m"] = (model, tabs)
    return tabs


def pixel_stats(body_img, nbody):
    """Per-body pixel count and inclusive extent from an HxW body-id image (-1 = none).

    Returns count, xmin, ymin, xmax, ymax arrays of length nbody (extents only valid where count>0).
    """
    sel = body_img >= 0
    ys, xs = np.nonzero(sel)
    ids = body_img[ys, xs]
    count = np.bincount(ids, minlength=nbody)
    xmin = np.zeros(nbody, np.int64)
    xmax = xmin.copy()
    ymin = xmin.copy()
    ymax = xmin.copy()
    if ids.size:
        order = np.argsort(ids, kind="stable")
        ids_s = ids[order]
        starts = np.flatnonzero(np.r_[True, ids_s[1:] != ids_s[:-1]])
        u = ids_s[starts]
        xs_s = xs[order]
        ys_s = ys[order]
        xmin[u] = np.minimum.reduceat(xs_s, starts)
        xmax[u] = np.maximum.reduceat(xs_s, starts)
        ymin[u] = np.minimum.reduceat(ys_s, starts)
        ymax[u] = np.maximum.reduceat(ys_s, starts)
    return count, xmin, ymin, xmax, ymax


def compute_bboxes(model, data, seg, cam_cfg, bbox_cfg, rng):
    """Detections for every visible cone in a geom-id segmentation image `seg` (HxW, -1 = none).

    Boxes are the extent of visible pixels (occlusion-aware, clipped at the image border),
    then filtered/noised per perception.yaml `bbox`. Sorted nearest first.
    """
    cls_idx, cone_idx, geom_to_cone, cone_bodies = _tables(model)
    if cone_bodies.size == 0:
        return []
    h, w = seg.shape
    body_img = geom_to_cone[seg]          # seg == -1 hits the trailing -1 slot
    count, xmin, ymin, xmax, ymax = pixel_stats(body_img, model.nbody)

    min_px = max(int(bbox_cfg.get("min_visible_pixels", 1)), 1)
    min_h = float(bbox_cfg.get("min_box_height_px", 0))
    max_r = float(bbox_cfg.get("max_range_m", np.inf))
    jc = float(bbox_cfg.get("center_jitter_px", 0.0))
    js = float(bbox_cfg.get("size_jitter_frac", 0.0))
    drop = float(bbox_cfg.get("dropout_prob", 0.0))

    cam_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, cam_cfg["name"])
    cam_pos = data.cam_xpos[cam_id]

    out = []
    for b in np.flatnonzero(count >= min_px):
        x0, x1 = float(xmin[b]), float(xmax[b] + 1)      # inclusive pixel extent -> edges
        y0, y1 = float(ymin[b]), float(ymax[b] + 1)
        if y1 - y0 < min_h:
            continue
        r = float(np.linalg.norm(data.xpos[b] - cam_pos))   # camera -> cone base (body origin)
        if r > max_r:
            continue
        if drop > 0 and rng.random() < drop:
            continue
        if jc > 0 or js > 0:
            cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
            bw, bh = x1 - x0, y1 - y0
            if jc > 0:
                cx += rng.normal(0, jc)
                cy += rng.normal(0, jc)
            if js > 0:
                bw *= max(0.1, 1 + rng.normal(0, js))
                bh *= max(0.1, 1 + rng.normal(0, js))
            x0, x1 = max(0.0, cx - bw / 2), min(float(w), cx + bw / 2)
            y0, y1 = max(0.0, cy - bh / 2), min(float(h), cy + bh / 2)
            if x1 <= x0 or y1 <= y0:
                continue
        cname = CONE_CLASSES[cls_idx[b]]
        out.append(Detection(cls=cname, index=int(cone_idx[b]), body=f"cone_{cname}_{int(cone_idx[b])}",
                             x_min=x0, y_min=y0, x_max=x1, y_max=y1,
                             visible_pixels=int(count[b]), range_m=r))
    out.sort(key=lambda d: d.range_m)
    return out
