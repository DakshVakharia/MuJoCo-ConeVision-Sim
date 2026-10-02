"""STUB - to be replaced by the scene agent.

Contract:
    build_scene_xml(track: Track, cfg: dict) -> str
        cfg = config.load_all() (keys: track, scene, car, camera, perception)
    Every cone is a body named cone_<class>_<index> (class in CONE_CLASSES);
    all of a cone's geoms are children of that body (bbox code groups by body).
    The car comes from car.car_model.add_car (mocap body "car" with the camera).
"""
import xml.etree.ElementTree as ET

from ..car.car_model import add_car


def cone_body_name(cls, idx):
    return f"cone_{cls}_{idx}"


CONE_RGBA = {"blue": "0.05 0.2 0.9 1", "yellow": "1 0.85 0 1",
             "orange": "1 0.4 0 1", "orange_big": "1 0.4 0 1"}


def build_scene_xml(track, cfg):
    cam = cfg["camera"]
    root = ET.Element("mujoco", model="fs_track")
    ET.SubElement(root, "option", timestep="0.01", gravity="0 0 -9.81")
    # znear/zfar are relative to statistic extent, so fix extent=1 to make them metres
    ET.SubElement(root, "statistic", extent="1", center="0 0 0")
    visual = ET.SubElement(root, "visual")
    ET.SubElement(visual, "global", offwidth=str(cam["width"]), offheight=str(cam["height"]))
    ET.SubElement(visual, "map", znear=str(cam["near_clip"]), zfar=str(cam["far_clip"]))
    asset = ET.SubElement(root, "asset")
    ET.SubElement(asset, "texture", name="grid", type="2d", builtin="checker",
                  rgb1=".3 .3 .32", rgb2=".27 .27 .29", width="512", height="512")
    ET.SubElement(asset, "material", name="grid", texture="grid", texrepeat="40 40")
    wb = ET.SubElement(root, "worldbody")
    ET.SubElement(wb, "light", directional="true", pos="0 0 50", dir="-0.4 0.3 -1")
    ET.SubElement(wb, "geom", name="ground", type="plane", size="200 200 0.1", material="grid")
    for cls, i, x, y in track.all_cones():
        h = 0.505 if cls == "orange_big" else 0.325
        b = ET.SubElement(wb, "body", name=cone_body_name(cls, i), pos=f"{x} {y} 0")
        ET.SubElement(b, "geom", type="cylinder", size=f"0.1 {h/2}", pos=f"0 0 {h/2}",
                      rgba=CONE_RGBA[cls], contype="0", conaffinity="0")
    add_car(wb, asset, cfg["car"], cam)
    return ET.tostring(root, encoding="unicode")
