"""Loading of the YAML files in config/."""
from pathlib import Path

import yaml

REPO_ROOT = Path(__file__).resolve().parents[2]
CONFIG_DIR = REPO_ROOT / "config"
GENERATED_DIR = REPO_ROOT / "generated"


def load_yaml(path):
    with open(path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f) or {}


def load_config(name, config_dir=CONFIG_DIR):
    """Return the top-level section of config/<name>.yaml.

    load_config("camera") -> the dict under `camera:` in config/camera.yaml.
    perception.yaml has two sections, so load_config("perception") returns the whole file.
    """
    data = load_yaml(Path(config_dir) / f"{name}.yaml")
    if name in data and len(data) == 1:
        return data[name]
    return data


def load_all(config_dir=CONFIG_DIR):
    return {name: load_config(name, config_dir)
            for name in ("track", "scene", "car", "camera", "perception")}
