# catkin_python_setup() uses this so `import fs_mono_cam_sim` works in ROS.
# Outside ROS: `pip install -e .` from the repo root.
from setuptools import setup, find_packages

setup(
    name="fs_mono_cam_sim",
    version="0.1.0",
    package_dir={"": "src"},
    packages=find_packages("src"),
)
