# catkin_python_setup() uses this so `import conevision_sim` works in ROS.
# Outside ROS: `pip install -e .` from the repo root.
from setuptools import setup, find_packages

setup(
    name="conevision_sim",
    version="0.1.0",
    package_dir={"": "src"},
    packages=find_packages("src"),
)
