cmake_minimum_required(VERSION 3.0.2)
project(fs_mono_cam_sim)

find_package(catkin REQUIRED)
catkin_python_setup()
catkin_package()

catkin_install_python(PROGRAMS
  scripts/sim_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
