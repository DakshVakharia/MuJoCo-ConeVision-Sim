cmake_minimum_required(VERSION 3.10)
project(fs_mono_cam_sim)

# ---------------------------------------------------------------------------------------------
# catkin package: Python offline tooling (sim_node.py, export_bundle.py) + C++ runtime node
# `fs_mono_cam_sim_node` (cpp/ros/sim_node.cpp) built on the ROS-independent core in cpp/.
#
# MuJoCo (3.14.0) is NOT a rosdep dependency. Point the build at an extracted release:
#     export MUJOCO_DIR=$HOME/mujoco-3.14.0        # or: catkin_make -DMUJOCO_DIR=$HOME/mujoco-3.14.0
# See docs/ros_cpp_node.md.
# ---------------------------------------------------------------------------------------------
find_package(catkin REQUIRED COMPONENTS
  roscpp roslib image_transport sensor_msgs nav_msgs rosgraph_msgs
  visualization_msgs geometry_msgs std_msgs foxglove_msgs)
find_package(Threads REQUIRED)

catkin_python_setup()
catkin_package()

if(NOT MUJOCO_DIR AND DEFINED ENV{MUJOCO_DIR})
  set(MUJOCO_DIR "$ENV{MUJOCO_DIR}" CACHE PATH "MuJoCo release directory (contains include/ and lib/)")
endif()

# Core library only; the offline/test tools in cpp/ are not needed inside the catkin workspace.
set(FSSIM_BUILD_TOOLS OFF CACHE BOOL "" FORCE)
add_subdirectory(cpp)     # defines target fs_sim_core

add_executable(fs_mono_cam_sim_node cpp/ros/sim_node.cpp)
target_include_directories(fs_mono_cam_sim_node SYSTEM PRIVATE ${catkin_INCLUDE_DIRS})
target_link_libraries(fs_mono_cam_sim_node PRIVATE fs_sim_core ${catkin_LIBRARIES} Threads::Threads)
set_target_properties(fs_mono_cam_sim_node PROPERTIES CXX_STANDARD 17 CXX_STANDARD_REQUIRED ON)
if(MUJOCO_DIR)
  # libmujoco.so.3.14.0 is found at run time without LD_LIBRARY_PATH
  set_target_properties(fs_mono_cam_sim_node PROPERTIES
    BUILD_RPATH "${MUJOCO_DIR}/lib" INSTALL_RPATH "${MUJOCO_DIR}/lib")
endif()
add_dependencies(fs_mono_cam_sim_node ${catkin_EXPORTED_TARGETS})

catkin_install_python(PROGRAMS
  scripts/sim_node.py
  scripts/export_bundle.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS fs_mono_cam_sim_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
