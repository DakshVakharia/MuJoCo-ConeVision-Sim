# Standalone CMake project for the C++ camera-sim runtime. Also usable via add_subdirectory(cpp)
# from a catkin root CMakeLists (nothing here assumes we are the top-level project).
#
#   cmake -S cpp -B build/cpp -DMUJOCO_DIR=/path/to/mujoco-3.x
#   cmake --build build/cpp --config Release
#
# Targets: fs_sim_core (static lib, alias fs_mono_cam_sim::core), fsim_bench, fsim_test_golden.
cmake_minimum_required(VERSION 3.16)
project(fs_mono_cam_sim_cpp LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)   # so the lib can be linked into ROS node executables/shared libs

if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
  set(_fssim_top ON)
  if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "" FORCE)
  endif()
else()
  set(_fssim_top OFF)
endif()
option(FSSIM_BUILD_TOOLS "Build fsim_bench and fsim_test_golden" ${_fssim_top})

# ---------------------------------------------------------------- MuJoCo
# Expects <MUJOCO_DIR>/include/mujoco/mujoco.h and lib/mujoco.lib (Windows) or lib/libmujoco.so*.
set(MUJOCO_DIR "$ENV{MUJOCO_DIR}" CACHE PATH "MuJoCo release directory")
if(NOT TARGET mujoco::mujoco)
  find_path(MUJOCO_INCLUDE_DIR mujoco/mujoco.h
            HINTS "${MUJOCO_DIR}/include" "${CMAKE_CURRENT_SOURCE_DIR}/../.deps/mujoco-3.14.0/include"
                  /opt/mujoco/include /usr/local/include)
  if(WIN32)
    find_library(MUJOCO_LIBRARY NAMES mujoco
                 HINTS "${MUJOCO_DIR}/lib" "${CMAKE_CURRENT_SOURCE_DIR}/../.deps/mujoco-3.14.0/lib")
    find_file(MUJOCO_DLL mujoco.dll
              HINTS "${MUJOCO_DIR}/bin" "${CMAKE_CURRENT_SOURCE_DIR}/../.deps/mujoco-3.14.0/bin")
  else()
    find_library(MUJOCO_LIBRARY NAMES mujoco libmujoco.so
                 HINTS "${MUJOCO_DIR}/lib" /opt/mujoco/lib /usr/local/lib)
  endif()
  if(NOT MUJOCO_INCLUDE_DIR OR NOT MUJOCO_LIBRARY)
    message(FATAL_ERROR "MuJoCo not found. Set -DMUJOCO_DIR=<release dir> or the MUJOCO_DIR env var.")
  endif()
  add_library(mujoco::mujoco SHARED IMPORTED)
  set_target_properties(mujoco::mujoco PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${MUJOCO_INCLUDE_DIR}")
  if(WIN32)
    set_target_properties(mujoco::mujoco PROPERTIES IMPORTED_IMPLIB "${MUJOCO_LIBRARY}"
                                                    IMPORTED_LOCATION "${MUJOCO_DLL}")
  else()
    set_target_properties(mujoco::mujoco PROPERTIES IMPORTED_LOCATION "${MUJOCO_LIBRARY}")
  endif()
endif()

# ---------------------------------------------------------------- yaml-cpp
find_package(yaml-cpp QUIET)
if(TARGET yaml-cpp::yaml-cpp)
  set(FSSIM_YAML_TARGET yaml-cpp::yaml-cpp)
elseif(TARGET yaml-cpp)
  set(FSSIM_YAML_TARGET yaml-cpp)
else()
  message(STATUS "yaml-cpp not installed: fetching it from github.com")
  include(FetchContent)
  set(YAML_CPP_BUILD_TESTS OFF CACHE BOOL "" FORCE)
  set(YAML_CPP_BUILD_TOOLS OFF CACHE BOOL "" FORCE)
  set(YAML_CPP_BUILD_CONTRIB OFF CACHE BOOL "" FORCE)
  set(YAML_CPP_INSTALL OFF CACHE BOOL "" FORCE)
  set(CMAKE_POLICY_VERSION_MINIMUM 3.5)
  FetchContent_Declare(yaml-cpp GIT_REPOSITORY https://github.com/jbeder/yaml-cpp.git
                       GIT_TAG 0.8.0 GIT_SHALLOW TRUE)
  FetchContent_MakeAvailable(yaml-cpp)
  set(FSSIM_YAML_TARGET yaml-cpp)
endif()

# ---------------------------------------------------------------- OpenGL context provider
if(WIN32)
  # Hidden GLFW window gives us a desktop GL context for local development.
  find_package(glfw3 QUIET)
  if(NOT TARGET glfw)
    include(FetchContent)
    set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
    set(GLFW_BUILD_TESTS OFF CACHE BOOL "" FORCE)
    set(GLFW_BUILD_DOCS OFF CACHE BOOL "" FORCE)
    set(GLFW_INSTALL OFF CACHE BOOL "" FORCE)
    set(BUILD_SHARED_LIBS_SAVED ${BUILD_SHARED_LIBS})
    set(BUILD_SHARED_LIBS OFF)
    FetchContent_Declare(glfw GIT_REPOSITORY https://github.com/glfw/glfw.git
                         GIT_TAG 3.4 GIT_SHALLOW TRUE)
    FetchContent_MakeAvailable(glfw)
    set(BUILD_SHARED_LIBS ${BUILD_SHARED_LIBS_SAVED})
  endif()
  set(FSSIM_GL_SRC src/gl_context_glfw.cpp)
  set(FSSIM_GL_LIBS glfw opengl32)
else()
  # Headless: EGL + desktop OpenGL (works with the NVIDIA driver, no X server needed).
  find_package(OpenGL REQUIRED COMPONENTS OpenGL EGL)
  set(FSSIM_GL_SRC src/gl_context_egl.cpp)
  set(FSSIM_GL_LIBS OpenGL::EGL OpenGL::OpenGL ${CMAKE_DL_LIBS})
endif()

# ---------------------------------------------------------------- core library
add_library(fs_sim_core STATIC
  src/config.cpp
  src/trajectory.cpp
  src/renderer.cpp
  src/bboxes.cpp
  src/simulation.cpp
  ${FSSIM_GL_SRC})
add_library(fs_mono_cam_sim::core ALIAS fs_sim_core)
target_include_directories(fs_sim_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_link_libraries(fs_sim_core PUBLIC mujoco::mujoco PRIVATE ${FSSIM_YAML_TARGET} ${FSSIM_GL_LIBS})
if(MSVC)
  target_compile_options(fs_sim_core PRIVATE /W3 /utf-8)
  target_compile_definitions(fs_sim_core PRIVATE _CRT_SECURE_NO_WARNINGS NOMINMAX)
else()
  target_compile_options(fs_sim_core PRIVATE -Wall -Wextra)
endif()

# Copy mujoco.dll next to an executable (Windows only).
function(fssim_copy_mujoco_dll tgt)
  if(WIN32 AND MUJOCO_DLL)
    add_custom_command(TARGET ${tgt} POST_BUILD
      COMMAND ${CMAKE_COMMAND} -E copy_if_different "${MUJOCO_DLL}" "$<TARGET_FILE_DIR:${tgt}>")
  endif()
endfunction()

# ---------------------------------------------------------------- tools + tests
if(FSSIM_BUILD_TOOLS)
  add_executable(fsim_bench tools/bench.cpp)
  target_link_libraries(fsim_bench PRIVATE fs_sim_core)
  fssim_copy_mujoco_dll(fsim_bench)

  find_package(nlohmann_json 3.2 QUIET)
  if(NOT TARGET nlohmann_json::nlohmann_json)
    include(FetchContent)
    set(JSON_BuildTests OFF CACHE INTERNAL "")
    set(JSON_Install OFF CACHE INTERNAL "")
    FetchContent_Declare(nlohmann_json GIT_REPOSITORY https://github.com/nlohmann/json.git
                         GIT_TAG v3.11.3 GIT_SHALLOW TRUE)
    FetchContent_MakeAvailable(nlohmann_json)
  endif()
  add_executable(fsim_test_golden test/test_golden.cpp)
  target_link_libraries(fsim_test_golden PRIVATE fs_sim_core nlohmann_json::nlohmann_json)
  fssim_copy_mujoco_dll(fsim_test_golden)
  if(MSVC)
    target_compile_options(fsim_bench PRIVATE /W3)
    target_compile_options(fsim_test_golden PRIVATE /W3)
    target_compile_definitions(fsim_bench PRIVATE _CRT_SECURE_NO_WARNINGS NOMINMAX)
    target_compile_definitions(fsim_test_golden PRIVATE _CRT_SECURE_NO_WARNINGS NOMINMAX)
  endif()
endif()
