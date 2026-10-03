// Offscreen camera rendering: RGB + per-pixel geom ids, one GL context.
#pragma once

#include <cstdint>
#include <memory>
#include <vector>

#include <mujoco/mujoco.h>

#include "fs_mono_cam_sim/config.h"

namespace fsim {

// Owns an OpenGL context with no visible window:
//   Linux  : EGL headless (prefers an NVIDIA device via EGL_EXT_device_enumeration)
//   Windows: GLFW hidden window (for local development/testing)
class GlContext {
 public:
  virtual ~GlContext() = default;
  virtual void make_current() = 0;
  static std::unique_ptr<GlContext> create();   // throws std::runtime_error on failure
};

struct RgbImage {                 // tightly packed rgb8, row 0 = TOP of the image
  int width = 0, height = 0;
  std::vector<uint8_t> data;      // width*height*3
};

struct SegImage {                 // geom id per pixel, -1 = no geom; row 0 = TOP
  int width = 0, height = 0;      // may be smaller than the camera (render.seg_scale)
  double scale = 1.0;             // camera_px = seg_px / scale
  std::vector<int32_t> geom;      // width*height
};

class CameraRenderer {
 public:
  // Creates its own GlContext, an mjvScene, and two mjrContexts in that GL context:
  // RGB (offsamples = render.msaa_samples or the model value) and segmentation (no MSAA).
  // Port of src/fs_mono_cam_sim/render/renderer.py. All calls must come from ONE thread.
  CameraRenderer(const mjModel* m, const SimConfig& cfg);
  ~CameraRenderer();
  CameraRenderer(const CameraRenderer&) = delete;
  CameraRenderer& operator=(const CameraRenderer&) = delete;

  // Renders RGB and segmentation for the current mjData state (after mj_forward).
  // Implementations may pipeline GPU readback (PBOs) internally but must return the images
  // of THIS state.
  void render(const mjData* d, RgbImage& rgb, SegImage& seg);

  int camera_id() const { return cam_id_; }

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
  int cam_id_ = -1;
};

}  // namespace fsim
