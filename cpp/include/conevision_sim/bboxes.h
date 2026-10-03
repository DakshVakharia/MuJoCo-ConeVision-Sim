// Fake-YOLO boxes from a segmentation image. Port of src/conevision_sim/render/bboxes.py.
#pragma once

#include <cstdint>
#include <random>
#include <string>
#include <vector>

#include <mujoco/mujoco.h>

#include "conevision_sim/config.h"
#include "conevision_sim/renderer.h"

namespace cvsim {

struct Detection {
  std::string cls;                 // blue | yellow | orange | orange_big
  int index = -1;                  // cone index from body name cone_<cls>_<index>
  double x_min = 0, y_min = 0, x_max = 0, y_max = 0;   // camera pixels; edges (inclusive extent + 1)
  int visible_pixels = 0;          // in camera pixels (seg pixel count / scale^2)
  double range_m = 0;              // |cone body origin - camera position|
};

class BoxComputer {
 public:
  // Builds the geom -> cone-body lookup from body names `cone_<cls>_<index>`.
  BoxComputer(const mjModel* m, const BBoxConfig& cfg, unsigned long long seed);

  // Same filters, noise model and ordering (nearest first) as bboxes.py compute_bboxes.
  // Boxes are scaled to camera pixels using seg.scale and clipped to [0, cam_w] x [0, cam_h].
  std::vector<Detection> compute(const mjData* d, int cam_id, const SegImage& seg,
                                 int cam_w, int cam_h);

 private:
  const mjModel* m_;
  BBoxConfig cfg_;
  std::mt19937_64 rng_;
  std::vector<int32_t> geom_to_cone_;   // geom id -> cone body id or -1
  std::vector<int> cls_of_body_;        // body id -> index into class names or -1
  std::vector<int> index_of_body_;      // body id -> cone index or -1
};

}  // namespace cvsim
