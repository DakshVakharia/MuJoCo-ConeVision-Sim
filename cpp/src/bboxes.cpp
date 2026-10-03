// Port of src/fs_mono_cam_sim/render/bboxes.py: boxes = extent of the visible pixels of each cone body.
//
// Performance: one pass over the segmentation image does geom -> cone body lookup and updates
// per-body count / min / max (no sorting, no per-pixel allocation). Everything after that is
// O(number of cones).
#include "fs_mono_cam_sim/bboxes.h"

#include <algorithm>
#include <climits>
#include <cmath>
#include <cstdlib>

namespace fsim {
namespace {

constexpr const char* kClasses[] = {"blue", "yellow", "orange", "orange_big"};
constexpr int kNumClasses = 4;

struct Stat {
  int count, x0, y0, x1, y1;   // x1/y1 inclusive pixel extents
};

}  // namespace

BoxComputer::BoxComputer(const mjModel* m, const BBoxConfig& cfg, unsigned long long seed)
    : m_(m), cfg_(cfg), rng_(seed) {
  const int nb = static_cast<int>(m->nbody);
  cls_of_body_.assign(nb, -1);
  index_of_body_.assign(nb, -1);
  for (int b = 0; b < nb; ++b) {
    const char* nm = mj_id2name(m, mjOBJ_BODY, b);
    if (!nm) continue;
    const std::string name = nm;
    if (name.compare(0, 5, "cone_") != 0) continue;
    const size_t us = name.rfind('_');                       // cone_<class>_<index>
    if (us == std::string::npos || us <= 5 || us + 1 >= name.size()) continue;
    const std::string cls = name.substr(5, us - 5), idx = name.substr(us + 1);
    if (idx.find_first_not_of("0123456789") != std::string::npos) continue;
    for (int c = 0; c < kNumClasses; ++c)
      if (cls == kClasses[c]) {
        cls_of_body_[b] = c;
        index_of_body_[b] = std::atoi(idx.c_str());
      }
  }
  geom_to_cone_.assign(m->ngeom, -1);
  for (int g = 0; g < m->ngeom; ++g) {
    const int b = m->geom_bodyid[g];
    if (cls_of_body_[b] >= 0) geom_to_cone_[g] = b;
  }
}

std::vector<Detection> BoxComputer::compute(const mjData* d, int cam_id, const SegImage& seg,
                                            int cam_w, int cam_h) {
  std::vector<Detection> out;
  const int nb = static_cast<int>(m_->nbody);

  // Scratch buffers kept across frames (allocated once per thread; the header has no member for them).
  thread_local std::vector<Stat> stats;
  if (static_cast<int>(stats.size()) < nb) stats.resize(nb);

  // ---------------------------------------------------------------- single pass over the pixels
  std::fill(stats.begin(), stats.begin() + nb, Stat{0, INT_MAX, INT_MAX, -1, -1});
  const int32_t* g2c = geom_to_cone_.data();
  const int ngeom = static_cast<int>(geom_to_cone_.size());
  const int32_t* px = seg.geom.data();
  for (int y = 0; y < seg.height; ++y) {
    for (int x = 0; x < seg.width; ++x) {
      const int32_t g = *px++;
      if (g < 0 || g >= ngeom) continue;
      const int32_t b = g2c[g];
      if (b < 0) continue;
      Stat& s = stats[b];
      ++s.count;
      if (x < s.x0) s.x0 = x;
      if (x > s.x1) s.x1 = x;
      if (y < s.y0) s.y0 = y;   // rows ascend, so y0 is the first row seen...
      s.y1 = y;                 // ...and y1 the last
    }
  }

  // ---------------------------------------------------------------- per-cone filters + noise
  const double inv_scale = 1.0 / seg.scale;
  const double area_scale = inv_scale * inv_scale;           // seg pixels -> camera pixels
  const int min_px = std::max(cfg_.min_visible_pixels, 1);
  const double* cam_pos = d->cam_xpos + 3 * cam_id;
  const bool noisy = cfg_.center_jitter_px > 0 || cfg_.size_jitter_frac > 0;
  std::normal_distribution<double> normal(0.0, 1.0);
  std::uniform_real_distribution<double> uniform(0.0, 1.0);

  for (int b = 0; b < nb; ++b) {                              // ascending body id, like np.flatnonzero
    const Stat& s = stats[b];
    if (s.count == 0 || cls_of_body_[b] < 0) continue;
    const long long vis = std::llround(s.count * area_scale);
    if (vis < min_px) continue;

    // inclusive pixel extent -> edges (+1), then back to camera pixels, clipped to the image
    double x0 = std::min(s.x0 * inv_scale, static_cast<double>(cam_w));
    double x1 = std::min((s.x1 + 1) * inv_scale, static_cast<double>(cam_w));
    double y0 = std::min(s.y0 * inv_scale, static_cast<double>(cam_h));
    double y1 = std::min((s.y1 + 1) * inv_scale, static_cast<double>(cam_h));
    if (y1 - y0 < cfg_.min_box_height_px) continue;

    const double* p = d->xpos + 3 * b;                        // camera -> cone base (body origin)
    const double dx = p[0] - cam_pos[0], dy = p[1] - cam_pos[1], dz = p[2] - cam_pos[2];
    const double range = std::sqrt(dx * dx + dy * dy + dz * dz);
    if (range > cfg_.max_range_m) continue;
    if (cfg_.dropout_prob > 0 && uniform(rng_) < cfg_.dropout_prob) continue;

    if (noisy) {
      double cx = (x0 + x1) / 2, cy = (y0 + y1) / 2;
      double bw = x1 - x0, bh = y1 - y0;
      if (cfg_.center_jitter_px > 0) {
        cx += normal(rng_) * cfg_.center_jitter_px;
        cy += normal(rng_) * cfg_.center_jitter_px;
      }
      if (cfg_.size_jitter_frac > 0) {
        bw *= std::max(0.1, 1 + normal(rng_) * cfg_.size_jitter_frac);
        bh *= std::max(0.1, 1 + normal(rng_) * cfg_.size_jitter_frac);
      }
      x0 = std::max(0.0, cx - bw / 2);
      x1 = std::min(static_cast<double>(cam_w), cx + bw / 2);
      y0 = std::max(0.0, cy - bh / 2);
      y1 = std::min(static_cast<double>(cam_h), cy + bh / 2);
      if (x1 <= x0 || y1 <= y0) continue;
    }

    Detection det;
    det.cls = kClasses[cls_of_body_[b]];
    det.index = index_of_body_[b];
    det.x_min = x0; det.y_min = y0; det.x_max = x1; det.y_max = y1;
    det.visible_pixels = static_cast<int>(vis);
    det.range_m = range;
    out.push_back(std::move(det));
  }
  std::stable_sort(out.begin(), out.end(),
                   [](const Detection& a, const Detection& c) { return a.range_m < c.range_m; });
  return out;
}

}  // namespace fsim
