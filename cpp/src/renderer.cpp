// Port of src/conevision_sim/render/renderer.py.
//
// One GL context, two mjrContexts:
//   * ctx_rgb: the normal colour render (MSAA if the scene XML / render.msaa_samples asks for it)
//   * ctx_seg: segmentation render WITHOUT MSAA (ids must never be blended, and it is faster)
// The seg pass uses MuJoCo's mjRND_SEGMENT + mjRND_IDCOLOR flags: every geom is drawn flat in a colour
// that encodes (segid + 1) as a little-endian 24-bit number; 0 means "nothing drawn" (background).
#include "conevision_sim/renderer.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <stdexcept>

#include "gl_internal.h"

namespace cvsim {

namespace {
constexpr int kMaxGeom = 10000;      // same as mujoco.Renderer's default scene capacity
constexpr unsigned kGlPackAlignment = 0x0D05;
}  // namespace

struct CameraRenderer::Impl {
  std::unique_ptr<GlContext> gl;
  mjvScene scn;
  mjvOption opt;
  mjvCamera cam;
  mjrContext ctx_rgb, ctx_seg;
  const mjModel* m = nullptr;
  int cam_w = 0, cam_h = 0;           // camera (RGB) resolution
  int seg_w = 0, seg_h = 0;           // segmentation resolution (cam * seg_scale)
  double seg_scale = 1.0;
  mjrRect rect_rgb{}, rect_seg{};
  std::vector<uint8_t> rgb_raw;       // bottom-up RGB straight from GL
  std::vector<uint8_t> seg_raw;       // bottom-up id colours
  std::vector<int32_t> lut;           // segid + 1 -> geom id (0 = background -> -1)
  void (*gl_pixel_store)(unsigned, int) = nullptr;
  bool contexts_made = false;
};

CameraRenderer::CameraRenderer(const mjModel* m, const SimConfig& cfg) : impl_(new Impl) {
  Impl& s = *impl_;
  s.m = m;
  cam_id_ = mj_name2id(m, mjOBJ_CAMERA, cfg.camera.name.c_str());
  if (cam_id_ < 0) throw std::runtime_error("camera '" + cfg.camera.name + "' not found in the model");
  s.cam_w = cfg.camera.width;
  s.cam_h = cfg.camera.height;
  if (s.cam_w > m->vis.global.offwidth || s.cam_h > m->vis.global.offheight)
    throw std::runtime_error("camera resolution exceeds <visual><global offwidth/offheight> of the scene");
  s.seg_scale = cfg.render.seg_scale;
  s.seg_w = std::max(1, static_cast<int>(std::lround(s.cam_w * s.seg_scale)));
  s.seg_h = std::max(1, static_cast<int>(std::lround(s.cam_h * s.seg_scale)));
  s.seg_scale = static_cast<double>(s.seg_w) / s.cam_w;
  s.rect_rgb = {0, 0, s.cam_w, s.cam_h};
  s.rect_seg = {0, 0, s.seg_w, s.seg_h};

  s.gl = GlContext::create();
  s.gl->make_current();
  s.gl_pixel_store = reinterpret_cast<void (*)(unsigned, int)>(gl_get_proc("glPixelStorei"));

  mjv_defaultOption(&s.opt);
  mjv_defaultCamera(&s.cam);
  s.cam.type = mjCAMERA_FIXED;
  s.cam.fixedcamid = cam_id_;
  mjv_defaultScene(&s.scn);
  mjv_makeScene(m, &s.scn, kMaxGeom);

  // mjr_makeContext reads the MSAA sample count from the model, so patch it temporarily
  // (RGB: configured value, seg: 0), exactly like renderer.py does.
  mjModel* mm = const_cast<mjModel*>(m);
  const int orig_samples = mm->vis.quality.offsamples;
  if (cfg.render.msaa_samples >= 0) mm->vis.quality.offsamples = cfg.render.msaa_samples;
  mjr_defaultContext(&s.ctx_rgb);
  mjr_makeContext(m, &s.ctx_rgb, mjFONTSCALE_100);
  mm->vis.quality.offsamples = 0;
  mjr_defaultContext(&s.ctx_seg);
  mjr_makeContext(m, &s.ctx_seg, mjFONTSCALE_100);
  mm->vis.quality.offsamples = orig_samples;
  s.contexts_made = true;
  if (s.ctx_rgb.offFBO == 0)
    throw std::runtime_error("failed to create an offscreen framebuffer (OpenGL too old?)");

  s.rgb_raw.resize(static_cast<size_t>(s.cam_w) * s.cam_h * 3);
  s.seg_raw.resize(static_cast<size_t>(s.seg_w) * s.seg_h * 3);
  s.lut.assign(kMaxGeom + 1, -1);
}

CameraRenderer::~CameraRenderer() {
  if (!impl_) return;
  Impl& s = *impl_;
  if (s.gl) s.gl->make_current();
  if (s.contexts_made) {
    mjr_freeContext(&s.ctx_rgb);
    mjr_freeContext(&s.ctx_seg);
  }
  mjv_freeScene(&s.scn);
}

void CameraRenderer::render(const mjData* d, RgbImage& rgb, SegImage& seg) {
  Impl& s = *impl_;
  s.gl->make_current();
  // mjv_updateScene wants a non-const mjData because it may touch plugin/contact buffers; we do not.
  mjv_updateScene(s.m, const_cast<mjData*>(d), &s.opt, nullptr, &s.cam, mjCAT_ALL, &s.scn);

  // ---- RGB pass (MSAA resolved inside mjr_readPixels)
  mjr_setBuffer(mjFB_OFFSCREEN, &s.ctx_rgb);
  if (s.gl_pixel_store) s.gl_pixel_store(kGlPackAlignment, 1);
  mjr_render(s.rect_rgb, &s.scn, &s.ctx_rgb);
  mjr_readPixels(s.rgb_raw.data(), nullptr, s.rect_rgb, &s.ctx_rgb);

  // GL origin is bottom-left; flip so row 0 is the top of the image.
  rgb.width = s.cam_w;
  rgb.height = s.cam_h;
  rgb.data.resize(s.rgb_raw.size());
  const size_t row = static_cast<size_t>(s.cam_w) * 3;
  for (int y = 0; y < s.cam_h; ++y)
    std::memcpy(rgb.data.data() + y * row, s.rgb_raw.data() + (s.cam_h - 1 - y) * row, row);

  // ---- segmentation pass (no MSAA, possibly smaller viewport)
  s.scn.flags[mjRND_SEGMENT] = 1;
  s.scn.flags[mjRND_IDCOLOR] = 1;
  mjr_setBuffer(mjFB_OFFSCREEN, &s.ctx_seg);
  if (s.gl_pixel_store) s.gl_pixel_store(kGlPackAlignment, 1);
  mjr_render(s.rect_seg, &s.scn, &s.ctx_seg);
  mjr_readPixels(s.seg_raw.data(), nullptr, s.rect_seg, &s.ctx_seg);
  s.scn.flags[mjRND_SEGMENT] = 0;
  s.scn.flags[mjRND_IDCOLOR] = 0;

  // ---- decode colour -> geom id. lut: (segid + 1) -> geom id, only for geoms that came from mjOBJ_GEOM.
  const int ng = std::min(s.scn.ngeom, kMaxGeom);
  std::fill(s.lut.begin(), s.lut.begin() + ng + 1, -1);
  for (int i = 0; i < ng; ++i) {
    const mjvGeom& g = s.scn.geoms[i];
    if (g.segid >= 0 && g.segid < ng && g.objtype == mjOBJ_GEOM) s.lut[g.segid + 1] = g.objid;
  }
  seg.width = s.seg_w;
  seg.height = s.seg_h;
  seg.scale = s.seg_scale;
  seg.geom.resize(static_cast<size_t>(s.seg_w) * s.seg_h);
  const int32_t* lut = s.lut.data();
  const uint32_t max_code = static_cast<uint32_t>(ng);
  for (int y = 0; y < s.seg_h; ++y) {
    const uint8_t* src = s.seg_raw.data() + static_cast<size_t>(s.seg_h - 1 - y) * s.seg_w * 3;
    int32_t* dst = seg.geom.data() + static_cast<size_t>(y) * s.seg_w;
    for (int x = 0; x < s.seg_w; ++x, src += 3) {
      const uint32_t code = src[0] | (src[1] << 8) | (src[2] << 16);
      dst[x] = code <= max_code ? lut[code] : -1;
    }
  }
}

}  // namespace cvsim
