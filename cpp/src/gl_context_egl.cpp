// Linux headless GL context via EGL. Picks an NVIDIA device if several are available
// (EGL_EXT_device_enumeration), so the sim renders on the dGPU without an X server.
// Override the choice with env FSIM_EGL_DEVICE=<index>.
// NOTE: written for Ubuntu 20.04 + NVIDIA driver; not compiled/tested on the Windows dev machine.
#include <EGL/egl.h>
#include <EGL/eglext.h>
#include <GL/gl.h>

#include <cstdlib>
#include <cstring>
#include <stdexcept>
#include <string>
#include <vector>

#include "fs_mono_cam_sim/renderer.h"
#include "gl_internal.h"

namespace fsim {
namespace {

class EglContext : public GlContext {
 public:
  EglContext() {
    display_ = pick_display();
    if (display_ == EGL_NO_DISPLAY) throw std::runtime_error("EGL: no usable display/device");
    EGLint maj = 0, min = 0;
    if (!eglInitialize(display_, &maj, &min)) throw std::runtime_error("eglInitialize failed");
    if (!eglBindAPI(EGL_OPENGL_API)) throw std::runtime_error("eglBindAPI(EGL_OPENGL_API) failed");

    const EGLint cfg_attr[] = {EGL_SURFACE_TYPE, EGL_PBUFFER_BIT, EGL_RENDERABLE_TYPE, EGL_OPENGL_BIT,
                               EGL_RED_SIZE, 8, EGL_GREEN_SIZE, 8, EGL_BLUE_SIZE, 8,
                               EGL_DEPTH_SIZE, 24, EGL_NONE};
    EGLConfig cfg;
    EGLint n = 0;
    if (!eglChooseConfig(display_, cfg_attr, &cfg, 1, &n) || n < 1)
      throw std::runtime_error("eglChooseConfig found no desktop-GL config");
    // No version/profile attributes: gives the compatibility context MuJoCo's mjr_* needs.
    context_ = eglCreateContext(display_, cfg, EGL_NO_CONTEXT, nullptr);
    if (context_ == EGL_NO_CONTEXT) throw std::runtime_error("eglCreateContext failed");

    // Surfaceless first (EGL_KHR_surfaceless_context); fall back to a 1x1 pbuffer.
    if (!eglMakeCurrent(display_, EGL_NO_SURFACE, EGL_NO_SURFACE, context_)) {
      const EGLint pb[] = {EGL_WIDTH, 1, EGL_HEIGHT, 1, EGL_NONE};
      surface_ = eglCreatePbufferSurface(display_, cfg, pb);
      if (surface_ == EGL_NO_SURFACE || !eglMakeCurrent(display_, surface_, surface_, context_))
        throw std::runtime_error("eglMakeCurrent failed");
    }
  }

  ~EglContext() override {
    if (display_ != EGL_NO_DISPLAY) {
      eglMakeCurrent(display_, EGL_NO_SURFACE, EGL_NO_SURFACE, EGL_NO_CONTEXT);
      if (surface_ != EGL_NO_SURFACE) eglDestroySurface(display_, surface_);
      if (context_ != EGL_NO_CONTEXT) eglDestroyContext(display_, context_);
      eglTerminate(display_);
    }
  }

  void make_current() override {
    eglBindAPI(EGL_OPENGL_API);
    eglMakeCurrent(display_, surface_, surface_, context_);
  }

 private:
  static EGLDisplay pick_display() {
    auto query_devices = reinterpret_cast<PFNEGLQUERYDEVICESEXTPROC>(eglGetProcAddress("eglQueryDevicesEXT"));
    auto get_platform_display =
        reinterpret_cast<PFNEGLGETPLATFORMDISPLAYEXTPROC>(eglGetProcAddress("eglGetPlatformDisplayEXT"));
    if (query_devices && get_platform_display) {
      EGLint count = 0;
      query_devices(0, nullptr, &count);
      std::vector<EGLDeviceEXT> devs(count > 0 ? count : 0);
      if (count > 0) query_devices(count, devs.data(), &count);

      const char* forced = std::getenv("FSIM_EGL_DEVICE");
      EGLDisplay fallback = EGL_NO_DISPLAY;
      for (EGLint i = 0; i < count; ++i) {
        if (forced && std::atoi(forced) != i) continue;
        EGLDisplay d = get_platform_display(EGL_PLATFORM_DEVICE_EXT, devs[i], nullptr);
        EGLint a, b;
        if (d == EGL_NO_DISPLAY || !eglInitialize(d, &a, &b)) continue;
        const char* vendor = eglQueryString(d, EGL_VENDOR);
        if (forced || (vendor && std::strstr(vendor, "NVIDIA"))) return d;   // initialize is refcounted
        if (fallback == EGL_NO_DISPLAY) fallback = d;
      }
      if (fallback != EGL_NO_DISPLAY) return fallback;
    }
    return eglGetDisplay(EGL_DEFAULT_DISPLAY);
  }

  EGLDisplay display_ = EGL_NO_DISPLAY;
  EGLContext context_ = EGL_NO_CONTEXT;
  EGLSurface surface_ = EGL_NO_SURFACE;
};

}  // namespace

std::unique_ptr<GlContext> GlContext::create() { return std::make_unique<EglContext>(); }

void* gl_get_proc(const char* name) { return reinterpret_cast<void*>(eglGetProcAddress(name)); }

std::string gl_renderer_string() {
  const char* s = reinterpret_cast<const char*>(glGetString(GL_RENDERER));
  return s ? s : "unknown";
}

}  // namespace fsim
