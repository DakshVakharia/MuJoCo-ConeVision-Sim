// Windows (development) GL context: an invisible GLFW window. MuJoCo needs a desktop-GL
// compatibility context, which is what GLFW creates when no version/profile hints are given.
#include <stdexcept>
#include <string>

#include <GLFW/glfw3.h>

#include "conevision_sim/renderer.h"
#include "gl_internal.h"

namespace cvsim {
namespace {

class GlfwContext : public GlContext {
 public:
  GlfwContext() {
    if (!glfwInit()) throw std::runtime_error("glfwInit failed");
    glfwWindowHint(GLFW_VISIBLE, GLFW_FALSE);
    window_ = glfwCreateWindow(16, 16, "conevision_sim", nullptr, nullptr);
    if (!window_) {
      glfwTerminate();
      throw std::runtime_error("glfwCreateWindow failed (no desktop OpenGL?)");
    }
    make_current();
    glfwSwapInterval(0);   // irrelevant offscreen, but never wait for vsync
  }
  ~GlfwContext() override {
    if (window_) glfwDestroyWindow(window_);
    glfwTerminate();
  }
  void make_current() override { glfwMakeContextCurrent(window_); }

 private:
  GLFWwindow* window_ = nullptr;
};

}  // namespace

std::unique_ptr<GlContext> GlContext::create() { return std::make_unique<GlfwContext>(); }

void* gl_get_proc(const char* name) { return reinterpret_cast<void*>(glfwGetProcAddress(name)); }

std::string gl_renderer_string() {
  const char* s = reinterpret_cast<const char*>(glGetString(GL_RENDERER));
  return s ? s : "unknown";
}

}  // namespace cvsim
