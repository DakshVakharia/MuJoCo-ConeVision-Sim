// Small internal helpers shared by the GL context implementations and renderer.cpp.
// (Not part of the public API in include/.)
#pragma once
#include <string>

namespace fsim {

// glXGetProcAddress-style lookup for GL entry points beyond OpenGL 1.1 (needed on Windows, where
// opengl32.dll only exports 1.1, and used on Linux for PBO functions). The GL context must be current.
void* gl_get_proc(const char* name);

// glGetString(GL_RENDERER) of the current context, for logging which GPU was picked.
std::string gl_renderer_string();

}  // namespace fsim
