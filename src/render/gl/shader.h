#pragma once

#include <glad/gl.h>

namespace sfs::render {

GLuint compileShaderProgram(const char *vertexSource, const char *fragmentSource, int *success, const char *name);

} // namespace sfs::render
