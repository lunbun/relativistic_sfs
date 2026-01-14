#pragma once

#include <glad/gl.h>

GLuint compileShaderProgram(const char *vertexSource, const char *fragmentSource, int *success, const char *name);
