#include "body.h"

// clang-format off
#include <glad/gl.h>
#include <GLFW/glfw3.h>
#include <entt/entt.hpp>
// clang-format on

#include "physics/kepler.h"
#include "physics/physics.h"
#include "render/gl/shader.h"
#include "render/scene/camera.h"

extern const char EMBED_START_ASSETS_DOT_VERT_GLSL[];
extern const char EMBED_START_ASSETS_DOT_FRAG_GLSL[];

namespace {

GLuint dotVAO, dotVBO;
GLuint dotShaderProgram;
GLuint dot_uSizeLoc, dot_uPositionLoc, dot_uViewLoc, dot_uProjectionLoc;

void initDotBuffers() {
    float vertices[] = { -0.5f, 0.5f, 0.5f, 0.5f, -0.5f, -0.5f, 0.5f, 0.5f, -0.5f, -0.5f, 0.5f, -0.5f };

    glGenVertexArrays(1, &dotVAO);
    glGenBuffers(1, &dotVBO);

    glBindVertexArray(dotVAO);

    glBindBuffer(GL_ARRAY_BUFFER, dotVBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);

    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 2 * sizeof(float), (void *) 0);
    glEnableVertexAttribArray(0);

    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);
}

void initShaders() {
    int success;
    dotShaderProgram = compileShaderProgram(EMBED_START_ASSETS_DOT_VERT_GLSL, EMBED_START_ASSETS_DOT_FRAG_GLSL, &success, "dot");
    if (!success) {
        throw std::runtime_error("Failed to compile trajectory shader program");
    }

    glUseProgram(dotShaderProgram);
    dot_uSizeLoc = glGetUniformLocation(dotShaderProgram, "uSize");
    dot_uPositionLoc = glGetUniformLocation(dotShaderProgram, "uPosition");
    dot_uViewLoc = glGetUniformLocation(dotShaderProgram, "uView");
    dot_uProjectionLoc = glGetUniformLocation(dotShaderProgram, "uProjection");
    glUseProgram(0);
}

} // namespace

void initRenderBodySystem() {
    initDotBuffers();
    initShaders();
}

void renderBodies(entt::registry &registry, entt::entity camera) {
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    glUseProgram(dotShaderProgram);
    glBindVertexArray(dotVAO);

    auto &cameraData = registry.get<Camera>(camera);
    glUniformMatrix4fv(dot_uViewLoc, 1, GL_FALSE, cameraData.viewMatrix.data());
    glUniformMatrix4fv(dot_uProjectionLoc, 1, GL_FALSE, cameraData.projectionMatrix.data());

    auto view = registry.view<BodyState, RenderDot>();
    double lastSize = -1.0;
    for (auto entity : view) {
        auto &body = view.get<BodyState>(entity);
        auto &dot = view.get<RenderDot>(entity);
        if (dot.size != lastSize) {
            lastSize = dot.size;
            glUniform1f(dot_uSizeLoc, dot.size);
        }
        // TODO: to avoid floating point error, we should calculate the position
        //  relative to the camera focus
        Eigen::Vector3d pos = calculateAbsolutePosition(registry, body);
        glUniform3f(dot_uPositionLoc, pos.x(), pos.y(), pos.z());
        glDrawArrays(GL_TRIANGLES, 0, 6);
    }

    glBindVertexArray(0);
    glUseProgram(0);

    glDisable(GL_BLEND);
}
