#include "trajectory.h"

// clang-format off
#include <glad/gl.h>
#include <GLFW/glfw3.h>
#include <entt/entt.hpp>
// clang-format on

#include "physics/kepler.h"
#include "physics/physics.h"
#include "render/gl/shader.h"
#include "render/scene/camera.h"

namespace sfs::render {

namespace {

extern "C" const char EMBED_START_ASSETS_TRAJ_VERT_GLSL[];
extern "C" const char EMBED_START_ASSETS_TRAJ_FRAG_GLSL[];

GLuint trajectoryVAO, trajectoryVBO;
GLuint trajectoryShaderProgram;
GLuint trajectory_uPositionLoc, trajectory_uViewLoc, trajectory_uProjectionLoc;

void initTrajectoryBuffers() {
    glGenVertexArrays(1, &trajectoryVAO);
    glGenBuffers(1, &trajectoryVBO);

    glBindVertexArray(trajectoryVAO);
    glBindBuffer(GL_ARRAY_BUFFER, trajectoryVBO);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void *) 0);
    glEnableVertexAttribArray(0);

    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);
}

void initShaders() {
    int success;
    trajectoryShaderProgram = compileShaderProgram(EMBED_START_ASSETS_TRAJ_VERT_GLSL, EMBED_START_ASSETS_TRAJ_FRAG_GLSL, &success, "trajectory");
    if (!success) {
        throw std::runtime_error("Failed to compile trajectory shader program");
    }

    glUseProgram(trajectoryShaderProgram);
    trajectory_uPositionLoc = glGetUniformLocation(trajectoryShaderProgram, "uPosition");
    trajectory_uViewLoc = glGetUniformLocation(trajectoryShaderProgram, "uView");
    trajectory_uProjectionLoc = glGetUniformLocation(trajectoryShaderProgram, "uProjection");
    glUseProgram(0);
}

} // namespace

void initRenderTrajectorySystem() {
    initTrajectoryBuffers();
    initShaders();
}

void renderTrajectories(entt::registry &registry, entt::entity camera) {
    glLineWidth(1.0f);

    glUseProgram(trajectoryShaderProgram);
    glBindVertexArray(trajectoryVAO);
    glBindBuffer(GL_ARRAY_BUFFER, trajectoryVBO);

    auto &cameraData = registry.get<Camera>(camera);
    glUniformMatrix4fv(trajectory_uViewLoc, 1, GL_FALSE, cameraData.viewMatrix.data());
    glUniformMatrix4fv(trajectory_uProjectionLoc, 1, GL_FALSE, cameraData.projectionMatrix.data());

    constexpr int n = 250;
    auto view = registry.view<physics::BodyState, physics::KeplerParameters, RenderTrajectory>();
    std::vector<Eigen::Vector3d> points;
    points.reserve(n);
    std::vector<float> bufferData;
    bufferData.reserve(3 * n);
    for (auto entity : view) {
        auto &state = view.get<physics::BodyState>(entity);
        if (state.st.primary == entt::null) continue;

        auto &primaryState = view.get<physics::BodyState>(state.st.primary);
        auto &p = view.get<physics::KeplerParameters>(entity);
        points.clear();
        sampleTrajectoryPoints(p, points, n);

        bufferData.clear();
        for (const auto &pt : points) {
            bufferData.push_back(static_cast<float>(pt.x()));
            bufferData.push_back(static_cast<float>(pt.y()));
            bufferData.push_back(static_cast<float>(pt.z()));
        }
        glBufferData(GL_ARRAY_BUFFER, bufferData.size() * sizeof(float), bufferData.data(), GL_DYNAMIC_DRAW);

        // TODO: to avoid floating point error, we should calculate the position
        //  relative to the camera focus
        Eigen::Vector3d primaryPos = calculateAbsolutePosition(registry, primaryState);
        glUniform3f(trajectory_uPositionLoc, primaryPos.x(), primaryPos.y(), primaryPos.z());
        glDrawArrays(GL_LINE_STRIP, 0, points.size());

        // glBegin(GL_LINE_STRIP);
        // for (const auto &pt : points) {
        //     Eigen::Vector3d screenPos = (primaryState.st.pos + pt) * kRenderScale;
        //     glVertex2d(screenPos.x(), screenPos.y());
        // }
        // glEnd();
    }

    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);
    glUseProgram(0);
}

} // namespace sfs::render
