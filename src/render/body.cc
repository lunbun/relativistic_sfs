#include "body.h"

#include <iostream>

// clang-format off
#include <glad/gl.h>
#include <GLFW/glfw3.h>
#include <entt/entt.hpp>
// clang-format on

#include "physics/kepler.h"
#include "physics/physics.h"
#include "render/camera.h"
#include "render/window.h"

namespace {

const char *dotVertexShaderSource = R"glsl(
#version 330 core

uniform float uSize;
uniform vec3 uPosition;
uniform mat4 uView;
uniform mat4 uProjection;

layout(location = 0) in vec2 aPos;

out vec2 fragPos;

void main() {
    fragPos = aPos;

    vec4 projPos = uProjection * (uView * vec4(uPosition, 1.0));
    gl_Position = projPos + vec4(uSize * projPos.w * aPos, 0.0, 0.0);
}
)glsl";

const char *dotFragmentShaderSource = R"glsl(
#version 330 core

in vec2 fragPos;

out vec4 FragColor;

void main() {
    float edge = 0.01;
    float radius = 0.5 - edge;
    float dist = length(fragPos);
    float alpha = 1.0 - smoothstep(radius - edge, radius + edge, dist);
    FragColor = vec4(1.0, 1.0, 1.0, alpha);
}
)glsl";

const char *trajectoryVertexShaderSource = R"glsl(
#version 330 core

uniform vec3 uPosition;
uniform mat4 uView;
uniform mat4 uProjection;

layout(location = 0) in vec3 aPos;

void main() {
    gl_Position = uProjection * (uView * vec4(uPosition + aPos, 1.0));
}
)glsl";

const char *trajectoryFragmentShaderSource = R"glsl(
#version 330 core

out vec4 FragColor;

void main() {
    FragColor = vec4(1.0, 1.0, 1.0, 1.0);
}
)glsl";

GLuint dotVAO, dotVBO;
GLuint dotShaderProgram;
GLuint dot_uSizeLoc, dot_uPositionLoc, dot_uViewLoc, dot_uProjectionLoc;

GLuint trajectoryVAO, trajectoryVBO;
GLuint trajectoryShaderProgram;
GLuint trajectory_uPositionLoc, trajectory_uViewLoc, trajectory_uProjectionLoc;

void initDotBuffers() {
    float vertices[] = { -0.5f, 0.5f, 0.5f,  0.5f,  -0.5f, -0.5f,
                         0.5f,  0.5f, -0.5f, -0.5f, 0.5f,  -0.5f };

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

void initTrajectoryBuffers() {
    glGenVertexArrays(1, &trajectoryVAO);
    glGenBuffers(1, &trajectoryVBO);

    glBindVertexArray(trajectoryVAO);
    glBindBuffer(GL_ARRAY_BUFFER, trajectoryVBO);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void *)0);
    glEnableVertexAttribArray(0);

    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);
}

GLuint compileShaderProgram(const char *vertexSource, const char *fragmentSource, const char *name) {
    GLuint vertexShader = glCreateShader(GL_VERTEX_SHADER);
    glShaderSource(vertexShader, 1, &vertexSource, nullptr);
    glCompileShader(vertexShader);

    int success;
    char infoLog[512];
    glGetShaderiv(vertexShader, GL_COMPILE_STATUS, &success);
    if (!success) {
        glGetShaderInfoLog(vertexShader, 512, nullptr, infoLog);
        std::cerr << "Vertex shader compilation for " << name << " failed\n" << infoLog << std::endl;
    }

    GLuint fragmentShader = glCreateShader(GL_FRAGMENT_SHADER);
    glShaderSource(fragmentShader, 1, &fragmentSource, nullptr);
    glCompileShader(fragmentShader);
    glGetShaderiv(fragmentShader, GL_COMPILE_STATUS, &success);
    if (!success) {
        glGetShaderInfoLog(fragmentShader, 512, nullptr, infoLog);
        std::cerr << "Fragment shader compilation for " << name << " failed\n" << infoLog << std::endl;
    }

    GLuint program = glCreateProgram();
    glAttachShader(program, vertexShader);
    glAttachShader(program, fragmentShader);
    glLinkProgram(program);

    glGetProgramiv(program, GL_LINK_STATUS, &success);
    if (!success) {
        glGetProgramInfoLog(program, 512, nullptr, infoLog);
        std::cerr << "Shader program linking for " << name << " failed\n" << infoLog << std::endl;
    }

    glDeleteShader(vertexShader);
    glDeleteShader(fragmentShader);

    return program;
}

void initShaders() {
    dotShaderProgram = compileShaderProgram(dotVertexShaderSource, dotFragmentShaderSource, "dot");

    glUseProgram(dotShaderProgram);
    dot_uSizeLoc = glGetUniformLocation(dotShaderProgram, "uSize");
    dot_uPositionLoc = glGetUniformLocation(dotShaderProgram, "uPosition");
    dot_uViewLoc = glGetUniformLocation(dotShaderProgram, "uView");
    dot_uProjectionLoc = glGetUniformLocation(dotShaderProgram, "uProjection");
    glUseProgram(0);

    trajectoryShaderProgram = compileShaderProgram(trajectoryVertexShaderSource, trajectoryFragmentShaderSource, "trajectory");

    glUseProgram(trajectoryShaderProgram);
    trajectory_uPositionLoc = glGetUniformLocation(trajectoryShaderProgram, "uPosition");
    trajectory_uViewLoc = glGetUniformLocation(trajectoryShaderProgram, "uView");
    trajectory_uProjectionLoc = glGetUniformLocation(trajectoryShaderProgram, "uProjection");
    glUseProgram(0);
}

} // namespace

void initRenderBodySystem() {
    initDotBuffers();
    initTrajectoryBuffers();
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

void renderTrajectories(entt::registry &registry, entt::entity camera) {
    glLineWidth(1.0f);

    glUseProgram(trajectoryShaderProgram);
    glBindVertexArray(trajectoryVAO);
    glBindBuffer(GL_ARRAY_BUFFER, trajectoryVBO);

    auto &cameraData = registry.get<Camera>(camera);
    glUniformMatrix4fv(dot_uViewLoc, 1, GL_FALSE, cameraData.viewMatrix.data());
    glUniformMatrix4fv(dot_uProjectionLoc, 1, GL_FALSE, cameraData.projectionMatrix.data());

    constexpr int n = 250;
    auto view = registry.view<BodyState, KeplerParameters, RenderTrajectory>();
    std::vector<Eigen::Vector3d> points;
    points.reserve(n);
    std::vector<float> bufferData;
    bufferData.reserve(3 * n);
    for (auto entity : view) {
        auto &state = view.get<BodyState>(entity);
        if (state.st.primary == entt::null) continue;

        auto &primaryState = view.get<BodyState>(state.st.primary);
        auto &p = view.get<KeplerParameters>(entity);
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
