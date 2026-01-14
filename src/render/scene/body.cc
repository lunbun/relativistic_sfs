#include "body.h"

#include <cstddef>
#include <vector>

#include <glad/gl.h>

#include "physics/kepler.h"
#include "physics/physics.h"
#include "render/gl/shader.h"
#include "render/scene/camera.h"

namespace sfs::render {

namespace {

extern "C" const char EMBED_START_ASSETS_BODY_VERT_GLSL[];
extern "C" const char EMBED_START_ASSETS_BODY_FRAG_GLSL[];

int nIndices;
GLuint bodyVAO, bodyVBO, bodyEBO;
GLuint bodyShaderProgram;
GLuint body_uPositionLoc, body_uModelLoc, body_uViewLoc, body_uProjectionLoc;

struct Vertex {
    float x, y, z;
    float nx, ny, nz;
    float u, v;
};

void initBodyBuffers() {
    constexpr int latitudeCount = 16;
    constexpr int longitudeCount = 16;

    std::vector<Vertex> vertices;
    std::vector<unsigned int> indices;

    for (int lat = 0; lat <= latitudeCount; lat++) {
        float theta = lat * M_PI / latitudeCount;
        float sinTheta = sin(theta);
        float cosTheta = cos(theta);

        int row1 = lat * (longitudeCount + 1);
        int row2 = (lat + 1) * (longitudeCount + 1);

        for (int lon = 0; lon <= longitudeCount; lon++) {
            float phi = lon * 2.0f * M_PI / longitudeCount;
            float sinPhi = sin(phi);
            float cosPhi = cos(phi);

            Vertex vertex;
            vertex.x = cosPhi * sinTheta;
            vertex.y = cosTheta;
            vertex.z = sinPhi * sinTheta;
            vertex.nx = vertex.x;
            vertex.ny = vertex.y;
            vertex.nz = vertex.z;
            vertex.u = 1.0f - (float)lon / longitudeCount;
            vertex.v = 1.0f - (float)lat / latitudeCount;

            vertices.push_back(vertex);

            indices.push_back(row1 + lon);
            indices.push_back(row2 + lon);
            indices.push_back(row1 + lon + 1);
            indices.push_back(row1 + lon + 1);
            indices.push_back(row2 + lon);
            indices.push_back(row2 + lon + 1);
        }
    }

    nIndices = indices.size();

    glGenVertexArrays(1, &bodyVAO);
    glGenBuffers(1, &bodyVBO);
    glGenBuffers(1, &bodyEBO);

    glBindVertexArray(bodyVAO);

    glBindBuffer(GL_ARRAY_BUFFER, bodyVBO);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, bodyEBO);

    glBufferData(GL_ARRAY_BUFFER, sizeof(Vertex) * vertices.size(), vertices.data(), GL_STATIC_DRAW);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(unsigned int) * indices.size(), indices.data(), GL_STATIC_DRAW);

    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void *)offsetof(Vertex, x));
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void *)offsetof(Vertex, nx));
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void *)offsetof(Vertex, u));
    glEnableVertexAttribArray(2);

    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
    glBindVertexArray(0);
}

void initShaders() {
    int success;
    bodyShaderProgram = compileShaderProgram(EMBED_START_ASSETS_BODY_VERT_GLSL, EMBED_START_ASSETS_BODY_FRAG_GLSL, &success, "body");
    if (!success) {
        throw std::runtime_error("Failed to compile trajectory shader program");
    }

    glUseProgram(bodyShaderProgram);
    body_uPositionLoc = glGetUniformLocation(bodyShaderProgram, "uPosition");
    body_uModelLoc = glGetUniformLocation(bodyShaderProgram, "uModel");
    body_uViewLoc = glGetUniformLocation(bodyShaderProgram, "uView");
    body_uProjectionLoc = glGetUniformLocation(bodyShaderProgram, "uProjection");
    glUseProgram(0);
}

} // namespace

void initRenderBodySystem() {
    initBodyBuffers();
    initShaders();
}

void renderBodies(entt::registry &registry, entt::entity camera) {
    glEnable(GL_CULL_FACE);
    glCullFace(GL_BACK);
    glFrontFace(GL_CW);

    glUseProgram(bodyShaderProgram);
    glBindVertexArray(bodyVAO);
    glBindBuffer(GL_ARRAY_BUFFER, bodyVBO);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, bodyEBO);

    auto &cameraData = registry.get<Camera>(camera);
    glUniformMatrix4fv(body_uViewLoc, 1, GL_FALSE, cameraData.viewMatrix.data());
    glUniformMatrix4fv(body_uProjectionLoc, 1, GL_FALSE, cameraData.projectionMatrix.data());

    auto view = registry.view<physics::BodyState, physics::KeplerParameters, RenderBody>();
    for (auto entity : view) {
        auto &bodyState = view.get<physics::BodyState>(entity);
        auto &renderBody = view.get<RenderBody>(entity);
        Eigen::Vector3d pos = calculateAbsolutePosition(registry, bodyState);

        Eigen::Affine3f transform = Eigen::Affine3f::Identity();
        transform.translate(pos.cast<float>());
        transform.scale(renderBody.radius * 1e3f);
        Eigen::Matrix4f modelMatrix = transform.matrix();
        glUniform3f(body_uPositionLoc, pos.x(), pos.y(), pos.z());
        glUniformMatrix4fv(body_uModelLoc, 1, GL_FALSE, modelMatrix.data());

        // TODO: to avoid floating point error, we should calculate the position
        //  relative to the camera focus
        glDrawElements(GL_TRIANGLES, nIndices, GL_UNSIGNED_INT, 0);
    }

    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
    glBindVertexArray(0);
    glUseProgram(0);

    glDisable(GL_CULL_FACE);
}

} // namespace sfs::render
