#include "camera.h"

#include <cmath>

#include <Eigen/Dense>
#include <GLFW/glfw3.h>

#include "physics/physics.h"

namespace {

Eigen::Matrix4f lookAt(
    const Eigen::Vector3f& eye,
    const Eigen::Vector3f& center,
    const Eigen::Vector3f& up)
{
    Eigen::Vector3f f = (center - eye).normalized();
    Eigen::Vector3f s = f.cross(up).normalized();
    Eigen::Vector3f u = s.cross(f);

    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

    view(0,0) =  s.x();
    view(0,1) =  s.y();
    view(0,2) =  s.z();

    view(1,0) =  u.x();
    view(1,1) =  u.y();
    view(1,2) =  u.z();

    view(2,0) = -f.x();
    view(2,1) = -f.y();
    view(2,2) = -f.z();

    view(0,3) = -s.dot(eye);
    view(1,3) = -u.dot(eye);
    view(2,3) =  f.dot(eye);

    return view;
}

Eigen::Matrix4f perspective(
    float fovYRadians,
    float aspect,
    float zNear,
    float zFar)
{
    float tanHalfFovy = tanf(fovYRadians / 2.0f);

    Eigen::Matrix4f proj = Eigen::Matrix4f::Zero();
    proj(0,0) = 1.0f / (aspect * tanHalfFovy);
    proj(1,1) = 1.0f / tanHalfFovy;
    proj(2,2) = -(zFar + zNear) / (zFar - zNear);
    proj(2,3) = -(2.0f * zFar * zNear) / (zFar - zNear);
    proj(3,2) = -1.0f;
    return proj;
}

Eigen::Matrix4f infinitePerspective(float fovY, float aspect, float zNear)
{
    float f = 1.0f / tanf(fovY / 2.0f);

    Eigen::Matrix4f proj = Eigen::Matrix4f::Zero();
    proj(0,0) = f / aspect;
    proj(1,1) = f;
    proj(2,2) = -1.0f;
    proj(2,3) = -2.0f * zNear;
    proj(3,2) = -1.0f;
    return proj;
}

}  // namespace

double cameraDistance = 1.0e12;
int focusIndex = 0;

void scroll_callback(GLFWwindow* window, double xoffset, double yoffset) {
    cameraDistance = std::clamp(
        cameraDistance * std::pow(0.9, yoffset),
        1.0e9,
        1.0e14);
}

void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods) {
    if (action == GLFW_PRESS) {
        if (key == GLFW_KEY_LEFT_BRACKET) focusIndex--;
        if (key == GLFW_KEY_RIGHT_BRACKET) focusIndex++;
    }
}

void initCameraGLFWCallbacks(const MainWindow &window) {
    glfwSetScrollCallback(window.handle(), scroll_callback);
    glfwSetKeyCallback(window.handle(), key_callback);
}

void updateCameras(entt::registry &registry, const MainWindow &window, double dt) {
    // TODO: less sus focus cycling
    entt::entity focus = static_cast<entt::entity>((focusIndex % 6 + 6) % 6);

    constexpr double kRotationSpeed = 2.8; // radians per second
    double dyaw = 0.0;
    double dpitch = 0.0;
    if (glfwGetKey(window.handle(), GLFW_KEY_LEFT) == GLFW_PRESS) {
        dyaw = -kRotationSpeed;
    }
    if (glfwGetKey(window.handle(), GLFW_KEY_RIGHT) == GLFW_PRESS) {
        dyaw = kRotationSpeed;
    }
    if (glfwGetKey(window.handle(), GLFW_KEY_UP) == GLFW_PRESS) {
        dpitch = kRotationSpeed;
    }
    if (glfwGetKey(window.handle(), GLFW_KEY_DOWN) == GLFW_PRESS) {
        dpitch = -kRotationSpeed;
    }

    auto view = registry.view<Camera>();
    for (auto entity : view) {
        auto &camera = view.get<Camera>(entity);

        camera.target = focus;
        camera.distance = cameraDistance;
        camera.yaw = fmod(camera.yaw + dyaw * dt, 2.0 * M_PI);
        camera.pitch = std::clamp(camera.pitch + dpitch * dt, -0.49f * M_PI, 0.49f * M_PI);

        float aspect = static_cast<float>(window.width()) / static_cast<float>(window.height());
        camera.projectionMatrix = infinitePerspective(
            45.0f * static_cast<float>(M_PI) / 180.0f,
            aspect,
            0.1f);

        if (camera.target == entt::null) continue;

        auto &targetState = registry.get<BodyState>(camera.target);

        Eigen::Vector3f targetPos = targetState.st.pos.cast<float>();
        float x = camera.distance * cosf(camera.pitch) * sinf(camera.yaw);
        float y = camera.distance * sinf(camera.pitch);
        float z = camera.distance * cosf(camera.pitch) * cosf(camera.yaw);
        Eigen::Vector3f eye = targetPos + Eigen::Vector3f(x, y, z);
        Eigen::Vector3f center = targetPos;
        Eigen::Vector3f up(0.0f, 1.0f, 0.0f);

        camera.viewMatrix = lookAt(eye, center, up);
    }
}
