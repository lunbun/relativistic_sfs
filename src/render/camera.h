#pragma once

#include <Eigen/Dense>
#include <entt/entt.hpp>

#include "render/window.h"

struct Camera {
    entt::entity target;
    double distance;
    double yaw;  // radians
    double pitch;  // radians
    Eigen::Matrix4f viewMatrix;
    Eigen::Matrix4f projectionMatrix;
};

void initCameraGLFWCallbacks(const MainWindow &window);
void updateCameras(entt::registry &registry, const MainWindow &window, double dt);
