#include <chrono>
#include <iostream>
#include <random>

#include <Eigen/Dense>
#include <entt/entt.hpp>
#include <imgui.h>
#include <render/camera.h>

#include "model/solar_system.h"
#include "physics/kepler.h"
#include "physics/physics.h"
#include "render/body.h"
#include "render/window.h"
#include "util.h"

int main() {
    std::cout << "Hello, World!" << std::endl;

    entt::registry registry;
    std::unique_ptr<MainWindow> window = MainWindow::create();

    createSolarSystem(registry);

    auto camera = registry.create();
    {
        auto &cameraData = registry.emplace<Camera>(camera);
        cameraData.target = entt::null;
        cameraData.distance = 1.0e12;
        cameraData.yaw = 0.0;
        cameraData.pitch = M_PI / 6.0;
    }

    initCameraGLFWCallbacks(*window);
    window->initImGui();

    double time = 0.0;
    initRenderBodySystem();

    // Game loop
    while (!window->shouldClose()) {
        window->startFrame();

        constexpr double dt = 36000.0;
        physicsUpdate(registry, dt);
        updateCameras(registry, *window, 1.0 / 144.0);
        renderBodies(registry, camera);
        renderTrajectories(registry, camera);

        time += dt;
        std::string formattedTime = formatDuration(std::chrono::seconds(static_cast<long long>(time)));
        ImGui::Text("Simulated time: %s", formattedTime.c_str());

        window->endFrame();
    }
    return 0;
}
