#include <chrono>
#include <iostream>
#include <random>

#include <Eigen/Dense>
#include <entt/entt.hpp>
#include <imgui.h>

#include "model/solar_system.h"
#include "physics/kepler.h"
#include "physics/physics.h"
#include "render/body.h"
#include "render/window.h"
#include "util.h"

int main() {
    std::cout << "Hello, World!" << std::endl;

    std::unique_ptr<MainWindow> window = MainWindow::create();

    entt::registry registry;
    createSolarSystem(registry);

    double time = 0.0;

    // Game loop
    while (!window->shouldClose()) {
        window->startFrame();

        constexpr double dt = 36000.0;
        physicsUpdate(registry, dt);
        renderBodies(registry);
        renderTrajectories(registry);

        time += dt;
        std::string formattedTime = formatDuration(std::chrono::seconds(static_cast<long long>(time)));
        ImGui::Text("Simulated time: %s", formattedTime.c_str());

        window->endFrame();
    }
    return 0;
}
