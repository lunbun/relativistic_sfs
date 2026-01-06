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

    double initialEnergy;
    Eigen::Vector3d initialCOM, initialMomentum, initialAngularMomentum;
    calculateConservedQuantities(registry, initialCOM, initialEnergy, initialMomentum, initialAngularMomentum);

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

        double energy;
        Eigen::Vector3d com, momentum, angularMomentum;
        calculateConservedQuantities(registry, com, energy, momentum, angularMomentum);
        ImGui::Text("Center of Mass: [%.3e, %.3e, %.3e] m", com.x(), com.y(), com.z());
        ImGui::Text("Total Energy: %.3e J", energy);
        ImGui::Text("Total Momentum: [%.3e, %.3e, %.3e] kg·m/s", momentum.x(), momentum.y(), momentum.z());
        ImGui::Text("Total Angular Momentum: [%.3e, %.3e, %.3e] kg·m²/s", angularMomentum.x(), angularMomentum.y(), angularMomentum.z());

        ImGui::Text("Energy Drift: %.3e J", energy - initialEnergy);
        ImGui::Text("Momentum Drift: [%.3e, %.3e, %.3e] kg·m/s",
            momentum.x() - initialMomentum.x(),
            momentum.y() - initialMomentum.y(),
            momentum.z() - initialMomentum.z()
        );
        ImGui::Text("Angular Momentum Drift: [%.3e, %.3e, %.3e] kg·m²/s",
            angularMomentum.x() - initialAngularMomentum.x(),
            angularMomentum.y() - initialAngularMomentum.y(),
            angularMomentum.z() - initialAngularMomentum.z()
        );

        entt::entity sun = static_cast<entt::entity>(0);
        ImGui::Text("Sun Velocity: [%.3e, %.3e, %.3e] m",
            registry.get<BodyState>(sun).st.vel.x(),
            registry.get<BodyState>(sun).st.vel.y(),
            registry.get<BodyState>(sun).st.vel.z()
        );

        window->endFrame();
    }
    return 0;
}
