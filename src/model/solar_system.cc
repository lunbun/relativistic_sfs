#include "solar_system.h"

#include <random>

#include <entt/entt.hpp>

#include "physics/kepler.h"
#include "physics/physics.h"
#include "render/body.h"

void createSolarSystem(entt::registry &registry) {
    auto sun = registry.create();
    registry.emplace<BodyState>(sun, Eigen::Vector3d(0, 0, 0), Eigen::Vector3d::Zero());
    registry.emplace<Body>(sun, entt::null, 1.989e30);
    registry.emplace<RenderDot>(sun, 0.02f);

    auto mercury = registry.create();
    registry.emplace<BodyState>(mercury, Eigen::Vector3d(57.9e9, 0, 0), Eigen::Vector3d(0, 0, 47870));
    registry.emplace<Body>(mercury, sun, 3.3011e23);
    registry.emplace<RenderDot>(mercury, 0.02f);
    registry.emplace<RenderTrajectory>(mercury);

    auto venus = registry.create();
    registry.emplace<BodyState>(venus, Eigen::Vector3d(108.2e9, 0, 0), Eigen::Vector3d(0, 0, 35020));
    registry.emplace<Body>(venus, sun, 4.8675e24);
    registry.emplace<RenderDot>(venus, 0.02f);
    registry.emplace<RenderTrajectory>(venus);

    auto earth = registry.create();
    registry.emplace<BodyState>(earth, Eigen::Vector3d(149.6e9, 0, 0), Eigen::Vector3d(0, 0, 29784.8));
    registry.emplace<Body>(earth, sun, 5.972e24);
    registry.emplace<RenderDot>(earth, 0.02f);
    registry.emplace<RenderTrajectory>(earth);

    auto mars = registry.create();
    registry.emplace<BodyState>(mars, Eigen::Vector3d(227.9e9, 0, 0), Eigen::Vector3d(0, 0, 24077));
    registry.emplace<Body>(mars, sun, 6.4171e23);
    registry.emplace<RenderDot>(mars, 0.02f);
    registry.emplace<RenderTrajectory>(mars);

    auto jupiter = registry.create();
    registry.emplace<BodyState>(jupiter, Eigen::Vector3d(778.5e9, 0, 0), Eigen::Vector3d(0, 0, 13070));
    registry.emplace<Body>(jupiter, sun, 1.8982e27);
    registry.emplace<RenderDot>(jupiter, 0.02f);
    registry.emplace<RenderTrajectory>(jupiter);

    // Create asteroids
    std::default_random_engine generator;
    std::uniform_real_distribution<double> distanceDistribution(300.0e9, 500.0e9);
    std::uniform_real_distribution<double> speedDistribution(15000.0, 25000.0);
    std::uniform_real_distribution<double> angleDistribution(0.0, 2.0 * M_PI);
    std::uniform_real_distribution<double> massDistribution(1.0e15, 1.0e20);
    for (int i = 0; i < 100; i++) {
        double distance = distanceDistribution(generator);
        double speed = speedDistribution(generator);
        double angle = angleDistribution(generator);
        double mass = massDistribution(generator);

        Eigen::Vector3d position(distance * cos(angle), 0, distance * sin(angle));
        Eigen::Vector3d velocity(-speed * sin(angle), 0, speed * cos(angle));

        auto asteroid = registry.create();
        registry.emplace<BodyState>(asteroid, position, velocity);
        registry.emplace<Body>(asteroid, sun, mass);
        registry.emplace<RenderDot>(asteroid, 0.007f);
        // registry.emplace<RenderTrajectory>(asteroid);
    }

    for (auto entity : registry.view<BodyState>()) {
        registry.emplace<KeplerParameters>(entity);
        if (entity != sun) registry.emplace<ForceAccumulator>(entity);
    }

    recalculateAllKeplerParameters(registry);
}
