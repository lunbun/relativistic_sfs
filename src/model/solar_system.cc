#include "solar_system.h"

#include <random>

#include <entt/entt.hpp>

#include "physics/kepler.h"
#include "physics/physics.h"
#include "render/scene/body.h"
#include "render/scene/trajectory.h"

namespace {

// position in km, velocity in km/s, mass in kg
entt::entity createBodyFromJPL(entt::registry &registry,
                             const Eigen::Vector3d &position,
                             const Eigen::Vector3d &velocity,
                             double mass,
                             entt::entity parent) {
    // Swap y and z components
    Eigen::Vector3d position2(position.x(), position.z(), position.y());
    Eigen::Vector3d velocity2(velocity.x(), velocity.z(), velocity.y());
    position2 *= 1e3;
    velocity2 *= 1e3;
    if (parent != entt::null) {
        const auto &parentState = registry.get<BodyState>(parent);
        position2 -= calculateAbsolutePosition(registry, parentState);
        velocity2 -= calculateAbsoluteVelocity(registry, parentState);
    }

    auto planet = registry.create();
    registry.emplace<BodyState>(planet, parent, position2, velocity2);
    registry.emplace<Body>(planet, mass);
    registry.emplace<RenderDot>(planet, 0.02f);
    if (parent != entt::null) registry.emplace<RenderTrajectory>(planet);
    return planet;
}

} // namespace

void createSolarSystem(entt::registry &registry) {
    // Data from January 1, 2025, 00:00 UTC
    auto sun = createBodyFromJPL(
        registry,
        Eigen::Vector3d(-8.572865039469250e+05, -7.346088835335051e+05, 2.685423265526889e+04),
        Eigen::Vector3d(1.239859639798033e-02, -6.348466611140617e-03, -2.037876555553517e-04),
        1.98841e30,
        entt::null
    );

    auto mercury = createBodyFromJPL(
        registry,
        Eigen::Vector3d(-5.879699189509091e+07, -2.492820404148239e+07, 3.364042452841429e+06),
        Eigen::Vector3d(8.711982611106873e+00, -4.284856986770977e+01, -4.299279282370732e+00),
        3.302e23,
        sun
    );
    auto venus = createBodyFromJPL(
        registry,
        Eigen::Vector3d(6.697319534635594e+07, 8.337171945245868e+07, -2.731933993919346e+06),
        Eigen::Vector3d(-2.735548307021769e+01, 2.182743070706988e+01, 1.878804135388283e+00),
        4.8685e24,
        sun
    );
    auto earth = createBodyFromJPL(
        registry,
        Eigen::Vector3d(-2.758794880287251e+07, 1.439239583084676e+08, 1.921064327326417e+04),
        Eigen::Vector3d(-2.977686364628585e+01, -5.535813340802556e+00, -1.943387942073826e-04),
        5.97219e24,
        sun
    );
    // TODO: moon will need tidal forces
    // auto moon = createBodyFromJPL(
    //     registry,
    //     Eigen::Vector3d(-2.743589644230116e+07, 1.435751546419631e+08, -1.145344989768416e+04),
    //     Eigen::Vector3d(-2.884424012475249e+01, -5.089320873036412e+00, 3.814177365090332e-02),
    //     7.349e22,
    //     earth
    // );
    auto mars = createBodyFromJPL(
        registry,
        Eigen::Vector3d(-7.890038131682469e+07, 2.274372361241295e+08, 6.722196400986686e+06),
        Eigen::Vector3d(-2.199759485544177e+01, -5.787405095472254e+00, 4.184257990340883e-01),
        6.4171e23,
        sun
    );
    auto jupiter = createBodyFromJPL(
        registry,
        Eigen::Vector3d(1.571230833020991e+08, 7.429840488421507e+08, -6.597049828231782e+06),
        Eigen::Vector3d(-1.293244436609816e+01, 3.325781476287804e+00, 2.755437569190042e-01),
        1.89819e27,
        sun
    );
    auto saturn = createBodyFromJPL(
        registry,
        Eigen::Vector3d(1.414498231862034e+09, -2.647172137275474e+08, -5.171551879510410e+07),
        Eigen::Vector3d(1.240660798615463e+00, 9.473546595187154e+00, -2.135791731559418e-01),
        5.6834e26,
        sun
    );
    auto uranus = createBodyFromJPL(
        registry,
        Eigen::Vector3d(1.660221941282016e+09, 2.406965914394302e+09, -1.256903703858864e+07),
        Eigen::Vector3d(-5.655920544550284e+00, 3.549247198028906e+00, 8.651776992957205e-02),
        8.6813e25,
        sun
    );
    auto neptune = createBodyFromJPL(
        registry,
        Eigen::Vector3d(4.469116222588663e+09, -9.560778256566879e+07, -1.010264767638457e+08),
        Eigen::Vector3d(8.064561683471368e-02, 5.465730017544922e+00, -1.151205185674022e-01),
        1.02409e26,
        sun
    );
    auto pluto = createBodyFromJPL(
        registry,
        Eigen::Vector3d(2.726134821346495e+09, -4.489869448498899e+09, -3.081172013616135e+08),
        Eigen::Vector3d(4.789449666460403e+00, 1.631136121618671e+00, -1.537878641005477e+00),
        1.307e22,
        sun
    );

    // Create asteroids
    // std::default_random_engine generator;
    // std::uniform_real_distribution<double> distanceDistribution(300.0e9, 500.0e9);
    // std::uniform_real_distribution<double> speedDistribution(15000.0, 25000.0);
    // std::uniform_real_distribution<double> angleDistribution(0.0, 2.0 * M_PI);
    // std::uniform_real_distribution<double> massDistribution(1.0e15, 1.0e20);
    // for (int i = 0; i < 100; i++) {
    //     double distance = distanceDistribution(generator);
    //     double speed = speedDistribution(generator);
    //     double angle = angleDistribution(generator);
    //     double mass = massDistribution(generator);
    //
    //     Eigen::Vector3d position(distance * cos(angle), 0, distance * sin(angle));
    //     Eigen::Vector3d velocity(-speed * sin(angle), 0, speed * cos(angle));
    //
    //     auto asteroid = registry.create();
    //     registry.emplace<BodyState>(asteroid, sun, position, velocity);
    //     registry.emplace<Body>(asteroid, mass);
    //     registry.emplace<RenderDot>(asteroid, 0.007f);
    //     // registry.emplace<RenderTrajectory>(asteroid);
    // }

    for (auto entity : registry.view<BodyState>()) {
        if (entity != sun) registry.emplace<KeplerParameters>(entity);
        registry.emplace<ForceAccumulator>(entity);
    }

    recalculateAllKeplerParameters(registry);
}
