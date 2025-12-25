#pragma once

#include <Eigen/Dense>
#include <entt/entt.hpp>

struct KeplerParameters {
    Eigen::Vector3d r0;
    Eigen::Vector3d v0;
    double sqrt_mu;
    double r0_norm;
    double alpha;
    double r_dot;
};

void recalculateAllKeplerParameters(entt::registry &registry);
void keplerPropagationSystem(entt::registry &registry, double dt);
