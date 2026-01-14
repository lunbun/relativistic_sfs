#pragma once

#include <vector>

#include <Eigen/Dense>
#include <entt/entt.hpp>

namespace sfs::physics {

struct KeplerParameters {
    Eigen::Vector3d r0;
    Eigen::Vector3d v0;
    double sqrt_mu;
    double r0_norm;
    double alpha;
    double r_dot;
    double e;
    double mu;
};

double calculatePeriapse(const KeplerParameters &p);
double calculateApoapse(const KeplerParameters &p);

void recalculateAllKeplerParameters(entt::registry &registry);
void keplerPropagationSystem(entt::registry &registry, double dt);

// NB: Caller must clear the `points` vector before calling
// NB: Sampled points are relative to the primary's position at the current time
void sampleTrajectoryPoints(const KeplerParameters &p, std::vector<Eigen::Vector3d> &points, int n);

} // namespace sfs::physics
