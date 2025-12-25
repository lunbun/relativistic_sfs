#include "physics.h"

#include <cmath>

constexpr double G = 6.67430e-11;

// void gravitySystem(entt::registry &registry) {
//     auto view1 = registry.view<NumIntegrState, Body>();
//     auto view2 = registry.view<ForceAccumulator, NumIntegrState, Body>();
//
//     for (auto a : view1) {
//         auto &physA = view1.get<NumIntegrState>(a);
//         auto &bodyA = view1.get<Body>(a);
//         for (auto b : view2) {
//             if (a == b) continue;
//
//             auto &physB = view2.get<NumIntegrState>(b);
//             auto &bodyB = view2.get<Body>(b);
//             auto &forceAcc = view2.get<ForceAccumulator>(b);
//
//             Eigen::Vector3d r = physB.st.pos - physA.st.pos;
//             double norm = r.norm();
//             if (norm < 1e6) continue; // Avoid singularity
//
//             forceAcc.force -= G * bodyA.mass * bodyB.mass * r / (norm * norm * norm);
//         }
//     }
// }

double stumpff_C(double z) {
    if (z > 0) {
        return (1 - cos(sqrt(z))) / z;
    } else if (z < 0) {
        return (cosh(sqrt(-z)) - 1) / -z;
    } else {
        return 0.5;
    }
}

double stumpff_S(double z) {
    if (z > 0) {
        return (sqrt(z) - sin(sqrt(z))) / (z * sqrt(z));
    } else if (z < 0) {
        return (sinh(sqrt(-z)) - sqrt(-z)) / (-z * sqrt(-z));
    } else {
        return 1.0 / 6.0;
    }
}

void keplerPropagate(entt::view<entt::get_t<NumIntegrState, Body>> &view, entt::entity entity, double dt) {
    auto &state = view.get<NumIntegrState>(entity);
    auto &body = view.get<Body>(entity);

    if (body.primary == entt::null) return;
    auto &primaryState = view.get<NumIntegrState>(body.primary);
    auto &primaryBody = view.get<Body>(body.primary);

    Eigen::Vector3d r0 = state.st.pos - primaryState.st.pos;
    Eigen::Vector3d v0 = state.st.vel - primaryState.st.vel;

    double mu = G * primaryBody.mass;
    double alpha = 2.0 / r0.norm() - v0.squaredNorm() / mu;
    double r_dot = r0.dot(v0) / r0.norm();

    double chi = sqrt(mu) * alpha * dt;
    for (int i = 0; i < 5; i++) {
        double z = alpha * chi * chi;
        double F = r_dot / sqrt(mu) * chi * chi * stumpff_C(z) + (1.0 - alpha * r0.norm()) * chi * chi * chi * stumpff_S(z) + r0.norm() * chi - sqrt(mu) * dt;
        if (fabs(F) < 1e-6) break;
        double dF = r_dot / sqrt(mu) * chi * (1.0 - z * stumpff_S(z)) + (1.0 - alpha * r0.norm()) * chi * chi * stumpff_C(z) + r0.norm();
        chi = chi - F / dF;
    }

    double z = alpha * chi * chi;
    double f = 1 - chi * chi / r0.norm() * stumpff_C(z);
    double g = dt - chi * chi * chi / sqrt(mu) * stumpff_S(z);
    Eigen::Vector3d r = f * r0 + g * v0;

    double f_dot = sqrt(mu) / (r.norm() * r0.norm()) * chi * (z * stumpff_S(z) - 1.0);
    double g_dot = 1.0 - chi * chi / r.norm() * stumpff_C(z);
    Eigen::Vector3d v = f_dot * r0 + g_dot * v0;

    state.st.pos = primaryState.st.pos + r;
    state.st.vel = primaryState.st.vel + v;
}

void keplerPropagationSystem(entt::registry &registry, double dt) {
    auto view = registry.view<NumIntegrState, Body>();
    for (auto entity : view) {
        keplerPropagate(view, entity, dt);
    }
}

void physicsUpdate(entt::registry &registry, double dt) {
    // auto forcesView = registry.view<ForceAccumulator, NumIntegrState, Body>();
    // for (auto entity : forcesView) {
    //     auto &forceAcc = forcesView.get<ForceAccumulator>(entity);
    //     forceAcc.force.setZero();
    // }

    // gravitySystem(registry);
    keplerPropagationSystem(registry, dt);

    // TODO: kick propagation system
    // for (auto entity : forcesView) {
    //     auto &forceAcc = forcesView.get<ForceAccumulator>(entity);
    //     auto &physics = forcesView.get<NumIntegrState>(entity);
    //     auto &body = forcesView.get<Body>(entity);
    //     physics.st.vel += dt * forceAcc.force / body.mass;
    //     physics.st.pos += dt * physics.st.vel;
    // }
}


