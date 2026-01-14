#pragma once

#include <Eigen/Dense>
#include <entt/entt.hpp>

namespace sfs::physics {

constexpr double kGravitationalConstant = 6.67430e-11;

struct PhysicsState {
    entt::entity primary;   // Primary body for orbiting, can be entt::null
    Eigen::Vector3d pos;    // Position relative to primary
    Eigen::Vector3d vel;    // Velocity relative to primary
};

struct ForceAccumulator {
    Eigen::Vector3d force;
};

struct Body {
    double mass;
};

struct BodyState { PhysicsState st; };

void physicsUpdate(entt::registry &registry, double dt);

void calculateConservedQuantities(entt::registry &registry, Eigen::Vector3d &com, double &energy, Eigen::Vector3d &momentum, Eigen::Vector3d &angularMomentum);

template<typename T>
bool isParentBody(entt::registry &registry, entt::entity child, entt::entity parent) {
    while (child != entt::null) {
        const auto &state = registry.get<T>(child);
        if (state.st.primary == parent) return true;
        child = state.st.primary;
    }
    return false;
}

template<typename T>
Eigen::Vector3d calculateAbsolutePosition(entt::registry &registry, const T &state) {
    Eigen::Vector3d pos = state.st.pos;
    entt::entity cur = state.st.primary;
    while (cur != entt::null) {
        const auto &parent = registry.get<T>(cur);
        pos += parent.st.pos;
        cur = parent.st.primary;
    }
    return pos;
}

template<typename T>
Eigen::Vector3d calculateAbsoluteVelocity(entt::registry &registry, const T &state) {
    Eigen::Vector3d pos = state.st.vel;
    entt::entity cur = state.st.primary;
    while (cur != entt::null) {
        const auto &parent = registry.get<T>(cur);
        pos += parent.st.vel;
        cur = parent.st.primary;
    }
    return pos;
}

template<typename From, typename To>
void syncState(entt::registry &registry) {
    auto view = registry.view<From, To>();
    for (auto entity : view) {
        auto &from = view.template get<From>(entity);
        auto &to = view.template get<To>(entity);
        to.st = from.st;
    }
}

} // namespace sfs::physics
