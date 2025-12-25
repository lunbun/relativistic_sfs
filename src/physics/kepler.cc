#include "kepler.h"

#include <cmath>

#include "physics/physics.h"

namespace {

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

double solveUniversalKeplerEquation(const KeplerParameters &p, double dt, double *out_C, double *out_S) {
    // Newton-Raphson iteration to solve for chi
    double chi = p.sqrt_mu * p.alpha * dt;
    for (int i = 0; i < 5; i++) {
        double z = p.alpha * chi * chi;
        double C = stumpff_C(z);
        double S = stumpff_S(z);
        double F = p.r0_norm * p.r_dot / p.sqrt_mu * chi * chi * C + (1.0 - p.alpha * p.r0_norm) * chi * chi * chi * S + p.r0_norm * chi - p.sqrt_mu * dt;
        if (fabs(F) < 1e-7) {
            if (out_C) *out_C = C;
            if (out_S) *out_S = S;
            return chi;
        }
        double dF = p.r0_norm * p.r_dot / p.sqrt_mu * chi * (1.0 - z * S) + (1.0 - p.alpha * p.r0_norm) * chi * chi * C + p.r0_norm;
        chi = chi - F / dF;
    }

    double z = p.alpha * chi * chi;
    if (out_C) *out_C = stumpff_C(z);
    if (out_S) *out_S = stumpff_S(z);
    return chi;
}

void keplerPropagate(double chi, const KeplerParameters &p, double C, double S, double dt, Eigen::Vector3d &r, Eigen::Vector3d *v) {
    double z = p.alpha * chi * chi;
    double f = 1 - chi * chi / p.r0_norm * C;
    double g = dt - chi * chi * chi / p.sqrt_mu * S;
    r = f * p.r0 + g * p.v0;

    if (v) {
        double r_norm = r.norm();
        double f_dot = p.sqrt_mu / (r_norm * p.r0_norm) * chi * (z * S - 1.0);
        double g_dot = 1.0 - chi * chi / r_norm * C;
        *v = f_dot * p.r0 + g_dot * p.v0;
    }
}

// Kepler propagation with unknown dt
void keplerPropagateUnknownTime(double chi, const KeplerParameters &p, Eigen::Vector3d &r, Eigen::Vector3d *v) {
    double z = p.alpha * chi * chi;
    double C = stumpff_C(z);
    double S = stumpff_S(z);
    // TODO: possibly can be optimized by avoiding recalculation of dt
    double dt = (p.r0_norm * p.r_dot / p.sqrt_mu * chi * chi * C + (1.0 - p.alpha * p.r0_norm) * chi * chi * chi * S + p.r0_norm * chi) / p.sqrt_mu;
    keplerPropagate(chi, p, C, S, dt, r, v);
}

} // namespace

// TODO: Technically, we do not have to recalculate parameters if no external forces are applied
//  It is also better to avoid recalculation if no external forces occur since
//  numerical errors can accumulate in alpha (the specific orbital energy) over
//  time otherwise.
void recalculateAllKeplerParameters(entt::registry &registry) {
    auto view = registry.view<NumIntegrState, Body, KeplerParameters>();
    for (auto entity : view) {
        auto &body = view.get<Body>(entity);
        if (body.primary == entt::null) continue;

        auto &state = view.get<NumIntegrState>(entity);
        auto &primaryState = registry.get<NumIntegrState>(body.primary);
        auto &primaryBody = registry.get<Body>(body.primary);

        Eigen::Vector3d r0 = state.st.pos - primaryState.st.pos;
        Eigen::Vector3d v0 = state.st.vel - primaryState.st.vel;

        double mu = kGravitationalConstant * primaryBody.mass;
        double r0_norm = r0.norm();
        double alpha = 2.0 / r0_norm - v0.squaredNorm() / mu;
        double r_dot = r0.dot(v0) / r0_norm;

        view.get<KeplerParameters>(entity) =
                KeplerParameters{ .r0 = r0, .v0 = v0, .sqrt_mu = sqrt(mu), .r0_norm = r0_norm, .alpha = alpha, .r_dot = r_dot };
    }
}

void keplerPropagationSystem(entt::registry &registry, double dt) {
    auto view = registry.view<NumIntegrState, Body, KeplerParameters>();
    for (auto entity : view) {
        auto &body = view.get<Body>(entity);
        if (body.primary == entt::null) continue;

        auto &state = view.get<NumIntegrState>(entity);
        auto &p = view.get<KeplerParameters>(entity);
        auto &primaryState = registry.get<NumIntegrState>(body.primary);

        double C, S;
        double chi = solveUniversalKeplerEquation(p, dt, &C, &S);

        Eigen::Vector3d r, v;
        keplerPropagate(chi, p, C, S, dt, r, &v);

        state.st.pos = primaryState.st.pos + r;
        state.st.vel = primaryState.st.vel + v;
    }
}

void sampleTrajectoryPoints(const KeplerParameters &p, std::vector<Eigen::Vector3d> &points, int n) {
    double chi_min = solveUniversalKeplerEquation(p, 0.0, nullptr, nullptr);
    double chi_max = chi_min;
    if (p.alpha > 0) {  // Elliptical orbit
        chi_max = chi_min + 2.0 * M_PI / sqrt(p.alpha);
    } else {  // Hyperbolic or parabolic orbit
        // For hyperbolic/parabolic orbits, chi goes to +inf. We set an arbitrary bound
        // for sampling purposes.
        chi_max = 1.0e7;
    }

    double step = (chi_max - chi_min) / (n - 1);
    for (int i = 0; i < n; i++) {
        double chi = chi_min + i * step;
        Eigen::Vector3d r;
        keplerPropagateUnknownTime(chi, p, r, nullptr);
        points.push_back(r);
    }
}
