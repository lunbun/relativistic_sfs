#include "kepler.h"

#include <iostream>
#include <cmath>
#include <cassert>

#include "physics/physics.h"

namespace {

double stumpff_C(double z) {
    if (z > 1e-7) {
        return (1 - cos(sqrt(z))) / z;
    } else if (z < -1e-7) {
        return (cosh(sqrt(-z)) - 1) / -z;
    } else {
        return 0.5 - z / 24 + z * z / 720;
    }
}

double stumpff_S(double z) {
    if (z > 1e-7) {
        return (sqrt(z) - sin(sqrt(z))) / (z * sqrt(z));
    } else if (z < -1e-7) {
        return (sinh(sqrt(-z)) - sqrt(-z)) / (-z * sqrt(-z));
    } else {
        return 1.0 / 6.0 - z / 120 + z * z / 5040;
    }
}

double evaluateUniversalKepler(const KeplerParameters &p, double chi, double C, double S) {
    return p.r0_norm * p.r_dot / p.sqrt_mu * chi * chi * C + (1.0 - p.alpha * p.r0_norm) * chi * chi * chi * S + p.r0_norm * chi;
}

double evaluateUniversalKeplerDerivChi(const KeplerParameters &p, double chi, double z, double C, double S) {
    return p.r0_norm * p.r_dot / p.sqrt_mu * chi * (1.0 - z * S) + (1.0 - p.alpha * p.r0_norm) * chi * chi * C + p.r0_norm;
}

double evaluateUniversalKeplerSecondDerivChi(const KeplerParameters &p, double chi, double z, double C, double S) {
    return p.r0_norm * p.r_dot / p.sqrt_mu * (1.0 - z * C) + (1.0 - p.alpha * p.r0_norm) * chi * (1.0 - z * S);
}

// Newton-Raphson iteration to solve for chi
double solveUniversalKeplerEquation(const KeplerParameters &p, double dt, double *out_C, double *out_S) {
    double r_peri = calculatePeriapse(p);
    double r_apo = calculateApoapse(p);
    double chi_max = p.sqrt_mu * dt / r_peri;
    double chi_min = p.alpha > 0 ? p.sqrt_mu * dt / r_apo : 0.0;
    double chi;
    if (fabs(p.e - 1.0) < 0.01) {
        // For roughly parabolic trajectories, we obtain a better estimate by exactly solving
        // Barker's equation.
        // TODO: optimize this
        double h = p.r0.cross(p.v0).norm();
        double M_p = p.mu * p.mu * dt / (h * h * h);
        double z = cbrt(3 * M_p + sqrt(1 + 9 * M_p * M_p));
        double D = z - 1.0 / z;
        chi = h / p.sqrt_mu * D;
    } else {
        double z = p.alpha * chi_max * chi_max;
        chi = p.mu * dt * dt / (r_peri * evaluateUniversalKepler(p, chi_max, stumpff_C(z), stumpff_S(z)));
    }
    if (dt < 0.0) std::swap(chi_min, chi_max);
    if (std::isnan(chi)) chi = chi_min;
    chi = std::clamp(chi, chi_min, chi_max);

    int i = 0;
    for (; i < 30; i++) {
        constexpr int n = 5;
        double z = p.alpha * chi * chi;
        double C = stumpff_C(z);
        double S = stumpff_S(z);
        double F = evaluateUniversalKepler(p, chi, C, S) - p.sqrt_mu * dt;
        if (fabs(F / p.sqrt_mu) < 1e-12) {
            if (out_C) *out_C = C;
            if (out_S) *out_S = S;
            return chi;
        }
        double dF = evaluateUniversalKeplerDerivChi(p, chi, z, C, S);
        double d2F = evaluateUniversalKeplerSecondDerivChi(p, chi, z, C, S);
        double D = (n - 1) * (n - 1) * dF * dF - n * (n - 1) * F * d2F;
        double delta;
        if (D > 1e-7) {  // Laguerre method
            delta = n * F / (dF + std::copysign(sqrt(D), dF));
        } else {  // Fallback to Newton-Raphson
            delta = F / dF;
        }
        chi = std::clamp(chi - delta, chi_min, chi_max);  // Prevent overshoot
        if (fabs(delta / std::max(1.0, fabs(chi))) < 1e-12) {
            break;
        }
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

double calculatePeriapse(const KeplerParameters &p) {
    if (p.alpha > 0) {
        return (1.0 - p.e) / p.alpha;
    } else if (p.alpha < 0) {
        return (p.e - 1.0) / -p.alpha;
    } else {
        // Parabolic orbit
        return p.r0.cross(p.v0).squaredNorm() / (2.0 * p.sqrt_mu * p.sqrt_mu);
    }
}

double calculateApoapse(const KeplerParameters &p) {
    if (p.alpha > 0) {
        return (1.0 + p.e) / p.alpha;
    } else {
        return INFINITY;  // No apoapse for hyperbolic or parabolic orbits
    }
}

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
        double e = sqrt(1 - r0.cross(v0).squaredNorm() * alpha / mu);

        view.get<KeplerParameters>(entity) =
                KeplerParameters{ .r0 = r0, .v0 = v0, .sqrt_mu = sqrt(mu), .r0_norm = r0_norm, .alpha = alpha, .r_dot = r_dot, .e = e, .mu = mu };
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
    double chi_max;
    if (p.alpha > 0) {  // Elliptical orbit
        chi_max = 2.0 * M_PI / sqrt(p.alpha);
    } else {  // Hyperbolic or parabolic orbit
        // For hyperbolic/parabolic orbits, chi goes to +inf. We set an arbitrary bound
        // for sampling purposes.
        chi_max = 1.0e7;
    }

    double step = chi_max / (n - 1);
    for (int i = 0; i < n; i++) {
        double chi = i * step;
        Eigen::Vector3d r;
        keplerPropagateUnknownTime(chi, p, r, nullptr);
        points.push_back(r);
    }
}
