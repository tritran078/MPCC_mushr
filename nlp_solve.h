#pragma once
#include <functional>
#include <limits>
#include <casadi/casadi.hpp>

struct NlpBounds {
    double pwm_min    = -5.0, pwm_max    = 5.0;   // pwm_min < 0 = braking allowed (was 0.0, changed 2026-10-27)
    double steer_max  = 0.4;
    double dtheta_min = 0.0,  dtheta_max = 3.0;
    double vx_min     = 0.1;   // keep > 0: slip angle divides by vx
    double vx_max     = std::numeric_limits<double>::infinity();   // no upper speed limit (was 4.0, removed 2026-10-27)
    double theta_max  = 1.0;   // MUST set to total path length

    // track boundary: |contour error| <= track_half_width - car_margin
    double track_half_width = 1.1;   // centerline to wall (m), Budapest track is ~2.2 m wide
    double car_margin       = 0.25;  // car half-width (~0.14 m) + safety buffer (m)
};

struct NlpSolver {
    casadi::Function solver;
    int N = 0;
    casadi::DM lbw, ubw;   // bounds on decision vector
    casadi::DM lbg, ubg;   // constraint bounds (all 0)
};

using CostFn = std::function<casadi::MX(casadi::MX& X, casadi::MX& U)>;

NlpSolver build_nlp(const casadi::Function& F, int N, const CostFn& cost, const NlpBounds& bounds,
                    const casadi::Function& err);
casadi::DM nlp_initial_guess(const NlpSolver& s, const casadi::DM& x0);
casadi::DM solve_nlp(const NlpSolver& s, const casadi::DM& x0, casadi::DM& w_guess);