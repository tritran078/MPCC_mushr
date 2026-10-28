#pragma once
#include <casadi/casadi.hpp>

using namespace casadi;

// ---- fixed car constants (never symbolic) ----
const double gravity = 9.81;

struct CarParam {
    double wheelbase  = 0.2965;   // wheel base
    double track_w    = 0.230;    // track width
    double radius     = 0.05;     // radius wheel
    double mass       = 4.0;      // mass of car
    double height_COG = 0.0947;   // height of center of gravity
    double lf         = 0.1385;   // distance from CoM to front axle
    double lr         = 0.158;    // distance from CoM to rear axle
    double C_m        = 9.0;      // constant for motor model
    double I          = 0.04385;   // yaw moment of inertia
    // resistance forces (ESTIMATED, tune against MuJoCo coast-down test), added 2026-10-27
    double C_r        = 0.5;      // rolling resistance (N), constant, ~1.3% of m*g
    double C_d        = 0.05;     // aerodynamic drag coefficient (N per (m/s)^2)
};

struct PacejkaParam {
    double B = 5.0;   // fixed tire stiffness constant
    double C = 1.6;   // fixed tire shape constant
    // Df, Dr removed from the struct: they depend on load transfer,
    // which depends on state, so they're computed as SX locals, not stored here
};

// ---- the only function the QP solver needs ----
// state x = [px, py, yaw, vx, vy, yaw_rate, theta]   (7)
// input u = [PWM, steering, d_theta]                 (3)
// returns xdot as one stacked SX vector, wrapped as a CasADi Function
Function bicycle_model(const CarParam& car, const PacejkaParam& pac);
Function rk4_step(const Function& f, double dt);