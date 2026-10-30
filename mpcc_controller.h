#pragma once
#include <vector>
#include <casadi/casadi.hpp>
#include "MPCC.h"
#include "nlp_solve.h"

// Wraps the whole MPCC so Python can use it:
//   constructor = everything that runs ONCE (spline, model, solver build)
//   step()      = everything that runs EVERY control period (one solve)
class MpccController {
public:
    MpccController(const std::vector<double>& waypoints_x,
                   const std::vector<double>& waypoints_y,
                   double dt,
                   int horizon,
                   const CostParam& cost_param,     // reward / penalty weights
                   const NlpBounds& bounds,         // input + state limits, track width
                   int speed_lookahead);            // which predicted step's vx is sent as the speed command

    // input : car state from MuJoCo [px, py, yaw, vx, vy, yaw_rate]   (6 numbers)
    // output: [steering_angle (rad), speed_command (m/s)]              (2 numbers)
    std::vector<double> step(const std::vector<double>& car_state);

    double get_theta() const;          // current progress along the track (m)
    double get_track_length() const;   // total centerline length (m)

private:
    // Arc-length position of the centerline point closest to (px, py).
    // Searches only a window of track around the current theta_, so it can never
    // jump to a different part of the circuit.
    double project_theta(double px, double py) const;

    // centerline points + their cumulative arc length, kept for project_theta()
    std::vector<double> wp_x_;
    std::vector<double> wp_y_;
    std::vector<double> wp_s_;
    double search_window_;   // how far along the track to search, in metres

    double dt_;
    int horizon_;
    int speed_lookahead_;
    double theta_;           // progress state, kept here so Python doesn't have to track it
    double track_length_;
    bool first_solve_;

    casadi::Function bicycle_;
    casadi::Function err_;
    NlpSolver solver_;
    casadi::DM w_guess_;     // last solution, reused as the warm start for the next solve
};
