#include "mpcc_controller.h"
#include <algorithm>
#include <limits>
#include <cmath>
using namespace std;
using namespace casadi;

MpccController::MpccController(const vector<double>& waypoints_x,
                               const vector<double>& waypoints_y,
                               double dt,
                               int horizon,
                               const CostParam& cost_param,
                               const NlpBounds& bounds_in,
                               int speed_lookahead)
    : dt_(dt), horizon_(horizon), theta_(0.0), first_solve_(true)
{
    // clamp look-ahead to a valid step: 1 .. horizon
    speed_lookahead_ = speed_lookahead;
    if (speed_lookahead_ < 1)        speed_lookahead_ = 1;
    if (speed_lookahead_ > horizon_) speed_lookahead_ = horizon_;

    // cumulative_distance() expects one flat list: x0, y0, x1, y1, ...
    vector<float> waypoints_flat;
    for (size_t i = 0; i < waypoints_x.size(); i++) {
        waypoints_flat.push_back(static_cast<float>(waypoints_x[i]));
        waypoints_flat.push_back(static_cast<float>(waypoints_y[i]));
    }
    vector<double> distance = cumulative_distance(waypoints_flat);
    track_length_ = distance.back();

    // keep the centerline so step() can project the car's real position onto it
    wp_x_ = waypoints_x;
    wp_y_ = waypoints_y;
    wp_s_ = distance;
    search_window_ = 3.0;   // metres of track searched either side of the current theta

    // vehicle model, discretized with RK4
    CarParam car_param;
    PacejkaParam pac;
    bicycle_ = rk4_step(bicycle_model(car_param, pac), dt_);

    // contour / lag error function built from the spline
    err_ = contour_lag_error(distance, waypoints_x, waypoints_y);

    // cost function (same tuning as main_mpcc.cpp)
    // cost weights now come from Python (see CostParam defaults in MPCC.h)
    int N = horizon_;
    double step_dt = dt_;
    Function err_copy = err_;
    CostFn cost = [N, step_dt, cost_param, err_copy](MX& X, MX& U) {
        return cost_function(N, static_cast<float>(step_dt), cost_param, X, U, err_copy);
    };

    // bounds + solver (built once, reused every step)
    NlpBounds bounds = bounds_in;
    bounds.theta_max = track_length_;     // always the real track length, ignore the Python value
    solver_ = build_nlp(bicycle_, horizon_, cost, bounds, err_);
}

vector<double> MpccController::step(const vector<double>& car_state) {
    // Progress theta comes from the car's MEASURED position, projected onto the
    // centerline. It used to be integrated from the previous plan, which meant it
    // drifted away from where the car actually was -- and because the contour/lag
    // errors and the track-boundary constraint are all evaluated at theta, they
    // ended up measured against the wrong point on the spline.
    theta_ = project_theta(car_state[0], car_state[1]);

    // full 7-state = MuJoCo's 6 measured states + the projected theta
    DM x0 = DM::vertcat({car_state[0], car_state[1], car_state[2],
                         car_state[3], car_state[4], car_state[5],
                         theta_});

    if (first_solve_) {
        w_guess_ = nlp_initial_guess(solver_, x0);
        first_solve_ = false;
    }

    DM u0 = solve_nlp(solver_, x0, w_guess_);   // also updates w_guess_ with the new solution

    // decision vector layout: [X(:,0), X(:,1), ..., X(:,N), U(:,0), ...], 7 states per column
    const int nx = 7;
    // speed command = predicted vx at k = speed_lookahead_ (set from Python, default 5)
    const int index_vx_next    = nx * speed_lookahead_ + 3;

    double steering_cmd = static_cast<double>(u0(1));
    double speed_cmd    = static_cast<double>(w_guess_(index_vx_next));
    // theta is NOT taken from the solution any more -- it is re-projected at the
    // top of the next step(), so a bad solve can no longer corrupt it.

    return {steering_cmd, speed_cmd};
}

// Finds where the car is along the centerline, in metres of arc length.
//
// The search is limited to a window measured in ARC LENGTH around the current
// theta_, not in straight-line distance. That matters: on a circuit that runs
// back alongside itself, a nearest-point search over the whole track could snap
// to a section that is physically close but far away along the path. A window in
// arc length is always one continuous piece of track, so that cannot happen.
//
// Assumes the car starts near the first waypoint (sim_test.py places it there).
double MpccController::project_theta(double px, double py) const {
    const int n = static_cast<int>(wp_x_.size());
    if (n < 2) return 0.0;

    // ---- 1. turn the arc-length window into a range of waypoint indices ----
    double s_low  = theta_ - search_window_;
    double s_high = theta_ + search_window_;

    int i_begin = static_cast<int>(std::lower_bound(wp_s_.begin(), wp_s_.end(), s_low)  - wp_s_.begin());
    int i_end   = static_cast<int>(std::upper_bound(wp_s_.begin(), wp_s_.end(), s_high) - wp_s_.begin());

    if (i_begin > 0) i_begin -= 1;      // one extra point each side so the segments
    if (i_end   < n) i_end   += 1;      // at the edge of the window still get checked
    if (i_begin < 0) i_begin = 0;
    if (i_end   > n) i_end   = n;

    // ---- 2. coarse pass: nearest waypoint inside the window ----
    int    best_i       = i_begin;
    double best_dist_sq = std::numeric_limits<double>::infinity();

    for (int i = i_begin; i < i_end; i++) {
        double dx = px - wp_x_[i];
        double dy = py - wp_y_[i];
        double dist_sq = dx * dx + dy * dy;
        if (dist_sq < best_dist_sq) {
            best_dist_sq = dist_sq;
            best_i = i;
        }
    }

    // ---- 3. fine pass: project onto the two segments either side of it ----
    // Waypoints are ~0.46 m apart, so snapping to the nearest one would quantise
    // theta. Projecting onto the segment gives a continuous value.
    double best_theta = wp_s_[best_i];
    best_dist_sq = std::numeric_limits<double>::infinity();

    for (int i = best_i - 1; i <= best_i; i++) {
        if (i < 0 || i + 1 >= n) continue;               // no segment starts here

        double ax = wp_x_[i],     ay = wp_y_[i];         // segment start
        double bx = wp_x_[i + 1], by = wp_y_[i + 1];     // segment end
        double seg_x = bx - ax;
        double seg_y = by - ay;
        double seg_len_sq = seg_x * seg_x + seg_y * seg_y;
        if (seg_len_sq < 1e-12) continue;                // duplicate waypoints

        // fraction along the segment where the closest point sits, clamped to the
        // segment itself so the result can never land outside it
        double t = ((px - ax) * seg_x + (py - ay) * seg_y) / seg_len_sq;
        if (t < 0.0) t = 0.0;
        if (t > 1.0) t = 1.0;

        double dx = px - (ax + t * seg_x);
        double dy = py - (ay + t * seg_y);
        double dist_sq = dx * dx + dy * dy;

        if (dist_sq < best_dist_sq) {
            best_dist_sq = dist_sq;
            best_theta   = wp_s_[i] + t * std::sqrt(seg_len_sq);
        }
    }

    // keep theta inside the range the spline is defined on
    if (best_theta < 0.0)           best_theta = 0.0;
    if (best_theta > track_length_) best_theta = track_length_;
    return best_theta;
}

double MpccController::get_theta() const        { return theta_; }
double MpccController::get_track_length() const { return track_length_; }
