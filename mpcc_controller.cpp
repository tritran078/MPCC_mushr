#include "mpcc_controller.h"
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
    // full 7-state = MuJoCo's 6 measured states + our own progress theta
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
    const int index_theta_next = nx * 1 + 6;    // predicted theta at k = 1

    double steering_cmd = static_cast<double>(u0(1));
    double speed_cmd    = static_cast<double>(w_guess_(index_vx_next));
    theta_              = static_cast<double>(w_guess_(index_theta_next));

    return {steering_cmd, speed_cmd};
}

double MpccController::get_theta() const        { return theta_; }
double MpccController::get_track_length() const { return track_length_; }
