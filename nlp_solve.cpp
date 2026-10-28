#include <vector>
#include <limits>
#include "nlp_solve.h"
using namespace std;
using namespace casadi;

NlpSolver build_nlp(const Function& F, int N, const CostFn& cost, const NlpBounds& b,
                    const Function& err){
    const int nx = 7, nu = 3;
    const double INF = numeric_limits<double>::infinity();

    MX X  = MX::sym("X", nx, N + 1);
    MX U  = MX::sym("U", nu, N);
    MX x0 = MX::sym("x0", nx);                     // parameter: current state

    // dynamics constraints
    vector<MX> g;
    g.push_back(X(Slice(), 0) - x0);
    for(int k = 0; k < N; k++){
        MX x_next = F(vector<MX>{X(Slice(), k), U(Slice(), k)})[0];
        g.push_back(X(Slice(), k + 1) - x_next);
    }

    // track boundary constraints: contour error at each predicted step must stay inside the track
    // (k starts at 1 because X(:,0) is fixed to the current state and cannot be changed)
    for(int k = 1; k <= N; k++){
        vector<MX> e = err(vector<MX>{X(0, k), X(1, k), X(6, k)});
        MX e_c = e[1];                               // contour error = sideways distance from centerline
        g.push_back(e_c);
    }

    MX J = cost(X, U);
    MX w = MX::vertcat({vec(X), vec(U)});

    // the NLP and the solver
    MXDict nlp = {{"x", w}, {"p", x0}, {"f", J}, {"g", vertcat(g)}};
    Dict opts;
    opts["ipopt.print_level"] = 0;
    opts["print_time"] = 0;
    opts["ipopt.max_iter"] = 50;    // was 200: caps how long one bad solve (e.g. near the wall) can take
    // opts["expand"] = true;  // NOT possible: the bspline interpolant (fx, fy) has no SX version

    NlpSolver s;
    s.N = N;
    s.solver = nlpsol("mpcc", "ipopt", nlp, opts);

    // bounds
    const int nw = nx*(N + 1) + nu*N;
    vector<double> lbw(nw, -INF), ubw(nw, INF);
    for(int k = 0; k <= N; k++){
        int base = nx*k;
        if(k >= 1){
            lbw[base + 3] = b.vx_min;  ubw[base + 3] = b.vx_max;
        }
        lbw[base + 6] = 0.0;           ubw[base + 6] = b.theta_max;
    }
    for(int k = 0; k < N; k++){
        int base = nx*(N + 1) + nu*k;
        lbw[base]     = b.pwm_min;     ubw[base]     = b.pwm_max;
        lbw[base + 1] = -b.steer_max;  ubw[base + 1] = b.steer_max;
        lbw[base + 2] = b.dtheta_min;  ubw[base + 2] = b.dtheta_max;
    }
    s.lbw = DM(lbw);  s.ubw = DM(ubw);
    // constraint bounds: first the dynamics (all = 0), then the track boundary (one per step)
    const int n_dyn   = nx*(N + 1);
    const int n_track = N;
    const double e_c_max = b.track_half_width - b.car_margin;
    vector<double> lbg(n_dyn + n_track, 0.0), ubg(n_dyn + n_track, 0.0);
    for(int k = 0; k < n_track; k++){
        lbg[n_dyn + k] = -e_c_max;
        ubg[n_dyn + k] =  e_c_max;
    }
    s.lbg = DM(lbg);
    s.ubg = DM(ubg);
    return s;
}

DM nlp_initial_guess(const NlpSolver& s, const DM& x0){
    DM X0 = DM::repmat(x0, s.N + 1, 1);
    DM U0 = DM::zeros(3 * s.N);
    return DM::vertcat({X0, U0});
}

DM solve_nlp(const NlpSolver& s, const DM& x0, DM& w_guess){
    DMDict arg = {{"x0", w_guess}, {"p", x0},
                  {"lbx", s.lbw}, {"ubx", s.ubw},
                  {"lbg", s.lbg}, {"ubg", s.ubg}};
    DMDict res = s.solver(arg);
    DM w = res.at("x");
    w_guess = w;
    int off = 7 * (s.N + 1);
    return w(Slice(off, off + 3));         // first input: PWM, steering, d_theta
}