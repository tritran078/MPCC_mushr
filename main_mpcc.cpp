#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <cmath>
#include <casadi/casadi.hpp>
#include "MPCC.h"
#include "nlp_solve.h"



// Wraps cost_function so build_nlp can call it as cost(X, U)
struct MpccCost {
    int N;
    double dt;
    CostParam cost_param;
    Function err;

    MX operator()(MX& X, MX& U) const {
        return cost_function(N, dt, cost_param, X, U, err);
    }
};

using namespace std;

// Reads a CSV with header "x,y" and returns one flat list: x0, y0, x1, y1, ...
// (the same layout cumulative_distance() expects)
vector<float> load_waypoints_csv(const string& csv_path){
    vector<float> waypoints;
    ifstream file(csv_path);
    if (!file.is_open()){
        cerr << "Could not open " << csv_path << endl;
        return waypoints;
    }

    string line;
    getline(file, line);                       // skip the header row "x,y"

    while (getline(file, line)){               // one line = "-0.1774,0.1463"
        if (line.empty()) continue;
        stringstream line_stream(line);
        string x_text, y_text;
        getline(line_stream, x_text, ',');     // text before the comma
        getline(line_stream, y_text, ',');     // text after the comma
        waypoints.push_back(stof(x_text));     // text -> float
        waypoints.push_back(stof(y_text));
    }
    return waypoints;
}

int main(){

    constexpr double dt = 0.02;  //time step = 50 hz
    constexpr int horizon = 100;      // horizon for each calculation of 0.25 * 30 = 7.5 sec
    // run ./mpcc from the build/ folder, so the CSV is one folder up
    vector<float> wp = load_waypoints_csv("../budapest_centerline.csv");   //waypoints path (x0,y0,x1,y1,...)
    cout << "loaded " << wp.size() / 2 << " waypoints" << endl;
    vector<double> wp_x , wp_y;  //list of waypoints only X and only Y
    CarParam car_param;          //physcial car param
    PacejkaParam pac;            // pacjecka param


    for(int i =0; i < wp.size(); i+=2){  ///seperate into to list
        wp_x.push_back(wp[i]);
        wp_y.push_back(wp[i+1]);
    }

    //create the bicycle model equation expression
    Function bicycle = bicycle_model(car_param, pac);
    //using RK4 ttp get the discrete values at timesteps
    bicycle = rk4_step(bicycle,dt);

    //makign cumulative distance vector
    vector<double> distance = cumulative_distance(wp);

    Function err = contour_lag_error(distance,wp_x, wp_y);


    CostParam cost_param{50, 10, 0.5,   0.1, 0.01, 0.01,   0.1, 0.1, 0.1};

    // limits
    NlpBounds bounds;
    bounds.theta_max = distance.back();

    // build the solver once
    MpccCost cost_obj{horizon, dt, cost_param, err};
    CostFn cost = cost_obj;
    NlpSolver s = build_nlp(bicycle, horizon, cost, bounds, err);

    // start state: px, py, yaw, vx, vy, yaw_rate, theta
    double yaw0 = atan2(wp_y[1] - wp_y[0], wp_x[1] - wp_x[0]);
    DM x = DM::vertcat({wp_x[0], wp_y[0], yaw0, 1.0, 0.0, 0.0, 0.0});
    DM w = nlp_initial_guess(s, x);

    // control loop
    for (int k = 0; k < 500; k++) {
        DM u = solve_nlp(s, x, w);          // [PWM, steering, d_theta]
        x = bicycle(DMVector{x, u})[0];           // advance the car (stand-in for MuJoCo)

        cout << k << " px=" << x(0) << " py=" << x(1)
             << " vx=" << x(3) << " theta=" << x(6) << endl;

        if (static_cast<double>(x(6)) >= distance.back() - 0.05) break;   // end of path
    }

    return 0;
}
 
