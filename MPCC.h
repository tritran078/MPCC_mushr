#pragma once
#include<vector>
#include<utility>
#include<cmath>
#include "mpcc_dynamic.h"
#include <casadi/casadi.hpp>


struct CostParam {
    float q_c      = 50.0;              //tuning constant for contour error
    float q_l      = 10.0;              //tuning constant for lag error
    float gamma    = 10.0;              //tuning constant for reward progress
    float R_steer  = 0.1;               //tuning constant for large input value
    float R_PWM    = 0.01;
    float R_delta  = 0.01;
    float Rt_steer = 1.0;               //tuning constant for large input difference
    float Rt_PWM   = 0.1;
    float Rt_delta = 0.1;
};


std::vector<double> cumulative_distance(std::vector<float>& waypoints);

Function contour_lag_error( 
                            const std::vector<double> th, 
                            const std::vector<double> x, 
                            const std::vector<double> y);



MX cost_function(int horizon, float dt, const CostParam& cost_param, MX& X /*output*/, MX& U/*input*/, const Function& err);
