#include "MPCC.h"
using namespace std;
using namespace casadi;


std::vector<float> waypoints = {}; //way points x1,x2,y1,y2

vector<double> cumulative_distance(vector<float>& waypoints){
    vector<double>  distance = {0.0};
    float sum =0;
    for(int i =0; i + 3 < waypoints.size();i += 2){
        float x_diff = waypoints[i] - waypoints[i+2];
        float y_diff = waypoints[i+1] - waypoints[i+3];
        float magnitude = sqrt(x_diff*x_diff + y_diff*y_diff);
        sum += magnitude;
        distance.push_back(sum);
    }
    return distance;
}


Function contour_lag_error(
                            const vector<double> th, 
                            const vector<double> x, 
                            const vector<double> y){
    //creating fucntion fx and fy that will turn cumulative waypoints into smooth curves
    Function fx = interpolant("fx"/* name */,"bspline"/* type of function */,{th}/*input*/,x/* output*/);
    Function fy = interpolant("fy","bspline",{th},y);

    //creating symbolic place holder
    MX px = MX::sym("px");                //making a symbol for actual position of car at x
    MX py = MX::sym("py");                //making a symbol for actual position of car at y
    MX theta = MX::sym("theta");          //making a symbol for the progress of the car

    //car's position
    MX rx = fx(vector<MX>{theta})[0];     //input theta to get the cars x reference position from smooth curve
    MX ry = fy(vector<MX>{theta})[0];     //input theta to get the cars y reference position from smooth curve

    //jacobian for derivative, getting tangent vector at the reference theta point
    MX dx = jacobian(rx,theta);           //jacobian of function for car's x position at theta
    MX dy = jacobian(ry,theta);           //jacobian of function for car's y position at theta

    //from tangent get unit tangent by divide with magnitude, normal unit vector = flip
    MX mag = sqrt(dx*dx + dy*dy);
    MX tanx = dx/mag;
    MX tany = dy/mag;
    
    //get the error vector
    MX ex = px - rx;
    MX ey = py - ry;
    //dot the error component with tangent and normal unit vectors
    MX e_l = ex * tanx + ey * tany;
    MX e_c = ex * (-tany) + ey * tanx;

    return Function("error",{px,py,theta},{e_l,e_c},{"px","py","theta"},{"e_l","e_c"});

}


 MX cost_function(int horizon, float dt, const CostParam& cost_param, MX& X /*output*/, MX& U/*input*/, const Function& err){
    MX J = 0;
    for(int i =0; i < horizon; i++){
        vector<MX> e = err(vector<MX>{X(0,i),X(1,i),X(6,i)});
        MX e_l = e[0];
        MX e_c = e[1];
        J +=  cost_param.q_l*e_l*e_l + cost_param.q_c*e_c*e_c;   //punish contour and error lag
        J +=  -(cost_param.gamma * U(2,i) * dt);                 //reward progress 
        J += cost_param.R_PWM*U(0,i)*U(0,i) + cost_param.R_steer*U(1,i)*U(1,i) + cost_param.R_delta*U(2,i)*U(2,i); //punish large input

         if (i > 0){                                           // input change
            MX dPWM = U(0,i) - U(0,i-1);
            MX dSt  = U(1,i) - U(1,i-1);
            MX dTh  = U(2,i) - U(2,i-1);
            J += cost_param.Rt_PWM*dPWM*dPWM + cost_param.Rt_steer*dSt*dSt + cost_param.Rt_delta*dTh*dTh;
        }
    }
    return J;
 }

 