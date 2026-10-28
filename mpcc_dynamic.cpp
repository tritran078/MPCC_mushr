#include "mpcc_dynamic.h"

// ---- internal helper functions (not exposed in header, same as your original) ----

Function rk4_step(const Function& f, double dt){
    SX x = SX::sym("x",7);
    SX u = SX::sym("u",3);
    
    SX k1 = f(std::vector<SX>{x, u})[0];
    SX k2 = f(std::vector<SX>{x + dt/2 * k1, u})[0];
    SX k3 = f(std::vector<SX>{x + dt/2 * k2, u})[0];
    SX k4 = f(std::vector<SX>{x + dt   * k3, u})[0];

    SX x_next = x + (dt/6.0) * (k1 + 2*k2 + 2*k3 + k4);

    return Function("F", {x, u}, {x_next}, {"x","u"}, {"x_next"});
}

static SX slip_angle_front(SX steering, SX vx, SX vy, SX yaw_rate, double lf, double eps = 1e-4){
    return steering - atan((vy + lf * yaw_rate) / (vx + eps));
}

static SX slip_angle_rear(SX vx, SX vy, SX yaw_rate, double lr, double eps = 1e-4){
    return (-1) * atan((vy - lr * yaw_rate) / (vx + eps));
}

// returns {Fzf, Fzr} -- always computed, no if-branch (accel_x=0 naturally gives static split)
static std::pair<SX,SX> load_transfer(const CarParam& car, SX accel_x){
    double Fzf_static = (car.mass * gravity * car.lr) / (car.lr + car.lf);
    double Fzr_static = (car.mass * gravity * car.lf) / (car.lr + car.lf);
    SX Fz_long = (car.mass * accel_x * car.height_COG) / (car.lr + car.lf);
    return { Fzf_static - Fz_long, Fzr_static + Fz_long };
}

// returns {front_friction, rear_friction}
static std::pair<SX,SX> tire_friction(const CarParam& car, const PacejkaParam& pac,
                                       SX vx, SX vy, SX yaw_rate, SX steering, SX accel_x){
    auto Fz = load_transfer(car, accel_x);
    SX Df = 0.8 * Fz.first;   // was D_param_compute, now just local SX
    SX Dr = 0.8 * Fz.second;

    SX front_slip = slip_angle_front(steering, vx, vy, yaw_rate, car.lf);
    SX rear_slip  = slip_angle_rear(vx, vy, yaw_rate, car.lr);

    SX front_friction = Df * sin(pac.C * atan(pac.B * front_slip));
    SX rear_friction  = Dr * sin(pac.C * atan(pac.B * rear_slip));

    return { front_friction, rear_friction };
}

// ---- the single function the solver calls ----
Function bicycle_model(const CarParam& car, const PacejkaParam& pac){

    SX x = SX::sym("x", 7);   // px, py, yaw, vx, vy, yaw_rate, theta
    SX u = SX::sym("u", 3);   // PWM, steering, d_theta

    SX px = x(0), py = x(1), yaw = x(2);
    SX vx = x(3), vy = x(4), yaw_rate = x(5), theta = x(6);
    SX PWM = u(0), steering = u(1), d_theta = u(2);

    // longitudinal force = motor - rolling resistance - air drag
    // PWM > 0 drives, PWM < 0 brakes (allowed by pwm_min < 0 in NlpBounds)
    SX F_motor   = car.C_m * PWM;
    SX F_rolling = car.C_r;              // vx >= vx_min > 0 in the plan, so this always opposes forward motion
    SX F_drag    = car.C_d * vx * vx;
    SX Fx = F_motor - F_rolling - F_drag;

    // NOTE (flagged, not solved): load_transfer needs accel_x, which is itself
    // an output of this same function (vx_dot). There's no "previous step" inside
    // a single symbolic evaluation. Using 0 here for now -- see "what's missing" below.
    SX accel_x_approx = SX(0);

    auto tire_F = tire_friction(car, pac, vx, vy, yaw_rate, steering, accel_x_approx);
    SX front_F = tire_F.first;
    SX rear_F  = tire_F.second;

    SX vx_dot    = (1.0 / car.mass) * (Fx - front_F * sin(steering) + car.mass * vy * yaw_rate);
    SX vy_dot    = (1.0 / car.mass) * (rear_F + front_F * cos(steering) - car.mass * vx * yaw_rate);
    SX yaw_accel = (1.0 / car.I)    * (front_F * car.lf * cos(steering) - rear_F * car.lr);

    SX px_dot = vx * cos(yaw) - vy * sin(yaw);
    SX py_dot = vx * sin(yaw) + vy * cos(yaw);
    SX theta_dot = d_theta;

    SX xdot = SX::vertcat({ px_dot, py_dot, yaw_rate, vx_dot, vy_dot, yaw_accel, theta_dot });

    return Function("f", {x, u}, {xdot}, {"x", "u"}, {"xdot"});
}