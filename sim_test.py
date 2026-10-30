import sys
import math
import mujoco
import mujoco.viewer

sys.path.append("build")                    # folder with mpcc_py .so
import mpcc_py
from load_waypoints import load_waypoints


def read_car_state(data):
    # 1. position (world frame)
    x = data.qpos[0]
    y = data.qpos[1]

    # 2. quaternion -> yaw (rotation around the vertical z axis)
    qw = data.qpos[3]
    qx = data.qpos[4]
    qy = data.qpos[5]
    qz = data.qpos[6]
    yaw = math.atan2(2.0 * (qw * qz + qx * qy),
                     1.0 - 2.0 * (qy * qy + qz * qz))

    # 3. world-frame velocity -> body-frame velocity (rotate by -yaw)
    vx_world = data.qvel[0]
    vy_world = data.qvel[1]
    vx = math.cos(yaw) * vx_world + math.sin(yaw) * vy_world     # forward speed
    vy = -math.sin(yaw) * vx_world + math.cos(yaw) * vy_world    # sideways (slip) speed

    # 4. yaw rate (free joint angular velocity is already in the body frame)
    yaw_rate = data.qvel[5]

    return [x, y, yaw, vx, vy, yaw_rate]


model = mujoco.MjModel.from_xml_path("meshes/scene_budapest.xml")
data = mujoco.MjData(model)

wp_x, wp_y = load_waypoints("budapest_centerline.csv")

# ======================= MPCC TUNING (no recompiling needed) =======================
# All values are baked into the solver when MpccController(...) is created,
# so change them here, before that line.

# ---- cost weights (CostParam) ----
cost = mpcc_py.CostParam()
cost.q_c      = 500.0   # contour error weight: pull toward centerline (low = more corner cutting)
cost.q_l      = 500.0   # lag error weight: keeps theta beside the car (keep high)
cost.gamma    = 600.0   # progress reward: go fast
cost.R_PWM    = 0.9    # penalty on large throttle / brake
cost.R_steer  = 0.9     # penalty on large steering angle
cost.R_delta  = 0.01    # penalty on large d_theta (keep tiny, it fights gamma)
cost.Rt_PWM   = 0.1     # penalty on throttle CHANGE between steps (smoothness)
cost.Rt_steer = 10.0    # penalty on steering CHANGE between steps (smoothness)
cost.Rt_delta = 0.1     # penalty on d_theta CHANGE between steps

# ---- limits (NlpBounds) ----
bounds = mpcc_py.NlpBounds()
bounds.pwm_min          = -5.0           # negative = braking
bounds.pwm_max          = 5.0            # max throttle
bounds.steer_max        = 0.49           # rad, matches MuJoCo steering joint range
bounds.dtheta_min       = 0.0            # progress can't go backwards (keep 0)
bounds.dtheta_max       = 7.0            # m/s, max progress speed = effective top speed
bounds.vx_min           = 0.1            # m/s, keep > 0 (slip angle divides by vx)
bounds.vx_max           = float("inf")   # m/s, no upper speed limit
bounds.track_half_width = 1.1            # m, centerline to wall
bounds.car_margin       = 0.25           # m, car half-width + safety buffer

# ---- solver setup ----
solver_dt       = 0.02   # s, MPC time step (must match control_dt below)
horizon         = 50      # steps, 50 x 0.02 s = 1.0 s look-ahead
speed_lookahead = 5       # which predicted step's vx is sent as the speed command

controller = mpcc_py.MpccController(wp_x, wp_y, solver_dt, horizon, cost, bounds, speed_lookahead)
# ===================================================================================

start_yaw = math.atan2(wp_y[1] - wp_y[0], wp_x[1] - wp_x[0])
data.qpos[0] = wp_x[0]
data.qpos[1] = wp_y[0]
data.qpos[3] = math.cos(start_yaw / 2.0)
data.qpos[4] = 0.0
data.qpos[5] = 0.0
data.qpos[6] = math.sin(start_yaw / 2.0)

# NEW 2: small starting speed (avoids NaN at vx = 0)
data.qvel[0] = 0.5 * math.cos(start_yaw)
data.qvel[1] = 0.5 * math.sin(start_yaw)

mujoco.mj_forward(model, data)    # recompute everything after editing qpos by hand


control_dt = solver_dt    # control period = MPC time step
physics_steps_per_control = round(control_dt / model.opt.timestep)   # 0.02 / 0.002 = 10

with mujoco.viewer.launch_passive(model, data) as viewer:
    # NEW 4: camera follows the car
    viewer.cam.type = mujoco.mjtCamera.mjCAMERA_TRACKING
    viewer.cam.trackbodyid = model.body("base_footprint").id
    viewer.cam.distance = 1.5

    while viewer.is_running():
        state = read_car_state(data)                 # ground truth from MuJoCo
        steer, speed = controller.step(state)        # MPCC = commands only

        data.ctrl[0] = steer                         # steering angle (rad)
        data.ctrl[1] = speed                         # speed command (m/s)

        for i in range(physics_steps_per_control):   # MuJoCo physics moves the car
            mujoco.mj_step(model, data)

        # MOVED 3: print once per control step (outside the for loop)
        print(f"t={data.time:.2f}  real_vx={state[3]:.2f}  cmd_speed={speed:.2f}  steer={steer:.2f}")

        viewer.sync()

        if controller.get_theta() >= controller.get_track_length() - 0.5:
            print("lap done, time =", data.time)
            break
