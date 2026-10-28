# MPCC tuning log

## 2026-10-27 — fix slow acceleration (car crept from 0 to ~0.2 m/s in 0.3 s)
Backup of the file before these changes: `mpcc_controller.cpp.bak_before_tuning`

| Change | Before | After | Where |
|---|---|---|---|
| gamma (progress reward) | 0.5 | 10.0 | `mpcc_controller.cpp`, CostParam line |
| Rt_steer (steering-change penalty) | 0.1 | 1.0 | `mpcc_controller.cpp`, CostParam line |
| speed command look-ahead | vx at k = 1 | vx at k = 5 | `mpcc_controller.cpp`, `step()` |

To revert: `cp mpcc_controller.cpp.bak_before_tuning mpcc_controller.cpp`, then `cd build && make mpcc_py`

## 2026-10-27 — tuning parameters moved to Python (no rebuild needed to tune)
Backups before this change: `MPCC.h.bak_before_params`, `mpcc_controller.h.bak_before_params`,
`mpcc_controller.cpp.bak_before_params`, `binding.cpp.bak_before_params`
- CostParam now has default values in MPCC.h (same as the tuned values above)
- CostParam, NlpBounds and speed_lookahead are constructor arguments, exposed to Python
- To revert: copy each `*.bak_before_params` back over its file, then `cd build && make mpcc_py`

## 2026-10-27 — sim_test.py updated (tuning params, start speed 0.5 m/s, print once per step, tracking camera)
Backup: `sim_test.py.bak_before_params`

## 2026-10-27 — removed MuJoCo throttle speed clamp (ctrlrange -5 5 -> unlimited) in meshes/racecar_converted.xml
Backup: `meshes/racecar_converted.xml.bak_before_unclamp`

## 2026-10-27 — removed vx_max (MPCC upper speed bound): default now infinity in nlp_solve.h; vx_min = 0.1 kept
Backups: `nlp_solve.h.bak_before_vxmax`, `sim_test.py.bak_before_vxmax`

## 2026-10-27 — dtheta_max 3.0 -> 12.0 (set in sim_test.py; nlp_solve.h default still 3.0)

## 2026-10-27 — dtheta_max 12.0 -> 5.0 (sim_test.py)

## 2026-10-27 — added braking + drag to the MPCC model
- `mpcc_dynamic.h`: CarParam gets `C_r = 0.5` N (rolling resistance) and `C_d = 0.05` N/(m/s)^2 (air drag), both ESTIMATED
- `mpcc_dynamic.cpp`: `Fx = C_m*PWM` -> `Fx = C_m*PWM - C_r - C_d*vx^2`
- `nlp_solve.h`: `pwm_min` default 0.0 -> -5.0 (braking); also set in `sim_test.py`
- Backups: `mpcc_dynamic.h.bak_before_drag`, `mpcc_dynamic.cpp.bak_before_drag`, `nlp_solve.h.bak_before_brake`, `sim_test.py.bak_before_brake`
- Rebuild needed: `cd build && make` (affects both mpcc and mpcc_py)

## 2026-10-27 — fix corner sliding (late braking, wheelspin, steering saturation)
| Change | Before | After | Where |
|---|---|---|---|
| horizon | 40 (0.8 s) | 75 (1.5 s) | `sim_test.py` |
| throttle actuator kv | 400 | 50 | `meshes/racecar_converted.xml` |
| throttle actuator forcerange | ±500 | ±40 (~ grip limit) | `meshes/racecar_converted.xml` |
| steer_max | 0.4 | 0.49 | `sim_test.py` |
Backups: `sim_test.py.bak_before_three`, `meshes/racecar_converted.xml.bak_before_three`

## 2026-10-27 — wheel (throttle) joint damping/armature too large -> sluggish acceleration
- `meshes/racecar_converted.xml`, default class "throttle": damping 0.01 -> 0.0005, armature 0.01 -> 0.0005
- Backup: `meshes/racecar_converted.xml.bak_before_wheeljoint`

## 2026-10-27 — horizon 75 -> 50 (1.0 s look-ahead) for faster solves (sim_test.py)

## 2026-10-27 — q_l 10 -> 500 (sim_test.py), keeps theta beside the car

## 2026-10-27 — q_l 10 -> 500 (sim_test.py), keeps theta beside the car. Backup: `sim_test.py.bak_before_ql`

## 2026-10-27 — all tuning constants (every CostParam + NlpBounds field, dt, horizon, speed_lookahead) listed explicitly in sim_test.py; values unchanged. Backup: `sim_test.py.bak_before_alltuning`

## 2026-10-27 — dtheta_max 5.0 -> 12.0 (sim_test.py)

## 2026-10-27 — wheel friction 1.0 (default) -> 1.5 on all 4 wheel geoms (meshes/racecar_converted.xml). Backup: `meshes/racecar_converted.xml.bak_before_friction`

## 2026-10-27 — MPCC tire friction 0.8 (hardcoded) -> CarParam.mu = 1.5, matches MuJoCo wheel friction
- `mpcc_dynamic.h`: new field `mu = 1.5`; `mpcc_dynamic.cpp`: Df/Dr use `car.mu`
- Backups: `mpcc_dynamic.h.bak_before_mu`, `mpcc_dynamic.cpp.bak_before_mu`. Rebuild needed.

## 2026-10-27 — REVERTED both tire-friction changes: wheel friction back to default 1.0 (XML), MPCC back to hardcoded 0.8 (mpcc_dynamic.h/.cpp). Rebuild needed.

## 2026-10-27 — dtheta_max 12.0 -> 7.0 (sim_test.py)

## 2026-10-27 — limits back to regular: pwm ±10 -> ±5, dtheta_max 12 -> 7 (sim_test.py). Backup: `sim_test.py.bak_before_threshold_revert`

## 2026-10-27 — solver speed: ipopt.max_iter 200 -> 50, expand = true (nlp_solve.cpp). Backup: `nlp_solve.cpp.bak_before_expand`. If build/run errors mention eval_sx / BSpline, remove the expand line.

## 2026-10-27 — removed expand = true (error: eval_sx not defined for BSpline). max_iter = 50 kept.
