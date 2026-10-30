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

## 2026-10-28 — theta now projected from the car's real position (ROOT-CAUSE FIX)
Was: `theta_ = w_guess_(nx*1 + 6)` — integrated open-loop from the previous plan, never
corrected against the car. Two consequences:
- theta drifted off the car's true arc-length position, so `contour_lag_error` evaluated
  fx/fy and the tangent/normal at the WRONG point on the spline. The track-boundary
  constraint `|e_c| <= 0.85` was therefore checking the wrong piece of track.
- theta was read out of `w_guess_`, which a FAILED solve overwrites with garbage, so one
  bad solve corrupted theta -> corrupted the next solve's reference frame -> more failures.
  That compounding loop is why the lag got progressively worse instead of recovering.

Now: `theta_ = project_theta(px, py)` at the top of `step()`, from the measured position.
- New private method `project_theta`, new members `wp_x_`, `wp_y_`, `wp_s_`, `search_window_`.
- Coarse pass (nearest waypoint) + fine pass (projection onto the two adjacent segments),
  so theta is continuous rather than quantised to the 0.46 m waypoint spacing.
- Search window is +/-3 m of ARC LENGTH, not straight-line distance, so it is always one
  continuous section of track and can never snap to a nearby part of the circuit.
- Assumes the car starts near waypoint 0 (sim_test.py places it there).

Validated against budapest_centerline.csv (401.95 m, 876 pts, 0.46 m spacing):
on-waypoint error 0.0 m; 0.6 m sideways offset error 0.037 m; full lap tracks to exactly
401.95 m with 0 backward steps. Projection function compiles clean under -Wall -Wextra.

Backups: `mpcc_controller.h.bak_before_project`, `mpcc_controller.cpp.bak_before_project`
Rebuild needed: `cd build && make`

## 2026-10-29 — chassis had 1 mm ground clearance, dragged on the floor in hard corners
The chassis geom used `mushr_base_collision_tx2` for BOTH visual and collision. That mesh is
a crude hull of the whole vehicle (0.439 x 0.265 x 0.226 m), reaching 66 mm past the front
axle and down to +1.0 mm above the floor -> only 0.5 deg of roll to touch. Present in
`racecar_converted_backup.xml` too, so it came from the URDF->MJCF conversion.

Fix in `meshes/racecar_converted.xml`:
- hull mesh geom -> `contype="0" conaffinity="0"` (visual only)
- new `chassis_collision` box, `size="0.19 0.10 0.045" pos="0 0 0.070"`
  bottom 0.025 m (15.1 deg of roll to scrape), top 0.115 m (above the 0.1 m walls, so wall
  hits still register), half-width 0.10 m (inside the 0.1365 m wheels, so wheels hit first).
  Mass/inertia unaffected: the body has an explicit <inertial>.

Measured, 800 steps of steady steer on OPEN GROUND (no walls), chassis-vs-floor contacts only:
| steer | speed | before: contacts / roll / speed kept | after: contacts / roll / speed kept |
|---|---|---|---|
| 0.10 | 3 m/s | 0 / 0.04 deg / 2.94 | 0 / 0.04 deg / 2.94 |
| 0.20 | 3 m/s | 0 / 0.06 deg / 2.91 | 0 / 0.06 deg / 2.91 |
| 0.35 | 5 m/s | 36 / 2.12 deg / 3.58 | 0 / 1.16 deg / 4.42 |
| 0.49 | 7 m/s | 101 / 8.67 deg / 3.01 | 0 / 4.75 deg / 5.08 |

So: hard corners only. Cornering speed +23% at steer 0.35, +69% at steer 0.49. Roll also
drops (8.67 -> 4.75 deg) because the hull hitting the floor was acting as a pivot.

NOTE: my earlier claim that gentle corners (0.20 steer @ 3 m/s) stopped the car was WRONG --
that test ran inside the track and the car was hitting a WALL, not the floor. A 0.20 steer
angle gives a 1.46 m turn radius, which does not fit a 2.2 m wide track.

`mushr_base_tx2.stl` (the real visual mesh) is an ASCII STL, which MuJoCo cannot load, so the
viewer still shows the collision hull. Would need converting to binary STL first.

Backup: `meshes/racecar_converted.xml.bak_before_clearance`. No rebuild needed (XML only).
