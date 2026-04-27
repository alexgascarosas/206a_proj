# ball_balance_controller

PID ball-on-plate controller for the **Universal Robots UR7e** using full 6-DOF inverse kinematics via **ikpy**.

```
/ball_state  →  PID  →  ikpy IK  →  FollowJointTrajectory  →  UR7e
```

---

## Package layout

```
ball_balance_controller/
├── ball_balance_controller/
│   ├── controller.py          ← main node (PID + IK)
│   └── mock_ball_publisher.py ← synthetic ball for offline testing
├── config/
│   ├── pid_params.yaml        ← all tunable parameters
│   └── tuning_presets.yaml    ← conservative / aggressive / etc.
├── launch/
│   ├── balance.launch.py      ← real robot launch
│   └── balance_sim.launch.py  ← simulation / offline launch
├── test/
│   └── test_controller_math.py← 27 pure-Python unit tests
├── package.xml
├── setup.py
└── setup.cfg
```

---

## Dependencies

| Dependency | Install |
|---|---|
| ROS2 Humble / Iron / Jazzy | [docs.ros.org](https://docs.ros.org) |
| `ball_tracker_msgs` | build from your ball_tracker_ros repo |
| `ikpy` | `pip install ikpy` |
| `numpy` | installed with ikpy |

---

## Quick start

### 1 — Install ikpy

```bash
pip install ikpy
```

### 2 — Build the package

```bash
cd ~/ros2_ws/src
# copy or symlink this package here
cd ~/ros2_ws
colcon build --packages-select ball_balance_controller
source install/setup.bash
```

### 3 — Set the URDF path

Edit `config/pid_params.yaml`:

```yaml
urdf_path: "/opt/ros/humble/share/ur_description/urdf/ur7e.urdf"
```

Or find it automatically:

```bash
find /opt/ros -name "ur7e.urdf" 2>/dev/null
```

### 4 — Launch on real robot

```bash
ros2 launch ball_balance_controller balance.launch.py
```

Or with a custom URDF path inline:

```bash
ros2 launch ball_balance_controller balance.launch.py \
  urdf_path:=/path/to/ur7e.urdf
```

### 5 — Test offline (no robot needed)

```bash
ros2 launch ball_balance_controller balance_sim.launch.py
```

Publishes a synthetic `/ball_state`; controller logic runs but no trajectory server is required.

---

## Control architecture

### Data flow

```
BallState { x, y, vx, vy, ball_found, markers_found }
    │
    ▼
Safety guards
  ├── stale message  (>0.5 s)  → return to base
  ├── markers < 3              → return to base
  └── ball missing (>0.25 s)  → return to base
    │
    ▼
PID ( x, y, vx, vy ) → pitch_cmd, roll_cmd  [rad]
    │
    ▼
build_target_transform( pitch_cmd, roll_cmd )
  position  = tcp_position (fixed from FK at startup)
  rotation  = base_orientation @ Rx(pitch) @ Ry(roll)
    │
    ▼
ikpy.inverse_kinematics( T_target, warm_start=filtered_joints )
    │
    ▼
rate_limit( Δjoint ≤ max_joint_delta_rad )
    │
    ▼
low_pass_filter( alpha=0.70 )
    │
    ▼
FollowJointTrajectory → /scaled_joint_trajectory_controller
```

## Parameters reference

All parameters live in `config/pid_params.yaml`.

### URDF

| Parameter | Default | Description |
|---|---|---|
| `urdf_path` | `""` | **Required.** Absolute path to UR7e URDF |
| `base_joints` | `[0.5,-2.35,2.65,-1.85,1.57,0.0]` | Level plate pose (rad) |

### PID gains

| Parameter | Default | Description |
|---|---|---|
| `kp_x` / `kp_y` | `0.00120` | Proportional gain |
| `kd_x` / `kd_y` | `0.00040` | Derivative gain |
| `ki_x` / `ki_y` | `0.00000` | Integral gain (disabled by default) |
| `roll_sign` | `1.0` | Set `-1.0` if ball moves wrong direction |
| `pitch_sign` | `1.0` | Set `-1.0` if ball moves wrong direction |

### Safety

| Parameter | Default | Description |
|---|---|---|
| `max_tilt_rad` | `0.060` | Max tilt per axis (~3.4°) |
| `max_joint_delta_rad` | `0.150` | Max Δjoint per step |
| `edge_limit_x_mm` | `60.0` | Hard edge rescue threshold X |
| `edge_limit_y_mm` | `60.0` | Hard edge rescue threshold Y |
| `deadband_mm` | `1.0` | Position deadband |
| `integral_limit` | `80.0` | Integrator anti-windup |
| `velocity_clip` | `120.0` | Velocity error clip (mm/s) |

### Timing & filtering

| Parameter | Default | Description |
|---|---|---|
| `command_rate_hz` | `10.0` | Control loop rate |
| `command_alpha` | `0.70` | Low-pass weight (higher = less filtering) |
| `trajectory_time` | `0.15` | Duration sent per trajectory goal (s) |
| `return_to_base_rate` | `0.25` | Blend rate toward flat pose |

### Reference trajectory

| Parameter | Default | Description |
|---|---|---|
| `track_circle` | `true` | Trace a circle |
| `track_figure8` | `false` | Trace a figure-8 |
| `circle_radius_mm` | `20.0` | Radius (mm) |
| `circle_period_s` | `20.0` | Period (s) |
| `settle_time_s` | `0.0` | Wait before starting trajectory |

---

## Tuning guide

Follow this sequence — do not skip steps.

### Step 1 — Verify direction signs

Set very small gains and check which way the ball moves:

```yaml
kp_x: 0.00030
kd_x: 0.00000
track_circle: false
```

- Ball at `x = +30 mm` → pitch should tilt to bring it back (negative x direction)
- If ball moves **away** from centre: flip `pitch_sign: -1.0`
- Repeat for Y axis with `roll_sign`

### Step 2 — Tune kp (proportional)

Increase `kp_x` and `kp_y` together until the ball oscillates around centre:

```
Too low  → ball drifts, slow return
Too high → ball oscillates / diverges
Sweet spot → ball overshoots slightly then settles
```

Typical range: `0.00080` – `0.00180`

### Step 3 — Tune kd (derivative)

Increase `kd_x` / `kd_y` until oscillation damps out in 1-2 swings:

```
Too low  → oscillation persists
Too high → sluggish, noisy response (amplifies velocity noise)
```

Typical range: `0.00020` – `0.00080`

### Step 4 — Enable integral (optional)

Only if there is a persistent steady-state offset after kp/kd are stable:

```yaml
ki_x: 0.000005
ki_y: 0.000005
```

Keep `integral_limit` small (20–40) to prevent windup.

### Step 5 — Enable circle tracking

Once the ball holds centre reliably:

```yaml
track_circle: true
circle_radius_mm: 10.0   # start small
circle_period_s:  30.0   # start slow
```

Increase radius and reduce period gradually.

---

## URDF active_links_mask

If IK gives wrong results, the `active_links_mask` may be mismatched. Check:

```python
import ikpy.chain
chain = ikpy.chain.Chain.from_urdf_file("/path/to/ur7e.urdf")
print(f"Chain has {len(chain.links)} links:")
for i, l in enumerate(chain.links):
    print(f"  [{i}] {l.name}")
```

The mask length must equal `len(chain.links)`. Standard UR7e:
```
[base_link(fixed), j1, j2, j3, j4, j5, j6, tool0(fixed)]
→ mask = [False, True, True, True, True, True, True, False]
```

If your URDF has an extra world frame:
```
[world(fixed), base_link(fixed), j1, j2, j3, j4, j5, j6, tool0(fixed)]
→ mask = [False, False, True, True, True, True, True, True, False]
```

---

## Running tests

No ROS2 installation required for unit tests:

```bash
cd ~/ros2_ws/src/ball_balance_controller
pip install pytest numpy
pytest test/test_controller_math.py -v
```

Expected output: **27 passed**.

---

## Troubleshooting

| Symptom | Likely cause | Fix |
|---|---|---|
| `RuntimeError: urdf_path not set` | Missing URDF path | Set `urdf_path` in params YAML |
| `IK solver error` appears in log | Bad warm-start or singular config | Reduce `max_tilt_rad`; check `active_links_mask` |
| Ball oscillates after tuning | kp too high or kd too low | Reduce `kp` by 20%; increase `kd` |
| Ball drifts to one side | Wrong `pitch_sign` / `roll_sign` | Flip the relevant sign |
| `Trajectory goal rejected` | Robot E-stop or speed limit | Check UR teach pendant; reduce `command_rate_hz` |
| `Only N markers found` | Camera occlusion | Improve lighting / marker placement |
