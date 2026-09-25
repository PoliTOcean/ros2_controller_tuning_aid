# Nereo Controller Tuning Aid

ROS 2 (Humble) workspace containing the **PID / state-space controller** for the **Nereo** ROV by PoliTOcean.

This repo is meant to live **next to** [`nereo_ros2_code`](https://github.com/PoliTOcean/nereo_ros2_code) — the workstation launch file in `gui_pkg` automatically picks up this overlay (`AMENT_PREFIX_PATH` is prepended at launch time) and spawns the controller node alongside the GUI.

The core control algorithms are reused from [`nereo_FC_firmware`](https://github.com/PoliTOcean/nereo_FC_firmware); this node acts as a *middleman* between ROS 2 and the firmware control logic, so gains and modes can be tuned at runtime via ROS parameters without re-flashing the FC.

---

## Table of contents

1. [What's inside](#whats-inside)
2. [Installation](#installation)
3. [Build](#build)
4. [Run](#run)
5. [Control modes](#control-modes)
6. [Parameters](#parameters)
7. [Saved gains](#saved-gains)
8. [Topics](#topics)
9. [Tuning from the GUI](#tuning-from-the-gui)
10. [Troubleshooting](#troubleshooting)

---

## What's inside

```
ros2_controller_tuning_aid/
└── src/
    ├── nereo_controller_node/
    │   ├── src/nereo_controller_node.cpp     # C++ controller node
    │   ├── launch/
    │   │   ├── nereo_controller.launch.py            # controller only
    │   │   └── nereo_controller_with_gui.launch.py   # controller + standalone PyQt tuner
    │   ├── scripts/pid_tuner_gui.py          # standalone tuner (used by *_with_gui launch)
    │   └── test/test_nereo_controller.py
    └── nereo_interfaces/                     # CommandVelocity, ThrusterStatuses (git submodule)
```

Two ways to drive the tuning UI:
- **Recommended:** use the integrated **TUNER** window inside the main Nereo dashboard (`gui_pkg`). It speaks the same ROS parameter services and shows live setpoints/errors/pid_terms.
- **Standalone:** `pid_tuner_gui.py` is kept as a fallback for headless tuning without the full GUI stack.

---

## Installation

### 1. Clone

`nereo_interfaces` is a git submodule. Clone with `--recurse-submodules`:

```bash
git clone --recurse-submodules https://github.com/PoliTOcean/ros2_controller_tuning_aid.git
```

If already cloned without it:

```bash
git submodule update --init
```

### 2. System dependencies

This package is pure C++/Python with standard ROS 2 deps (`rclcpp`, `sensor_msgs`, `std_msgs`, `nereo_interfaces`). For the standalone tuner only:

```bash
sudo apt install python3-pyqt5
```

The integrated tuner inside `gui_pkg` uses PyQt6 — see the `nereo_ros2_code` README for that install.

### 3. rosdep

```bash
rosdep install --from-paths src --ignore-src -r -y
```

---

## Build

```bash
colcon build --symlink-install && source install/setup.zsh
```

> Build with `--symlink-install` so the tuner's **Salva YAML** button writes to
> `src/nereo_controller_node/config/controller_params.yaml` — the file tracked
> in git — instead of a copy under `install/` that the next plain `colcon
> build` would silently overwrite. If this overlay was already built without
> the flag, remove `build/` and `install/` once and rebuild with it.
>
> When this overlay is built **before** launching `gui_pkg/workstation.launch.py`, the workstation launch file finds the controller automatically — no manual `source` of this workspace is needed at run time.

---

## Run

### Standalone (controller only)

```bash
ros2 launch nereo_controller_node nereo_controller.launch.py
```

With an initial mode (0 passthrough / 1 PID / 2 PID-AW / 3 CS) and/or a
non-default params file:

```bash
ros2 launch nereo_controller_node nereo_controller.launch.py control_mode:=2 params_file:=/path/to/controller_params.yaml
```

`params_file` defaults to the installed `config/controller_params.yaml`
(D-03) and carries `kp`/`ki`/`kd`, `anti_windup_gains`, `authority_cap` and
the `cs_*` gains. The per-gain and per-CS launch arguments from earlier
versions of this launch file are gone — edit the YAML by hand or with the
tuner's **Salva YAML** button instead.

### Standalone with the legacy PyQt5 tuner

```bash
ros2 launch nereo_controller_node nereo_controller_with_gui.launch.py
```

### Inside the full Nereo stack (recommended)

Just launch the workstation stack from `nereo_ros2_code` — the controller is started inside it:

```bash
ros2 launch gui_pkg workstation.launch.py
```

Then open the **TUNER** window in the dashboard.

---

## Control modes

Set via `control_mode` ROS parameter:

| Mode | Name | Behavior |
|---|---|---|
| 0 | `DIRECT_PASSTHROUGH` | Commands forwarded as-is, no feedback |
| 1 | `PID_CONTROL` | PID on depth, roll, pitch, yaw |
| 2 | `PID_ANTI_WINDUP` | PID with anti-windup on integrators |
| 3 | `CS_CONTROLLER` | State-space controllers for depth/roll/pitch + PID for yaw |

---

## Parameters

All parameters are runtime-tunable via `ros2 param set` or the TUNER GUI.

| Name | Type | Notes |
|---|---|---|
| `control_mode` | `int` | 0–3, see [Control modes](#control-modes) |
| `kp`, `ki`, `kd` | `double[4]` | PID gains, ordered `[depth, roll, pitch, yaw]`; ship at zero (D-05) |
| `anti_windup_gains` | `double[4]` | Back-calculation gain per axis, ordered `[depth, roll, pitch, yaw]`; default `1.0`; needs 4 finite values >= 0, `0` disables back-calculation on that axis |
| `authority_cap` | `double` | `0`–`1`; clamps only the correction the controller adds to the pilot's command on each axis (also the mode-2 back-calculation limit); code default `0.0` (no feedback), the shipped YAML uses `0.25` |
| `manual_setpoint_depth/roll/pitch/yaw` | `bool` | When `true`, axis uses `setpoint_*` instead of tracking the current value |
| `setpoint_depth` | `double` | **metres**, positive down, same frame as `/barometer_depth`. Positive heave ascends (operator-confirmed sign, 04-02) |
| `setpoint_roll/pitch/yaw` | `double` | **radians**. GUI sends them converted from degrees |
| `cs_kx0`, `cs_kx1`, `cs_kx2` | `double[2]` | State feedback gains for heave / roll / pitch |
| `cs_ki0`, `cs_ki1`, `cs_ki2` | `double` | Integral gains for the CS controller |
| `cs_heave_min`, `cs_heave_max` | `double` | Saturation limits — heave |
| `cs_angle_min`, `cs_angle_max` | `double` | Saturation limits — roll / pitch |

> **Unit conventions:** the controller works internally in radians for
> roll/pitch/yaw and metres for depth (no Pascal conversion since 04-02). The
> Nereo dashboard converts angle setpoints to/from degrees; `setpoint_depth`
> passes through in metres end to end. If you set parameters directly from
> the CLI, angles are in radians and depth is in metres.

---

## Saved gains

`config/controller_params.yaml` — installed to the package share and loaded
by `nereo_controller.launch.py`'s `params_file` argument (D-03) — is the one
place tuning survives a restart and stays reviewable in git. It holds
exactly the D-03 key set: `kp`, `ki`, `kd`, `anti_windup_gains`,
`authority_cap` and the eleven `cs_*` parameters. It deliberately never
holds `control_mode` or any `manual_setpoint_*`/`setpoint_*` value, so a
restart always boots into passthrough instead of a stale manual depth
setpoint or straight into an active control mode.

Depth and attitude gains ship at **zero** (D-05) — there is no hand-derived
seed; they are found empirically with the tuner in a wet session. The
tuner's **Salva YAML** button writes the live parameter values it just read
from the node back to this file, through the resolved path (so a
`--symlink-install` build updates the tracked source file, not a stale copy
under `install/`).

---

## Topics

### Subscribed

| Topic | Type | Source |
|---|---|---|
| `/nereo_cmd_vel_no_fb` | `nereo_interfaces/CommandVelocity` | `joy_to_cmd_vel` (controller mode) |
| `/imu_data` | `sensor_msgs/Imu` | `imu_publisher` (RPi) |
| `/barometer_depth` | `std_msgs/Float32` | `bar_publisher` (RPi); metres, positive down, tare-relative, best-effort |

### Published

| Topic | Type | Consumer | Purpose |
|---|---|---|---|
| `/nereo_cmd_vel_ctrl` | `nereo_interfaces/CommandVelocity` | `safety_node` | Command after controller feedback; `safety_node` arbitrates the final `/nereo_cmd_vel` sent to the ROV firmware |
| `/controller/setpoints` | `std_msgs/Float64MultiArray` | TUNER GUI | Live `[depth, roll, pitch, yaw]` setpoints |
| `/controller/errors` | `std_msgs/Float64MultiArray` | TUNER GUI | Live error vector |
| `/controller/pid_terms` | `std_msgs/Float64MultiArray` | TUNER GUI | Per-axis P / I / D contributions |

### Parameter services

Standard ROS 2 parameter services are used by the TUNER:
- `/nereo_controller_node/get_parameters`
- `/nereo_controller_node/set_parameters`

---

## Tuning from the GUI

In the Nereo dashboard, click **TUNER** to open the controller window. You can:

- Switch `control_mode` from the dropdown
- Edit `kp / ki / kd` per axis
- Toggle manual setpoint on/off per axis
- Set `setpoint_depth` in **metres**, `setpoint_roll/pitch/yaw` in **degrees** (angle conversion is done in the GUI; the controller works in metres for depth, radians for angles)
- Edit the full CS section (`cs_kx0/1/2`, `cs_ki0/1/2`, heave/angle limits)
- Watch live `/controller/setpoints`, `/controller/errors`, `/controller/pid_terms` at the bottom

Press **RELOAD** to fetch the current parameter values from the node. Press **APPLY** to push the on-screen values back to the node.

---

## Troubleshooting

| Symptom | Cause | Fix |
|---|---|---|
| Workstation launch can't find `nereo_controller_node` | Overlay not built or in unexpected path | `colcon build` here; the launch file expects this repo at `~/Documents/PoliTOcean/RD/ros2_controller_tuning_aid` |
| TUNER shows "DISCONNESSO" | Controller node not running, or parameter services not yet up | Check `ros2 node list` for `/nereo_controller_node`; click RELOAD again |
| Controller doesn't react to joystick | Joystick is in *direct* mode | Press the **mode toggle** button on the joystick (Xbox View / DS5 Share) |
| Controller seems to drift | `manual_setpoint_*` is `false` and the axis is tracking the current value | Enable the manual toggle and set a fixed setpoint, or arm the ROV at the desired pose |
| Node refuses to start: parameter type | The params YAML has an int where the node expects a double (declared parameter types are strict) | Write every number in the YAML with a decimal point, e.g. `authority_cap: 1.0`, not `1` |
