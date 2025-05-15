# Nereo Controller Node

This repository provides a ROS 2 node for controlling the **Nereo** underwater vehicle (ROV).  
The node reuses core control logic from the [nereo_firmware](https://github.com/PoliTOcean/nereo_FC_firmware) repository and acts as a "middleman" between ROS 2 and the low-level controller code.  
This design allows you to work on and tune the controllers at runtime using ROS 2 parameters, without the need to recompile the firmware or restart the node.

## Main Features

- **Receives velocity commands** (`/nereo_cmd_vel_no_fb`) and publishes controlled commands to `/nereo_cmd_vel`
- **Sensor feedback**: uses IMU data (`/imu_data`) and pressure sensor (`/barometer_pressure`)
- **Automatic setpoints**: maintains depth and orientation when no active commands are present
- **Runtime parameters**: all controller parameters (PID and CS) can be changed via the ROS 2 parameter server without recompiling
- **Dynamic tuning support**: controller gains and limits can be adjusted in real time

## Control Modes

- **0: DIRECT_PASSTHROUGH**  
  Commands are forwarded directly without feedback.
- **1: PID_CONTROL**  
  Uses PID controllers for depth, roll, pitch, and yaw.
- **2: PID_ANTI_WINDUP**  
  PID with anti-windup on integrators.
- **3: CS_CONTROLLER**  
  State-space controllers for depth, roll, and pitch; PID for yaw.

## Main Parameters

- `control_mode`: selects the control mode (0-3)
- `kp`, `ki`, `kd`: PID gains (arrays)
- `cs_kx0`, `cs_kx1`, `cs_kx2`: state feedback gains for CS controller (heave, roll, pitch)
- `cs_ki0`, `cs_ki1`, `cs_ki2`: integral gains for CS controller
- `cs_heave_min`, `cs_heave_max`, `cs_angle_min`, `cs_angle_max`: saturation limits for CS controllers

## Usage

1. **Launch the node** (see [test_commands.md](./test_commands.md) for examples)
2. **Publish test data** to `/barometer_pressure`, `/imu_data`, and `/nereo_cmd_vel_no_fb`
3. **Change parameters** at runtime using `ros2 param set` for dynamic tuning
4. **Monitor output** on `/nereo_cmd_vel`

## Notes

- The core control algorithms are reused from the [nereo_firmware](https://github.com/PoliTOcean/nereo_FC_firmware) project.
- For test and command details, see [test_commands.md](./test_commands.md).
