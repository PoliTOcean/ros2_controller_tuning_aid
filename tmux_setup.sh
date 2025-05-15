#!/bin/bash

# Start new tmux session
tmux new-session -d -s nereo_test

# Create a 3x2 layout
tmux split-window -h
tmux split-window -h
tmux select-pane -t 0
tmux split-window -v
tmux select-pane -t 2
tmux split-window -v
tmux select-pane -t 4
tmux split-window -v

# Pane 0: Launch node with debug logging
tmux select-pane -t 0
tmux send-keys "cd ~/nereo_ws" C-m
tmux send-keys "source install/setup.bash" C-m
tmux send-keys "export RCUTILS_CONSOLE_OUTPUT_FORMAT='[{severity}] [{name}]: {message}'" C-m
tmux send-keys "export RCUTILS_LOGGING_USE_STDOUT=1" C-m
tmux send-keys "export RCUTILS_LOGGING_BUFFERED_STREAM=1" C-m
tmux send-keys "export RCUTILS_COLORIZED_OUTPUT=1" C-m
tmux send-keys "ros2 launch nereo_controller_node nereo_controller.launch.py control_mode:=1" C-m


# Pane 1: Publish IMU
tmux select-pane -t 1
tmux send-keys "cd ~/nereo_ws" C-m
tmux send-keys "source install/setup.bash" C-m
tmux send-keys "ros2 topic pub -r 10 /imu_data sensor_msgs/msg/Imu '{orientation: {w: 1, x: 0.0, y: 0.0, z: 0.0}}'" C-m

# Pane 2: Publish Pressure
tmux select-pane -t 2
tmux send-keys "cd ~/nereo_ws" C-m
tmux send-keys "source install/setup.bash" C-m
tmux send-keys "ros2 topic pub -r 10 /barometer_pressure sensor_msgs/msg/FluidPressure '{fluid_pressure: 101325.0}'" C-m

# Pane 3: Publish varying Command Velocity
tmux select-pane -t 3
tmux send-keys "cd ~/nereo_ws" C-m
tmux send-keys "source install/setup.bash" C-m
tmux send-keys "ros2 topic pub -r 1 /nereo_cmd_vel_no_fb nereo_interfaces/msg/CommandVelocity '{cmd_vel: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}'" C-m

# Pane 4: Echo output
tmux select-pane -t 4
tmux send-keys "cd ~/nereo_ws" C-m
tmux send-keys "source install/setup.bash" C-m
tmux send-keys "ros2 topic echo /nereo_cmd_vel" C-m

# Pane 5: Free terminal for manual commands
tmux select-pane -t 5
tmux send-keys "cd ~/nereo_ws" C-m
tmux send-keys "source install/setup.bash" C-m

# Pane 6: Monitor debug messages
tmux select-pane -t 6
tmux send-keys "cd ~/nereo_ws" C-m
tmux send-keys "source install/setup.bash" C-m
tmux send-keys "tmux kill-session -t nereo_test"
#tmux send-keys "ros2 run rqt_console rqt_console" C-m

# Select the free terminal
tmux select-pane -t 5

# Attach to session
tmux attach-session -t nereo_test

# Optional: Kill the session when done
# tmux kill-session -t nereo_test