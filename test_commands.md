## ROS 2 Test Commands

### Launch the Node

Puoi lanciare il nodo e, opzionalmente, impostare i parametri `control_mode`, `kp`, `ki`, `kd` e i parametri del CS Controller direttamente dal comando di launch:

```sh
ros2 launch nereo_controller_node nereo_controller.launch.py control_mode:=<valore> kp:="[<valori>]" ki:="[<valori>]" kd:="[<valori>]" cs_kx0:="[<v1>, <v2>]" cs_ki0:=<val> cs_heave_min:=<val> cs_heave_max:=<val>
```

Esempio:
```sh
ros2 launch nereo_controller_node nereo_controller.launch.py control_mode:=3 cs_kx0:="[200.0, 470.0]" cs_ki0:=-360.0 cs_heave_min:=-70.0 cs_heave_max:=90.0
```

### Launch Node + PID Tuner GUI

```sh
ros2 launch nereo_controller_node nereo_controller_with_gui.launch.py
```

Oppure, se il nodo e gia attivo:

```sh
ros2 run nereo_controller_node pid_tuner_gui.py --ros-args -p target_node:=/nereo_controller_node
```

### Publish Test Messages

- **Publish Pressure:**
    ```sh
    ros2 topic pub /barometer_pressure sensor_msgs/msg/FluidPressure "{fluid_pressure: 101325.0}"
    ```

- **Publish IMU Data:**
    ```sh
    ros2 topic pub /imu_data sensor_msgs/msg/Imu "{orientation: {w: 1, x: 0.0, y: 0.0, z: 0.0}}"
    ```

- **Publish Command Velocity (No Feedback):**
    ```sh
    ros2 topic pub /nereo_cmd_vel_no_fb nereo_interfaces/msg/CommandVelocity "{cmd_vel: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}"
    ```

### Check Output
```sh
ros2 topic echo /nereo_cmd_vel
```

### Run Automated Integration Tests

Dopo aver avviato il controller, esegui la suite di test automatica:

```sh
python3 src/nereo_controller_node/test/test_nereo_controller.py
```

Il comando restituisce `0` se tutti i test passano, `1` se almeno un test fallisce.

### Set Parameters

Set parameters for `/nereo_controller_node`:
```sh
ros2 param set /nereo_controller_node <param_name> <value>
```
Where `<param_name>` can be:
- `control_mode` (values: 0, 1, 2, 3)
- `kp`, `ki`, `kd` as arrays of floats:
    ```sh
    ros2 param set /nereo_controller_node kp "[0.0, 0.0, 0.0, 0.0]"
    ros2 param set /nereo_controller_node ki "[0.0, 0.0, 0.0, 0.0]"
    ros2 param set /nereo_controller_node kd "[0.0, 0.0, 0.0, 0.0]"
    ```
- **CS Controller parameters**:
    ```sh
    ros2 param set /nereo_controller_node cs_kx0 "[198.1, 468.1]"
    ros2 param set /nereo_controller_node cs_kx1 "[3.8, 6.0]"
    ros2 param set /nereo_controller_node cs_kx2 "[5.2, 11.8]"
    ros2 param set /nereo_controller_node cs_ki0 -359.2
    ros2 param set /nereo_controller_node cs_ki1 0.0
    ros2 param set /nereo_controller_node cs_ki2 -16.7
    ros2 param set /nereo_controller_node cs_heave_min -60.0
    ros2 param set /nereo_controller_node cs_heave_max 80.0
    ros2 param set /nereo_controller_node cs_angle_min -30.0
    ros2 param set /nereo_controller_node cs_angle_max 30.0
    ```