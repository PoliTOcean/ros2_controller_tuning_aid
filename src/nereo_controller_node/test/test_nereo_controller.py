#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nereo_interfaces.msg import CommandVelocity
from sensor_msgs.msg import Imu, FluidPressure
import math
import time
from rclpy.parameter import Parameter
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import ParameterValue, ParameterType, Parameter as ParameterMsg

class NereoControllerTester(Node):
    def __init__(self):
        super().__init__('nereo_controller_tester')
        
        # Create publishers
        self.cmd_vel_pub = self.create_publisher(
            CommandVelocity,
            '/nereo_cmd_vel_no_fb',
            10
        )
        
        self.imu_pub = self.create_publisher(
            Imu,
            '/imu_data',
            10
        )
        
        self.pressure_pub = self.create_publisher(
            FluidPressure,
            '/barometer_pressure',
            10
        )
        
        # Create subscription to receive response
        self.cmd_vel_sub = self.create_subscription(
            CommandVelocity,
            '/nereo_cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        self.response_received = False
        self.latest_cmd_vel = None
        
        # Parameter client
        self.parameter_client = self.create_client(
            SetParameters,
            '/nereo_controller_node/set_parameters'
        )
        
        self.get_logger().info('Nereo Controller Tester initialized')
        
    def cmd_vel_callback(self, msg):
        self.response_received = True
        self.latest_cmd_vel = msg
        self.get_logger().info(f'Received cmd_vel: surge={msg.cmd_vel[0]:.3f}, sway={msg.cmd_vel[1]:.3f}, heave={msg.cmd_vel[2]:.3f}, roll={msg.cmd_vel[3]:.3f}, pitch={msg.cmd_vel[4]:.3f}, yaw={msg.cmd_vel[5]:.3f}')
        
    def wait_for_response(self, timeout=5.0):
        self.response_received = False
        start_time = time.time()
        
        while not self.response_received:
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.time() - start_time > timeout:
                self.get_logger().error('Timeout waiting for response')
                return False
                
        return True
        
    def set_control_mode(self, mode):
        self.get_logger().info(f'Setting control mode to {mode}')
        while not self.parameter_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for parameter service...')
            
        request = SetParameters.Request()
        parameter = ParameterMsg()
        parameter.name = 'control_mode'
        parameter.value.type = ParameterType.PARAMETER_INTEGER
        parameter.value.integer_value = mode
        request.parameters = [parameter]
        
        self.parameter_client.call_async(request)
        time.sleep(0.5)  # Give time for parameter to take effect
        
    def set_pid_gains(self, kp, ki, kd):
        self.get_logger().info(f'Setting PID gains - kp: {kp}, ki: {ki}, kd: {kd}')
        while not self.parameter_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for parameter service...')
            
        request = SetParameters.Request()
        
        # KP
        param_kp = ParameterMsg()
        param_kp.name = 'kp'
        param_kp.value.type = ParameterType.PARAMETER_DOUBLE_ARRAY
        param_kp.value.double_array_value = kp
        
        # KI
        param_ki = ParameterMsg()
        param_ki.name = 'ki'
        param_ki.value.type = ParameterType.PARAMETER_DOUBLE_ARRAY
        param_ki.value.double_array_value = ki
        
        # KD
        param_kd = ParameterMsg()
        param_kd.name = 'kd'
        param_kd.value.type = ParameterType.PARAMETER_DOUBLE_ARRAY
        param_kd.value.double_array_value = kd
        
        request.parameters = [param_kp, param_ki, param_kd]
        
        self.parameter_client.call_async(request)
        time.sleep(0.5)  # Give time for parameter to take effect
        
    def publish_imu(self, w, x, y, z):
        msg = Imu()
        msg.orientation.w = w
        msg.orientation.x = x
        msg.orientation.y = y
        msg.orientation.z = z
        
        self.imu_pub.publish(msg)
        self.get_logger().info(f'Published IMU data: w={w}, x={x}, y={y}, z={z}')
        time.sleep(0.2)  # Give time for message to be processed
        
    def publish_pressure(self, pressure):
        msg = FluidPressure()
        msg.fluid_pressure = pressure
        
        self.pressure_pub.publish(msg)
        self.get_logger().info(f'Published pressure: {pressure} Pa')
        time.sleep(0.2)  # Give time for message to be processed
        
    def publish_cmd_vel(self, surge, sway, heave, roll, pitch, yaw):
        msg = CommandVelocity()
        msg.cmd_vel[0] = surge
        msg.cmd_vel[1] = sway
        msg.cmd_vel[2] = heave
        msg.cmd_vel[3] = roll
        msg.cmd_vel[4] = pitch
        msg.cmd_vel[5] = yaw
    
        self.cmd_vel_pub.publish(msg)
        self.get_logger().info(f'Published cmd_vel: surge={surge}, sway={sway}, heave={heave}, roll={roll}, pitch={pitch}, yaw={yaw}')

        
    def run_test_1_passthrough(self):
        """Test 1: Passthrough mode"""
        self.get_logger().info('\n\n--- RUNNING TEST 1: PASSTHROUGH MODE ---')
        
        # Set mode to passthrough
        self.set_control_mode(0)
        
        # Publish neutral orientation and standard pressure
        self.publish_imu(1.0, 0.0, 0.0, 0.0)
        self.publish_pressure(101325.0)
        
        # Publish command velocity
        self.publish_cmd_vel(0.5, -0.3, 0.2, 0.1, -0.1, 0.2)
        
        # Wait for response
        if self.wait_for_response():
            # Check that values are the same
            expected = [0.5, -0.3, 0.2, 0.1, -0.1, 0.2]
            actual = [
                self.latest_cmd_vel.cmd_vel[0],
                self.latest_cmd_vel.cmd_vel[1],
                self.latest_cmd_vel.cmd_vel[2],
                self.latest_cmd_vel.cmd_vel[3],
                self.latest_cmd_vel.cmd_vel[4],
                self.latest_cmd_vel.cmd_vel[5]
            ]
            
            for i, name in enumerate(['surge', 'sway', 'heave', 'roll', 'pitch', 'yaw']):
                if abs(expected[i] - actual[i]) < 0.01:
                    self.get_logger().info(f'✓ {name} = {expected[i]} matches {name} = {actual[i]}')
                else:
                    self.get_logger().error(f'✗ {name} = {expected[i]} does not match {name} = {actual[i]}')
                    
            self.get_logger().info('Test 1 completed')
            
    def run_test_2_basic_pid(self):
        """Test 2: Basic PID with P gain only"""
        self.get_logger().info('\n\n--- RUNNING TEST 2: BASIC PID ---')
        
        # Set PID gains (P only)
        self.set_pid_gains([1.0, 1.0, 1.0, 1.0], [0.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0])
        
        # Set mode to PID control
        self.set_control_mode(1)
        
        # Part 1: Neutral orientation
        self.get_logger().info('Part 1: Neutral orientation')
        
        # Publish neutral orientation and standard pressure
        self.publish_imu(1.0, 0.0, 0.0, 0.0)
        self.publish_pressure(101325.0)
        
        # Publish zero command velocity to initialize setpoints
        self.publish_cmd_vel(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        
        # Wait for response
        if self.wait_for_response():
            # Check values (should be close to zero)
            actual = [
                self.latest_cmd_vel.cmd_vel[0],
                self.latest_cmd_vel.cmd_vel[1],
                self.latest_cmd_vel.cmd_vel[2],
                self.latest_cmd_vel.cmd_vel[3],
                self.latest_cmd_vel.cmd_vel[4],
                self.latest_cmd_vel.cmd_vel[5]
            ]
            
            for i, name in enumerate(['surge', 'sway', 'heave', 'roll', 'pitch', 'yaw']):
                if abs(actual[i]) < 0.1:
                    self.get_logger().info(f'✓ {name} = {actual[i]} is close to zero')
                else:
                    self.get_logger().error(f'✗ {name} = {actual[i]} is not close to zero')
        
        # Part 2: Roll error of 0.1 rad
        self.get_logger().info('Part 2: Roll error of 0.1 rad')
        
        # Publish orientation with roll of 0.1 rad
        self.publish_imu(0.9987, 0.05, 0.0, 0.0)
        
        # Publish zero command velocity again
        self.publish_cmd_vel(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        
        # Wait for response
        if self.wait_for_response():
            # Check roll correction (should be approximately -0.1)
            if abs(self.latest_cmd_vel.cmd_vel[3] + 0.1) < 0.02:
                self.get_logger().info(f'✓ roll = {self.latest_cmd_vel.cmd_vel[3]} is close to expected -0.1')
            else:
                self.get_logger().error(f'✗ roll = {self.latest_cmd_vel.cmd_vel[3]} is not close to expected -0.1')
                
            self.get_logger().info('Test 2 completed')
            
    def run_test_3_pid_anti_windup(self):
        """Test 3: PID with anti-windup"""
        self.get_logger().info('\n\n--- RUNNING TEST 3: PID WITH ANTI-WINDUP ---')
        
        # Set PID gains (P and I)
        self.set_pid_gains([1.0, 1.0, 1.0, 1.0], [1.0, 1.0, 1.0, 1.0], [0.0, 0.0, 0.0, 0.0])
        
        # Set mode to PID with anti-windup
        self.set_control_mode(2)
        
        # Publish neutral orientation and standard pressure
        self.publish_imu(1.0, 0.0, 0.0, 0.0)
        self.publish_pressure(101325.0)
        
        # Publish zero command velocity to initialize setpoints
        self.publish_cmd_vel(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        self.wait_for_response()
        
        # Publish orientation with pitch of 0.5 rad
        self.publish_imu(0.9689, 0.0, 0.2474, 0.0)
        
        # Publish zero command velocity again
        self.publish_cmd_vel(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        
            # Wait for response
        if self.wait_for_response():
            # First update should have pitch correction (should be around -1.0 due to limit)
            if abs(self.latest_cmd_vel.cmd_vel[4] + 1.0) < 0.1:
                self.get_logger().info(f'✓ pitch = {self.latest_cmd_vel.cmd_vel[4]} is close to expected -1.0')
            else:
                self.get_logger().error(f'✗ pitch = {self.latest_cmd_vel.cmd_vel[4]} is not close to expected -1.0')
        
        # Do a few more iterations to see if anti-windup keeps value within bounds
        for i in range(5):
            self.publish_cmd_vel(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
            self.wait_for_response()
            
         # Check that pitch correction is still within bounds
        if abs(self.latest_cmd_vel.cmd_vel[4]) <= 1.0:
            self.get_logger().info(f'✓ pitch = {self.latest_cmd_vel.cmd_vel[4]} is within bounds [-1.0, 1.0]')
        else:
            self.get_logger().error(f'✗ pitch = {self.latest_cmd_vel.cmd_vel[4]} is outside bounds [-1.0, 1.0]')
            
        self.get_logger().info('Test 3 completed')
        
    def run_test_4_reference_transformation(self):
        """Test 4: Reference frame transformation"""
        self.get_logger().info('\n\n--- RUNNING TEST 4: REFERENCE FRAME TRANSFORMATION ---')
        
        # Set PID gains (only depth control)
        self.set_pid_gains([1.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0])
        
        # Set mode to PID control
        self.set_control_mode(1)
        
        # Publish 90 degree pitch orientation
        self.publish_imu(0.7071, 0.0, 0.7071, 0.0)
        
        # Publish initial pressure
        self.publish_pressure(101325.0)
        
        # Publish zero command velocity to initialize setpoints
        self.publish_cmd_vel(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        self.wait_for_response()
        
        # Increase pressure (as if ROV is descending)
        self.publish_pressure(102325.0)
        
        # Publish zero command velocity again
        self.publish_cmd_vel(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        
         # Wait for response
        if self.wait_for_response():
            # Check that depth correction is being applied to surge (x) axis
            if abs(self.latest_cmd_vel.cmd_vel[0]) > 0.5:
                self.get_logger().info(f'✓ surge = {self.latest_cmd_vel.cmd_vel[0]} has depth correction applied')
            else:
                self.get_logger().error(f'✗ surge = {self.latest_cmd_vel.cmd_vel[0]} does not have depth correction applied')
                
            if abs(self.latest_cmd_vel.cmd_vel[2]) < 0.1:
                self.get_logger().info(f'✓ heave = {self.latest_cmd_vel.cmd_vel[2]} has little to no correction')
            else:
                self.get_logger().error(f'✗ heave = {self.latest_cmd_vel.cmd_vel[2]} has unexpected correction')
                
            self.get_logger().info('Test 4 completed')
            
    def run_test_5_setpoint_update(self):
        """Test 5: Setpoint update when command goes from non-zero to zero"""
        self.get_logger().info('\n\n--- RUNNING TEST 5: SETPOINT UPDATE ---')
        
        # Set PID gains (P only)
        self.set_pid_gains([1.0, 1.0, 1.0, 1.0], [0.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0])
        
        # Set mode to PID control
        self.set_control_mode(1)
        
        # Publish neutral orientation
        self.publish_imu(1.0, 0.0, 0.0, 0.0)
        
        # Publish standard pressure
        self.publish_pressure(101325.0)
        
        # Publish non-zero roll command
        self.publish_cmd_vel(0.0, 0.0, 0.0, 0.2, 0.0, 0.0)
        self.wait_for_response()
        
        # Change orientation to roll of 0.2 rad
        self.publish_imu(0.995, 0.1, 0.0, 0.0)
        
        # Publish zero roll command
        self.publish_cmd_vel(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        
        # Wait for response
        if self.wait_for_response():
            # Roll correction should be near zero since setpoint was updated to current roll
            if abs(self.latest_cmd_vel.cmd_vel[3]) < 0.05:
                self.get_logger().info(f'✓ roll = {self.latest_cmd_vel.cmd_vel[3]} is close to zero after setpoint update')
            else:
                self.get_logger().error(f'✗ roll = {self.latest_cmd_vel.cmd_vel[3]} is not close to zero after setpoint update')
            self.get_logger().info('Test 5 completed')
        else:
            self.get_logger().error('Test 5 failed: No response received after setpoint update')
            
    def run_all_tests(self):
        self.run_test_1_passthrough()
        self.run_test_2_basic_pid()
        self.run_test_3_pid_anti_windup()
        self.run_test_4_reference_transformation()
        self.run_test_5_setpoint_update()
        self.get_logger().info('All tests completed')
        rclpy.shutdown()
        self.get_logger().info('Shutting down...')

        # Clean up
        self.cmd_vel_pub.destroy()
        self.imu_pub.destroy()
        self.pressure_pub.destroy()
        self.cmd_vel_sub.destroy()
        self.parameter_client.destroy()
        self.destroy_node()
        self.get_logger().info('Nereo Controller Tester destroyed')
        self.get_logger().info('All resources cleaned up')
        self.get_logger().info('Exiting...')
        # Exit the program
        exit(0)


if __name__ == '__main__':
    rclpy.init()
    tester = NereoControllerTester()
    tester.run_all_tests()
    #tester.run_test_3_pid_anti_windup()
    rclpy.spin(tester)
    tester.destroy_node()
    rclpy.shutdown()