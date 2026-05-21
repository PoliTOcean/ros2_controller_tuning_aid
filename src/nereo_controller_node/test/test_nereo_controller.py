#!/usr/bin/env python3

import sys
import time
from dataclasses import dataclass
from typing import List, Sequence

import rclpy
from nereo_interfaces.msg import CommandVelocity
from rcl_interfaces.msg import Parameter as ParameterMsg
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.srv import SetParameters
from rclpy.node import Node
from sensor_msgs.msg import FluidPressure, Imu


@dataclass
class TestResult:
    name: str
    passed: bool
    detail: str


class NereoControllerTester(Node):
    def __init__(self):
        super().__init__("nereo_controller_tester")

        self.cmd_vel_pub = self.create_publisher(CommandVelocity, "/nereo_cmd_vel_no_fb", 10)
        self.imu_pub = self.create_publisher(Imu, "/imu_data", 10)
        self.pressure_pub = self.create_publisher(FluidPressure, "/barometer_pressure", 10)
        self.cmd_vel_sub = self.create_subscription(
            CommandVelocity,
            "/nereo_cmd_vel",
            self.cmd_vel_callback,
            10,
        )

        self.latest_cmd_vel = None
        self.response_seq = 0

        self.parameter_client = self.create_client(SetParameters, "/nereo_controller_node/set_parameters")
        self.get_logger().info("Nereo Controller Tester initialized")

    def cmd_vel_callback(self, msg: CommandVelocity):
        self.latest_cmd_vel = msg
        self.response_seq += 1

    def spin_for(self, duration_sec: float):
        end_t = time.time() + duration_sec
        while time.time() < end_t:
            rclpy.spin_once(self, timeout_sec=0.05)

    def wait_for_new_response(self, previous_seq: int, timeout_sec: float = 3.0) -> CommandVelocity:
        end_t = time.time() + timeout_sec
        while time.time() < end_t:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.response_seq > previous_seq and self.latest_cmd_vel is not None:
                return self.latest_cmd_vel
        raise TimeoutError("Timeout waiting for /nereo_cmd_vel response")

    def _call_set_parameters(self, parameters: Sequence[ParameterMsg]):
        while not self.parameter_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for /set_parameters service...")

        req = SetParameters.Request()
        req.parameters = list(parameters)
        future = self.parameter_client.call_async(req)

        end_t = time.time() + 3.0
        while time.time() < end_t:
            rclpy.spin_once(self, timeout_sec=0.05)
            if future.done():
                result = future.result()
                if result is None:
                    raise RuntimeError("set_parameters returned no result")
                for p_res in result.results:
                    if not p_res.successful:
                        raise RuntimeError(f"Parameter rejected: {p_res.reason}")
                return
        raise TimeoutError("Timeout waiting for set_parameters response")

    def set_control_mode(self, mode: int):
        p = ParameterMsg()
        p.name = "control_mode"
        p.value.type = ParameterType.PARAMETER_INTEGER
        p.value.integer_value = mode
        self._call_set_parameters([p])
        # Controller applies parameter updates in its 1 Hz timer callback.
        self.spin_for(1.2)

    def set_pid_gains(self, kp: Sequence[float], ki: Sequence[float], kd: Sequence[float]):
        p_kp = ParameterMsg()
        p_kp.name = "kp"
        p_kp.value.type = ParameterType.PARAMETER_DOUBLE_ARRAY
        p_kp.value.double_array_value = list(kp)

        p_ki = ParameterMsg()
        p_ki.name = "ki"
        p_ki.value.type = ParameterType.PARAMETER_DOUBLE_ARRAY
        p_ki.value.double_array_value = list(ki)

        p_kd = ParameterMsg()
        p_kd.name = "kd"
        p_kd.value.type = ParameterType.PARAMETER_DOUBLE_ARRAY
        p_kd.value.double_array_value = list(kd)

        self._call_set_parameters([p_kp, p_ki, p_kd])
        # Controller applies parameter updates in its 1 Hz timer callback.
        self.spin_for(1.2)

    def set_manual_depth_setpoint(self, enabled: bool, depth_value: float):
        p_enable = ParameterMsg()
        p_enable.name = "manual_setpoint_depth"
        p_enable.value.type = ParameterType.PARAMETER_BOOL
        p_enable.value.bool_value = enabled

        p_depth = ParameterMsg()
        p_depth.name = "setpoint_depth"
        p_depth.value.type = ParameterType.PARAMETER_DOUBLE
        p_depth.value.double_value = depth_value

        self._call_set_parameters([p_enable, p_depth])
        # Controller applies parameter updates in its 1 Hz timer callback.
        self.spin_for(1.2)

    def publish_imu(self, w: float, x: float, y: float, z: float):
        msg = Imu()
        msg.orientation.w = w
        msg.orientation.x = x
        msg.orientation.y = y
        msg.orientation.z = z
        self.imu_pub.publish(msg)

    def publish_pressure(self, pressure_pa: float):
        msg = FluidPressure()
        msg.fluid_pressure = pressure_pa
        self.pressure_pub.publish(msg)

    def publish_cmd_vel(self, surge: float, sway: float, heave: float, roll: float, pitch: float, yaw: float):
        msg = CommandVelocity()
        msg.cmd_vel[0] = surge
        msg.cmd_vel[1] = sway
        msg.cmd_vel[2] = heave
        msg.cmd_vel[3] = roll
        msg.cmd_vel[4] = pitch
        msg.cmd_vel[5] = yaw
        self.cmd_vel_pub.publish(msg)

    def send_and_wait(self, cmd: Sequence[float], timeout_sec: float = 3.0) -> CommandVelocity:
        previous_seq = self.response_seq
        self.publish_cmd_vel(*cmd)
        return self.wait_for_new_response(previous_seq, timeout_sec=timeout_sec)

    @staticmethod
    def _assert_close(actual: float, expected: float, tol: float, label: str):
        if abs(actual - expected) > tol:
            raise AssertionError(
                f"{label}: expected {expected:.4f}, got {actual:.4f}, tol={tol:.4f}"
            )

    @staticmethod
    def _assert_true(condition: bool, message: str):
        if not condition:
            raise AssertionError(message)

    def _prime_sensors(self):
        self.publish_imu(1.0, 0.0, 0.0, 0.0)
        self.publish_pressure(101325.0)
        self.spin_for(0.2)

    def test_passthrough(self):
        self.set_control_mode(0)
        self._prime_sensors()
        out = self.send_and_wait([0.5, -0.3, 0.2, 0.1, -0.1, 0.2])

        expected = [0.5, -0.3, 0.2, 0.1, -0.1, 0.2]
        for i, axis in enumerate(["surge", "sway", "heave", "roll", "pitch", "yaw"]):
            self._assert_close(out.cmd_vel[i], expected[i], 0.02, f"passthrough {axis}")

    def test_pid_roll_correction(self):
        self.set_pid_gains([0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0])
        self.set_control_mode(1)

        self.publish_imu(1.0, 0.0, 0.0, 0.0)
        self.publish_pressure(101325.0)
        self.spin_for(0.2)

        # First command updates setpoint to current roll (0 rad).
        self.send_and_wait([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

        # Simulate positive roll, expect negative corrective command on roll axis.
        self.publish_imu(0.99875026, 0.04997917, 0.0, 0.0)
        self.spin_for(0.2)
        out = self.send_and_wait([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

        self._assert_true(out.cmd_vel[3] < -0.03, f"roll correction should be negative, got {out.cmd_vel[3]:.4f}")

    def test_pid_depth_feedback_on_heave(self):
        self.set_pid_gains([1.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0])
        self.set_control_mode(1)
        self.set_manual_depth_setpoint(True, 101325.0)

        try:
            self.publish_imu(1.0, 0.0, 0.0, 0.0)
            self.publish_pressure(101325.0)
            self.spin_for(0.2)
            self.send_and_wait([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

            # Increase pressure: depth error should produce negative heave correction.
            self.publish_pressure(104325.0)
            self.spin_for(0.25)

            min_heave = 0.0
            for _ in range(3):
                out = self.send_and_wait([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
                min_heave = min(min_heave, out.cmd_vel[2])
                self.spin_for(0.1)

            self._assert_true(
                min_heave < -0.05,
                f"heave correction should be negative after pressure increase, min observed={min_heave:.4f}",
            )
        finally:
            self.set_manual_depth_setpoint(False, 0.0)

    def test_setpoint_update_on_zero_command(self):
        self.set_pid_gains([0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0])
        self.set_control_mode(1)

        self.publish_imu(1.0, 0.0, 0.0, 0.0)
        self.publish_pressure(101325.0)
        self.spin_for(0.2)

        # Keep a non-zero roll command: setpoint should not update.
        self.send_and_wait([0.0, 0.0, 0.0, 0.2, 0.0, 0.0])

        # Move vehicle to about +0.2 rad roll.
        self.publish_imu(0.99500417, 0.09983342, 0.0, 0.0)
        self.spin_for(0.2)

        # First zero command updates setpoint to current roll.
        self.send_and_wait([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

        # Second zero command should now require almost no roll correction.
        out = self.send_and_wait([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self._assert_true(abs(out.cmd_vel[3]) < 0.08, f"roll should be near zero after setpoint update, got {out.cmd_vel[3]:.4f}")


def run_test_case(tester: NereoControllerTester, name: str, fn) -> TestResult:
    try:
        fn()
        return TestResult(name=name, passed=True, detail="ok")
    except Exception as exc:  # noqa: BLE001
        return TestResult(name=name, passed=False, detail=str(exc))


def main() -> int:
    rclpy.init()
    tester = NereoControllerTester()

    tests = [
        ("passthrough", tester.test_passthrough),
        ("pid_roll_correction", tester.test_pid_roll_correction),
        ("pid_depth_feedback_on_heave", tester.test_pid_depth_feedback_on_heave),
        ("setpoint_update_on_zero_command", tester.test_setpoint_update_on_zero_command),
    ]

    results: List[TestResult] = []
    tester.get_logger().info("Starting integration test suite for nereo_controller_node")

    for test_name, fn in tests:
        tester.get_logger().info(f"RUN {test_name}")
        results.append(run_test_case(tester, test_name, fn))
        tester.spin_for(0.15)

    passed = sum(1 for r in results if r.passed)
    failed = len(results) - passed

    for r in results:
        if r.passed:
            tester.get_logger().info(f"PASS {r.name}")
        else:
            tester.get_logger().error(f"FAIL {r.name}: {r.detail}")

    tester.get_logger().info(f"Test summary: {passed}/{len(results)} passed, {failed} failed")

    tester.destroy_node()
    rclpy.shutdown()
    return 0 if failed == 0 else 1


if __name__ == "__main__":
    sys.exit(main())