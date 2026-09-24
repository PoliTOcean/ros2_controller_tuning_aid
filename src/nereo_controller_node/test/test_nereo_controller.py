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
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32

# Task 1 decision (2026-09-24, operator): positive heave ascends. This
# matches the joystick mapping (right stick up -> +cmd_vel[2]), the web
# joystick's "up -> positive heave" comment, and the bench-verified
# vertical thruster rows. The controller's depthError() is therefore
# current_depth_m_ - setpoints_[0]: a vehicle deeper than its setpoint
# gets a positive (ascending) heave correction.
HEAVE_SIGN = 1


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
        self.depth_pub = self.create_publisher(Float32, "/barometer_depth", 10)
        self.cmd_vel_sub = self.create_subscription(
            CommandVelocity,
            "/nereo_cmd_vel_ctrl",
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
        raise TimeoutError("Timeout waiting for /nereo_cmd_vel_ctrl response")

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

    def set_params(self, values: dict):
        """Build one SetParameters call from plain Python types and wait
        for the controller's 1 Hz parameter poll to pick it up.

        bool -> PARAMETER_BOOL, int -> PARAMETER_INTEGER,
        float -> PARAMETER_DOUBLE, list -> PARAMETER_DOUBLE_ARRAY of floats.
        """
        params = []
        for name, value in values.items():
            p = ParameterMsg()
            p.name = name
            if isinstance(value, bool):
                p.value.type = ParameterType.PARAMETER_BOOL
                p.value.bool_value = value
            elif isinstance(value, int):
                p.value.type = ParameterType.PARAMETER_INTEGER
                p.value.integer_value = value
            elif isinstance(value, float):
                p.value.type = ParameterType.PARAMETER_DOUBLE
                p.value.double_value = value
            elif isinstance(value, list):
                p.value.type = ParameterType.PARAMETER_DOUBLE_ARRAY
                p.value.double_array_value = [float(v) for v in value]
            else:
                raise TypeError(f"Unsupported parameter type for {name}: {type(value)}")
            params.append(p)

        self._call_set_parameters(params)
        # Controller applies parameter updates in its 1 Hz timer callback.
        self.spin_for(1.2)

    def set_control_mode(self, mode: int):
        self.set_params({"control_mode": mode})

    def set_pid_gains(self, kp: Sequence[float], ki: Sequence[float], kd: Sequence[float]):
        self.set_params({"kp": list(kp), "ki": list(ki), "kd": list(kd)})

    def set_manual_depth_setpoint(self, enabled: bool, depth_value: float):
        self.set_params({"manual_setpoint_depth": enabled, "setpoint_depth": depth_value})

    def _assert_within_cap(self, out: CommandVelocity, pilot: Sequence[float], cap: float, label: str):
        for i, axis in enumerate(["surge", "sway", "heave", "roll", "pitch", "yaw"]):
            diff = abs(out.cmd_vel[i] - pilot[i])
            self._assert_true(
                diff <= cap + 0.001,
                f"{label} {axis}: correction {diff:.4f} exceeds cap {cap:.4f}",
            )

    def publish_imu(self, w: float, x: float, y: float, z: float):
        msg = Imu()
        msg.orientation.w = w
        msg.orientation.x = x
        msg.orientation.y = y
        msg.orientation.z = z
        self.imu_pub.publish(msg)

    def publish_depth(self, depth_m: float):
        msg = Float32()
        msg.data = depth_m
        self.depth_pub.publish(msg)

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
        self.publish_depth(0.0)
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
        self.publish_depth(0.0)
        self.spin_for(0.2)

        # First command updates setpoint to current roll (0 rad).
        self.send_and_wait([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

        # Simulate positive roll, expect negative corrective command on roll axis.
        self.publish_imu(0.99875026, 0.04997917, 0.0, 0.0)
        self.spin_for(0.2)
        out = self.send_and_wait([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

        self._assert_true(out.cmd_vel[3] < -0.03, f"roll correction should be negative, got {out.cmd_vel[3]:.4f}")

    def test_depth_metres_heave(self):
        # D-07/DEPTH-08: known Kp and known metre error give a known heave
        # command, with the sign pinned by the Task 1 decision (HEAVE_SIGN).
        # setpoint_depth 1.0 is unique in this suite so the controller's
        # setpoint-change reset starts this test from zero PID state.
        self.set_params({
            "kp": [0.5, 0.0, 0.0, 0.0],
            "ki": [0.0, 0.0, 0.0, 0.0],
            "kd": [0.0, 0.0, 0.0, 0.0],
            "control_mode": 1,
            "manual_setpoint_depth": True,
            "setpoint_depth": 1.0,
        })

        try:
            self.publish_imu(1.0, 0.0, 0.0, 0.0)

            self.publish_depth(1.0)
            self.spin_for(0.2)
            out = self.send_and_wait([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
            self._assert_close(out.cmd_vel[2], 0.0, 0.005, "heave at setpoint")

            self.publish_depth(1.4)
            self.spin_for(0.2)
            out = self.send_and_wait([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
            self._assert_close(
                out.cmd_vel[2], HEAVE_SIGN * 0.2, 0.005, "heave when deeper than setpoint"
            )

            self.publish_depth(0.6)
            self.spin_for(0.2)
            out = self.send_and_wait([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
            self._assert_close(
                out.cmd_vel[2], -HEAVE_SIGN * 0.2, 0.005, "heave when shallower than setpoint"
            )
        finally:
            self.set_params({"manual_setpoint_depth": False})

    def test_authority_cap(self):
        # DEPTH-10/D-08: authority_cap bounds only the correction the
        # controller adds, saturating exactly at the cap, never the
        # pilot's own command component.
        self.set_params({
            "kp": [1.0, 0.0, 0.0, 0.0],
            "ki": [0.0, 0.0, 0.0, 0.0],
            "kd": [0.0, 0.0, 0.0, 0.0],
            "control_mode": 1,
            "authority_cap": 0.25,
            "manual_setpoint_depth": True,
            "setpoint_depth": 0.0,
        })

        try:
            self.publish_imu(1.0, 0.0, 0.0, 0.0)

            # Above the cap: saturates.
            self.publish_depth(2.0)
            self.spin_for(0.2)
            out = self.send_and_wait([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
            self._assert_close(out.cmd_vel[2], HEAVE_SIGN * 0.25, 0.001, "heave saturates above cap")
            self._assert_within_cap(out, [0.0] * 6, 0.25, "above cap")

            # Exactly at the cap: unchanged.
            self.publish_depth(0.25)
            self.spin_for(0.2)
            out = self.send_and_wait([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
            self._assert_close(out.cmd_vel[2], HEAVE_SIGN * 0.25, 0.001, "heave exactly at cap")
            self._assert_within_cap(out, [0.0] * 6, 0.25, "at cap")

            # One step below the cap: unchanged.
            self.publish_depth(0.2)
            self.spin_for(0.2)
            out = self.send_and_wait([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
            self._assert_close(out.cmd_vel[2], HEAVE_SIGN * 0.2, 0.001, "heave one step below cap")
            self._assert_within_cap(out, [0.0] * 6, 0.25, "below cap")

            # Pilot authority: surge/sway never carry depth feedback under
            # an identity orientation, so they pass through unmodified
            # even while a capped depth correction is active on heave.
            pilot = [0.9, 0.0, 0.9, 0.0, 0.0, 0.0]
            out = self.send_and_wait(pilot)
            self._assert_close(out.cmd_vel[0], 0.9, 0.001, "pilot surge passes through unmodified")
            self._assert_within_cap(out, pilot, 0.25, "pilot command plus capped correction")

            # Depth back at setpoint: the correction is exactly zero, so
            # the pilot's own heave command also comes back unmodified.
            self.publish_depth(0.0)
            self.spin_for(0.2)
            out = self.send_and_wait(pilot)
            self._assert_close(out.cmd_vel[0], 0.9, 0.001, "pilot surge passes through unmodified")
            self._assert_close(out.cmd_vel[2], 0.9, 0.001, "pilot heave passes through unmodified at zero error")

            # authority_cap 0.0 disables feedback entirely (fail toward
            # zero authority, never toward more).
            self.set_params({"authority_cap": 0.0})
            self.publish_depth(2.0)
            self.spin_for(0.2)
            out = self.send_and_wait([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
            self._assert_close(out.cmd_vel[2], 0.0, 0.001, "authority_cap 0.0 disables feedback")

            # Out-of-range values are rejected by the FloatingPointRange.
            rejected = False
            try:
                self.set_params({"authority_cap": -0.01})
            except RuntimeError:
                rejected = True
            self._assert_true(rejected, "authority_cap -0.01 should be rejected")

            rejected = False
            try:
                self.set_params({"authority_cap": 1.01})
            except RuntimeError:
                rejected = True
            self._assert_true(rejected, "authority_cap 1.01 should be rejected")
        finally:
            self.set_params({"authority_cap": 1.0, "manual_setpoint_depth": False})

    def test_mode_change_resets_integrator(self):
        # DEPTH-09: a mode change must zero every integrator and re-capture
        # non-manual setpoints, so the first cycle after switching modes is
        # neither wound up nor built on a stale setpoint. setpoint_depth 1.5
        # is unique in this suite for the same reset-isolation reason as
        # test_depth_metres_heave.
        self.set_params({
            "kp": [0.0, 0.0, 0.0, 0.0],
            "ki": [0.1, 0.0, 0.0, 0.0],
            "kd": [0.0, 0.0, 0.0, 0.0],
            "control_mode": 1,
            "manual_setpoint_depth": True,
            "setpoint_depth": 1.5,
        })

        try:
            self.publish_imu(1.0, 0.0, 0.0, 0.0)
            self.publish_depth(2.5)
            self.spin_for(0.2)

            out = None
            for _ in range(5):
                out = self.send_and_wait([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
            self._assert_true(
                abs(out.cmd_vel[2]) >= 0.45,
                f"integrator should have wound up, got heave={out.cmd_vel[2]:.4f}",
            )

            self.set_params({"control_mode": 2})
            out = self.send_and_wait([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
            self._assert_close(
                out.cmd_vel[2],
                HEAVE_SIGN * 0.1,
                0.02,
                "heave on first cycle after mode change (must not carry windup)",
            )
        finally:
            self.set_params({"manual_setpoint_depth": False, "control_mode": 1})

    def test_invalid_mode_passthrough(self):
        # An out-of-range control_mode must publish the pilot command
        # unchanged instead of an uninitialised output array.
        self.set_params({"control_mode": 7})

        try:
            self._prime_sensors()
            cmd = [0.3, -0.2, 0.1, 0.05, -0.05, 0.1]
            out = self.send_and_wait(cmd)
            for i, axis in enumerate(["surge", "sway", "heave", "roll", "pitch", "yaw"]):
                self._assert_close(out.cmd_vel[i], cmd[i], 0.001, f"invalid mode passthrough {axis}")
        finally:
            self.set_params({"control_mode": 0})

    def test_setpoint_update_on_zero_command(self):
        self.set_pid_gains([0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0])
        self.set_control_mode(1)

        self.publish_imu(1.0, 0.0, 0.0, 0.0)
        self.publish_depth(0.0)
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


def setup_suite(tester: NereoControllerTester):
    """Suite-wide defaults so per-test parameter sets stay minimal.

    authority_cap defaults to 0.0 (fail toward zero authority, D-08); the
    pre-04-03 tests assume an unclamped +/-1 feedback range, so the suite
    opens it back up before any test runs.
    """
    tester.set_params({"authority_cap": 1.0})


def run_test_case(tester: NereoControllerTester, name: str, fn) -> TestResult:
    try:
        fn()
        return TestResult(name=name, passed=True, detail="ok")
    except Exception as exc:  # noqa: BLE001
        return TestResult(name=name, passed=False, detail=str(exc))


def main() -> int:
    rclpy.init()
    tester = NereoControllerTester()
    setup_suite(tester)

    tests = [
        ("passthrough", tester.test_passthrough),
        ("pid_roll_correction", tester.test_pid_roll_correction),
        ("depth_metres_heave", tester.test_depth_metres_heave),
        ("authority_cap", tester.test_authority_cap),
        ("mode_change_resets_integrator", tester.test_mode_change_resets_integrator),
        ("invalid_mode_passthrough", tester.test_invalid_mode_passthrough),
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