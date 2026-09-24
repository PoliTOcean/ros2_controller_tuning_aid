#!/usr/bin/env python3
"""04-05 Task 1: proves the committed params YAML loads through launch into
the running controller, and that a malformed gains array fails safe to zero
instead of the old non-zero 0.1/0.01/0.05 fallback (D-03/D-05, T-04-21)."""

import os
import signal
import subprocess
import sys
import tempfile
import time
from pathlib import Path
from typing import Dict, List, Sequence

import rclpy
from ament_index_python.packages import get_package_prefix
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.srv import GetParameters

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from test_nereo_controller import NereoControllerTester, TestResult, run_test_case  # noqa: E402


def _decode_value(value):
    t = value.type
    if t == ParameterType.PARAMETER_BOOL:
        return value.bool_value
    if t == ParameterType.PARAMETER_INTEGER:
        return value.integer_value
    if t == ParameterType.PARAMETER_DOUBLE:
        return value.double_value
    if t == ParameterType.PARAMETER_STRING:
        return value.string_value
    if t == ParameterType.PARAMETER_DOUBLE_ARRAY:
        return list(value.double_array_value)
    if t == ParameterType.PARAMETER_INTEGER_ARRAY:
        return list(value.integer_array_value)
    if t == ParameterType.PARAMETER_BOOL_ARRAY:
        return list(value.bool_array_value)
    return None


def get_params(node, names: Sequence[str]) -> Dict[str, object]:
    """Fetch parameters from /nereo_controller_node's GetParameters
    service, waiting up to 10 s for the service to become available."""
    client = node.create_client(GetParameters, "/nereo_controller_node/get_parameters")
    if not client.wait_for_service(timeout_sec=10.0):
        raise TimeoutError("Timeout waiting for /nereo_controller_node/get_parameters")

    req = GetParameters.Request()
    req.names = list(names)
    future = client.call_async(req)

    end_t = time.time() + 10.0
    while time.time() < end_t:
        rclpy.spin_once(node, timeout_sec=0.05)
        if future.done():
            result = future.result()
            if result is None:
                raise RuntimeError("get_parameters returned no result")
            return {name: _decode_value(v) for name, v in zip(names, result.values)}
    raise TimeoutError("Timeout waiting for get_parameters response")


def _start_launch() -> subprocess.Popen:
    return subprocess.Popen(
        ["ros2", "launch", "nereo_controller_node", "nereo_controller.launch.py"],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )


def _stop_launch(proc: subprocess.Popen, timeout: float = 10.0) -> None:
    if proc.poll() is not None:
        return
    proc.send_signal(signal.SIGINT)
    try:
        proc.wait(timeout=timeout)
    except subprocess.TimeoutExpired:
        proc.kill()
        proc.wait(timeout=5.0)


def _start_node_with_params(params_file: str) -> subprocess.Popen:
    exe = os.path.join(
        get_package_prefix("nereo_controller_node"),
        "lib", "nereo_controller_node", "nereo_controller_node",
    )
    return subprocess.Popen(
        [exe, "--ros-args", "--params-file", params_file],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )


def _stop_node(proc: subprocess.Popen, timeout: float = 10.0) -> None:
    if proc.poll() is not None:
        return
    proc.terminate()
    try:
        proc.wait(timeout=timeout)
    except subprocess.TimeoutExpired:
        proc.kill()
        proc.wait(timeout=5.0)


def _write_malformed_params(path: str) -> None:
    # kp is malformed (3 elements, not PID_NUMBER=4); every other
    # persisted array is valid so only the kp size trips the fail-safe.
    Path(path).write_text(
        "/nereo_controller_node:\n"
        "  ros__parameters:\n"
        "    kp: [1.0, 1.0, 1.0]\n"
        "    ki: [0.0, 0.0, 0.0, 0.0]\n"
        "    kd: [0.0, 0.0, 0.0, 0.0]\n"
        "    anti_windup_gains: [1.0, 1.0, 1.0, 1.0]\n"
        "    authority_cap: 1.0\n"
    )


def test_launch_loads_shipped_yaml(tester: NereoControllerTester) -> None:
    proc = _start_launch()
    try:
        values = get_params(
            tester, ["kp", "ki", "kd", "anti_windup_gains", "authority_cap", "control_mode"]
        )
        NereoControllerTester._assert_true(
            values["kp"] == [0.0, 0.0, 0.0, 0.0], f"kp: expected [0,0,0,0], got {values['kp']}"
        )
        NereoControllerTester._assert_true(
            values["ki"] == [0.0, 0.0, 0.0, 0.0], f"ki: expected [0,0,0,0], got {values['ki']}"
        )
        NereoControllerTester._assert_true(
            values["kd"] == [0.0, 0.0, 0.0, 0.0], f"kd: expected [0,0,0,0], got {values['kd']}"
        )
        NereoControllerTester._assert_true(
            values["anti_windup_gains"] == [1.0, 1.0, 1.0, 1.0],
            f"anti_windup_gains: expected [1,1,1,1], got {values['anti_windup_gains']}",
        )
        NereoControllerTester._assert_close(values["authority_cap"], 0.25, 0.001, "authority_cap")
        NereoControllerTester._assert_true(
            values["control_mode"] == 0, f"control_mode: expected 0, got {values['control_mode']}"
        )
    finally:
        _stop_launch(proc)


def test_malformed_gains_fail_safe_to_zero(tester: NereoControllerTester) -> None:
    tmp_dir = tempfile.mkdtemp()
    params_file = os.path.join(tmp_dir, "malformed_params.yaml")
    _write_malformed_params(params_file)

    proc = _start_node_with_params(params_file)
    try:
        tester.set_params({
            "control_mode": 1,
            "manual_setpoint_depth": True,
            "setpoint_depth": 0.0,
        })
        tester.publish_imu(1.0, 0.0, 0.0, 0.0)
        tester.publish_depth(1.0)
        tester.spin_for(0.2)
        out = tester.send_and_wait([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        NereoControllerTester._assert_close(
            out.cmd_vel[2], 0.0, 0.001, "heave with a malformed kp array"
        )
    finally:
        tester.set_params({"manual_setpoint_depth": False, "control_mode": 0})
        _stop_node(proc)


def main() -> int:
    rclpy.init()
    tester = NereoControllerTester()

    tests = [
        ("test_launch_loads_shipped_yaml", lambda: test_launch_loads_shipped_yaml(tester)),
        (
            "test_malformed_gains_fail_safe_to_zero",
            lambda: test_malformed_gains_fail_safe_to_zero(tester),
        ),
    ]

    results: List[TestResult] = []
    tester.get_logger().info("Starting params-file test suite for nereo_controller_node")

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
