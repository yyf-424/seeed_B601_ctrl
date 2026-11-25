#!/usr/bin/env python3
"""Deactivate the parked B601 through controller-manager services before exit."""

import argparse
import os
import signal
import sys

import rclpy
from controller_manager_msgs.srv import ListControllers, SetHardwareComponentState, SwitchController
from lifecycle_msgs.msg import State
from rclpy.duration import Duration

from project_config import load_project_config


def deactivate_parked(node, config):
    safety = config["safety"]["estop"]
    timeout = safety["service_timeout_sec"]

    def call(service, name, request):
        path = f"/controller_manager/{name}"
        client = node.create_client(service, path)
        try:
            if not client.wait_for_service(timeout_sec=timeout):
                raise RuntimeError(f"服务不可用 {path}")
            future = client.call_async(request)
            rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
            if not future.done():
                future.cancel()
                raise RuntimeError(f"服务应答超时 {path}")
            response = future.result()
            if response is None:
                raise RuntimeError(f"服务未返回应答 {path}")
            return response
        finally:
            node.destroy_client(client)

    def active_controllers():
        response = call(ListControllers, "list_controllers", ListControllers.Request())
        return [controller.name for controller in response.controller if controller.state == "active"]

    active = active_controllers()
    if active:
        request = SwitchController.Request()
        request.deactivate_controllers = active
        request.strictness = SwitchController.Request.STRICT
        request.timeout = Duration(seconds=timeout).to_msg()
        if not call(SwitchController, "switch_controller", request).ok:
            raise RuntimeError(f"控制器停用失败：{', '.join(active)}")
        if remaining := active_controllers():
            raise RuntimeError(f"控制器仍在运行：{', '.join(remaining)}")
        print(f"已停用控制器：{', '.join(active)}", flush=True)

    request = SetHardwareComponentState.Request()
    request.name = safety["hardware_component"]
    request.target_state = State(id=State.PRIMARY_STATE_INACTIVE, label="inactive")
    response = call(SetHardwareComponentState, "set_hardware_component_state", request)
    if not response.ok or response.state.id != State.PRIMARY_STATE_INACTIVE:
        raise RuntimeError(f"硬件 {request.name} 未进入 inactive，状态为 {response.state.label}")
    print(f"硬件 {request.name} 已确认进入 inactive", flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("controller_manager_pid", type=int)
    args = parser.parse_args()
    try:
        process = os.pidfd_open(args.controller_manager_pid)
        try:
            rclpy.init(args=[])
            node = rclpy.create_node("b601_park_stop")
            try:
                deactivate_parked(node, load_project_config())
                # Humble's signal shutdown can run concurrently with a controller update.
                signal.pidfd_send_signal(process, signal.SIGTERM)
            finally:
                node.destroy_node()
                rclpy.try_shutdown()
        finally:
            os.close(process)
    except (OSError, RuntimeError) as exc:
        print(f"停机未完成：{exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
