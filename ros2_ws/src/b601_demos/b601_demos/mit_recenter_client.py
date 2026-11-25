"""Call the resident MIT parking node and use its success flag as the exit status."""

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

from b601_description.project_config import load_project_config
from b601_demos.mit_recenter import RECENTER_SERVICE


def recenter_result_timeout(project):
    return (3 * project["recenter"]["mit"]["phase_timeout_sec"]
            + project["tools"]["dm_park"]["result_grace_sec"])


def request_recenter(node, project):
    discovery = project["validation"]["mit_five_poses"]["joint_state_timeout_sec"]
    timeout = recenter_result_timeout(project)
    client = node.create_client(Trigger, RECENTER_SERVICE)
    try:
        if not client.wait_for_service(timeout_sec=discovery):
            raise RuntimeError(f"没有归中服务 {RECENTER_SERVICE}")
        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
        if not future.done() or future.result() is None:
            raise RuntimeError("未收到归中结果")
        result = future.result()
        if not result.success:
            raise RuntimeError(result.message)
        return result.message
    finally:
        node.destroy_client(client)


def main(args=None):
    rclpy.init(args=args)
    node = Node("b601_mit_recenter_client")
    try:
        print(request_recenter(node, load_project_config()))
        return 0
    except RuntimeError as error:
        node.get_logger().error(str(error))
        return 1
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
