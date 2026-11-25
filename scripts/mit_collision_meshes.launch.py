from copy import deepcopy
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from b601_demos.web_robot_model import MESH_URI, VENDOR_URDF, find_vendor_model
from launch import LaunchDescription
from launch_ros.actions import Node

sys.path.insert(0, str(Path(get_package_share_directory("b601_moveit_config")) / "launch"))
from moveit_common import build_moveit_configs


def generate_launch_description():
    vendor_model = find_vendor_model()
    if vendor_model is None:
        raise RuntimeError("找不到 B601-DM 模型。请构建当前工作区，或设置 B601_VENDOR_DESCRIPTION。")
    vendor_model = vendor_model.resolve()
    config = build_moveit_configs(hardware="dm").to_dict()
    robot = ET.fromstring(config["robot_description"])
    vendor = ET.parse(vendor_model / VENDOR_URDF)
    for link in robot.findall("link"):
        vendor_link = vendor.find(f"link[@name='{link.get('name')}']")
        if vendor_link is None:
            continue
        for collision in link.findall("collision"):
            link.remove(collision)
        for original in vendor_link.findall("collision"):
            collision = deepcopy(original)
            mesh = collision.find("./geometry/mesh")
            path = vendor_model / mesh.get("filename").removeprefix(MESH_URI)
            if not path.is_file():
                raise FileNotFoundError(path)
            mesh.set("filename", path.as_uri())
            link.append(collision)
    config["robot_description"] = ET.tostring(robot, encoding="unicode")
    return LaunchDescription([
        Node(package="moveit_ros_move_group", executable="move_group", output="screen",
             parameters=[config, {"allow_trajectory_execution": False}]),
    ])
