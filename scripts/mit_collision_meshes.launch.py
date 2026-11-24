from copy import deepcopy
import os
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

sys.path.insert(0, str(Path(get_package_share_directory("b601_moveit_config")) / "launch"))
from moveit_common import build_moveit_configs

DEFAULT_REFERENCE = Path(__file__).resolve().parents[2] / "rebotarm_ros2/src/rebotarm_bringup"


def generate_launch_description():
    reference = Path(os.environ.get("B601_VENDOR_DESCRIPTION", DEFAULT_REFERENCE)).resolve()
    config = build_moveit_configs(hardware="dm").to_dict()
    robot = ET.fromstring(config["robot_description"])
    vendor = ET.parse(reference / "description/DM/urdf/ReBot_Arm_DM.urdf")
    for link in robot.findall("link"):
        vendor_link = vendor.find(f"link[@name='{link.get('name')}']")
        if vendor_link is None:
            continue
        for collision in link.findall("collision"):
            link.remove(collision)
        for original in vendor_link.findall("collision"):
            collision = deepcopy(original)
            mesh = collision.find("./geometry/mesh")
            path = reference / mesh.get("filename").removeprefix("package://rebotarm_bringup/")
            if not path.is_file():
                raise FileNotFoundError(path)
            mesh.set("filename", path.as_uri())
            link.append(collision)
    config["robot_description"] = ET.tostring(robot, encoding="unicode")
    return LaunchDescription([
        Node(package="moveit_ros_move_group", executable="move_group", output="screen",
             parameters=[config, {"allow_trajectory_execution": False}]),
    ])
