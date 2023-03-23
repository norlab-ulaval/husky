import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.descriptions import ParameterValue


def generate_launch_description():
    # Get URDF via xacro
    package = get_package_share_directory("husky_description")
    xacro_path = os.path.join(package, "urdf/husky.urdf.xacro")

    robot_description_command = Command(["xacro ", xacro_path, " name:=husky"])
    robot_description = {
        "robot_description": ParameterValue(robot_description_command, value_type=str)
    }

    node_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    return LaunchDescription(
        [
            node_robot_state_publisher,
        ]
    )
