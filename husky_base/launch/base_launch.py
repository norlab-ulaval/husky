from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.descriptions import ParameterValue
import xacro


def generate_launch_description():
    # Get URDF via xacro
    # Get URDF via xacro
    package = get_package_share_directory("husky_description")
    xacro_path = os.path.join(package, "urdf/husky.urdf.xacro")

    robot_description_command = Command(["xacro ", xacro_path, " name:=husky"])
    robot_description = {
        "robot_description": ParameterValue(robot_description_command, value_type=str)
    }

    config_husky_velocity_controller = PathJoinSubstitution(
        [FindPackageShare("husky_control"),
        "config",
        "control.yaml"],
    )

    node_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    node_controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, config_husky_velocity_controller],
        output={
            "stdout": "screen",
            "stderr": "screen",
        },
        remappings=[('odom', 'husky_velocity_controller/odom')],
    )

    spawn_controller = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["joint_state_broadcaster"],
        output="screen",
    )

    spawn_husky_velocity_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["husky_velocity_controller"],
        output="screen",
    )

    ld = LaunchDescription()
    ld.add_action(node_robot_state_publisher)
    ld.add_action(node_controller_manager)
    ld.add_action(spawn_controller)
    ld.add_action(spawn_husky_velocity_controller)

    return ld
