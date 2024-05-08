from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("robotic_arm", package_name="panda_moveit_config").to_dict()

    pick_place_demo = Node(
        package="panda_moveit_config",
        # executable="moveit_interface",
        executable="robotic_arm_moveit",
        output="screen",
        parameters=[
            moveit_config,
        ],
    )

    return LaunchDescription([pick_place_demo])