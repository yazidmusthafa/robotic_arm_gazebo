from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("robotic_arm", package_name="robotic_arm_v3").to_moveit_configs()

    # MoveGroupInterface demo executable
    move_group_demo = Node(
        name="cartesian_planning",
        package="robotic_arm_v3",
        executable="cartesian_planning",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
        ],
    )

    return LaunchDescription([move_group_demo])