# Reference) ~/ws_moveit2/src/moveit2_tutorials/doc/examples/move_group_interface/launch/move_group_interface_tutorial.launch.py

from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("moveit_resources_panda").to_moveit_configs()

    # Panda moveit practice demo executable
    panda_moveit_practice_demo = Node(
        name="panda_moveit_practice",
        package="panda_moveit_practice",
        executable="panda_moveit_practice_node",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
        ],
    )

    return LaunchDescription([panda_moveit_practice_demo])
