import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
import launch_ros

def generate_launch_description():

    # robot_ip_parameter_name = 'robot_ip'
    # use_fake_hardware_parameter_name = 'use_fake_hardware'
    # load_gripper_parameter_name = 'load_gripper'
    # fake_sensor_commands_parameter_name = 'fake_sensor_commands'

    # robot_ip = LaunchConfiguration(robot_ip_parameter_name)
    # use_fake_hardware = LaunchConfiguration(use_fake_hardware_parameter_name)
    # load_gripper = LaunchConfiguration(load_gripper_parameter_name)
    # fake_sensor_commands = LaunchConfiguration(fake_sensor_commands_parameter_name)

    # planning_context
    franka_xacro_file = os.path.join(get_package_share_directory('franka_robotiq_description'), 'urdf',
                                     'robot.urdf.xacro')

    robot_description_config = Command(
        [FindExecutable(name='xacro'), ' ', franka_xacro_file, ' hand:=', "False",
         ' robot_ip:=', "192.168.0.4", ' use_fake_hardware:=', "False",
         ' fake_sensor_commands:=', "False"])

    robot_description = {'robot_description': robot_description_config}

    rviz_file = os.path.join(get_package_share_directory('franka_robotiq_description'), 'rviz',
                             'visualize.rviz')

    return LaunchDescription([
        # Node(
        #     package='joint_state_publisher_gui',
        #     executable='joint_state_publisher_gui',
        #     name='joint_state_publisher_gui'
        # ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            #parameters=[{'robot_description': launch_ros.parameter_descriptions.ParameterValue(robot_description, value_type=str)}]
            #parameters=[{'robot_description': robot_description}]
            parameters=[
                robot_description
            ],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['--display-config', rviz_file],
            # parameters=[
            #     robot_description
            # ],
        ),
    ])
