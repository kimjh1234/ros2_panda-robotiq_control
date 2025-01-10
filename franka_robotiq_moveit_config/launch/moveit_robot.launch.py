import os
from launch import LaunchDescription
from launch.actions import OpaqueFunction, IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.launch_description_sources import load_python_launch_file_as_module
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
import launch_ros
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node, SetParameter
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():

    # declare parameter for using robot ip
    robot_ip = DeclareLaunchArgument(
        "robot_ip",
        default_value="192.168.0.4",
        description="Robot IP",
    )

    # declare parameter for using gripper
    use_gripper = DeclareLaunchArgument(
        "use_gripper",
        default_value="false",
        description="Use gripper",
    )

    # declare parameter for using fake controller
    use_fake_hardware = DeclareLaunchArgument(
        "use_fake_hardware",
        default_value="false", # default to fake hardware (Important: that user is explicit with intention of launching real hardware!)
        description="Use fake hardware",
    )

    # panda arm config
    moveit_config = (
            MoveItConfigsBuilder(robot_name="panda", package_name="franka_robotiq_moveit_config")
            .robot_description(file_path=get_package_share_directory("franka_robotiq_description") + "/urdf/robot.urdf.xacro",
                mappings={
                    "robot_ip": LaunchConfiguration("robot_ip"),
                    "robotiq_gripper": LaunchConfiguration("use_gripper"),
                    "use_fake_hardware": LaunchConfiguration("use_fake_hardware"),
                    })
            .robot_description_semantic("config/panda.srdf.xacro")
            .trajectory_execution("config/moveit_controllers.yaml")
            .to_moveit_configs()
            )

    panda_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="panda_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description],
        namespace="panda",
    )

    panda_controller_config = os.path.join(
        get_package_share_directory("franka_robotiq_moveit_config"),
        "config",
        "panda_controllers.yaml",
    )

    panda_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[moveit_config.robot_description, panda_controller_config],
        output="both",
        namespace="panda"
    )


    # gripper config
    robotiq_xacro = os.path.join(
            get_package_share_directory("robotiq_description"),
            "urdf",
            "robotiq_2f_85_gripper.urdf.xacro",
            )

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            robotiq_xacro,
            " ",
            "use_fake_hardware:=false",
        ]
    )

    robotiq_description_param = {
        "robot_description": launch_ros.parameter_descriptions.ParameterValue(
            robot_description_content, value_type=str
        )
    }

    robotiq_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robotiq_state_publisher",
        output="both",
        parameters=[robotiq_description_param],
        namespace="robotiq",
    )

    robotiq_controller_config = os.path.join(
        get_package_share_directory("franka_robotiq_moveit_config"),
        "config",
        "robotiq_controllers.yaml",
    )

    robotiq_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robotiq_description_param, robotiq_controller_config],
        output="both",
        namespace="robotiq",
    )

    # publishing 중복 발생!!

    # # overall joint state
    # joint_state_publisher = Node(
    #     package='joint_state_publisher',
    #     executable='joint_state_publisher',
    #     name='joint_state_publisher',
    #     output='screen',
    #     parameters=[
    #             {'source_list': ['panda_state_broadcaster/joint_states', 'robotiq_state_broadcaster/joint_states'],
    #              'rate': 30}],
    # )


    load_panda_controllers = []
    for controller in [
        'panda_jtc_controller',
        'panda_state_broadcaster',
    ]:
        load_panda_controllers += [
            ExecuteProcess(
                cmd=["ros2 run controller_manager spawner {} -c /panda/controller_manager".format(controller)],
                shell=True,
                output="screen",
            )
        ]
    load_robotiq_controllers = []
    for controller in [
        'robotiq_state_broadcaster',
        'robotiq_gripper_controller',
        'robotiq_activation_controller',
    ]:
        load_robotiq_controllers += [
            ExecuteProcess(
                cmd=["ros2 run controller_manager spawner {} -c /robotiq/controller_manager".format(controller)],
                shell=True,
                output="screen",
            )
        ]

    # rviz
    franka_xacro_file = os.path.join(get_package_share_directory('franka_robotiq_description'), 'urdf',
                                     'robot.urdf.xacro')

    robot_description_config = Command(
        [FindExecutable(name='xacro'), ' ', franka_xacro_file, ' hand:=', "False",
         ' robot_ip:=', "192.168.0.4", ' use_fake_hardware:=', "False",
         ' fake_sensor_commands:=', "False"])

    robot_description = {'robot_description': robot_description_config}

    rviz_file = os.path.join(get_package_share_directory('franka_robotiq_description'), 'rviz',
                             'visualize.rviz')

    robot_state_publisher = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[robot_description],
        )

    rviz_node = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['--display-config', rviz_file],
        )

    return LaunchDescription(
        [
            robot_ip,
            use_gripper,
            use_fake_hardware,
            panda_state_publisher,
            panda_control_node,
            robotiq_state_publisher,
            robotiq_control_node,
            robot_state_publisher,
            rviz_node

        ]
        + load_panda_controllers
        + load_robotiq_controllers
        )


