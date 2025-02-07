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
import yaml
from launch.actions import TimerAction

def load_yaml(package_name, file_path):
    """
    패키지 내부 YAML 파일을 dict로 로드하는 헬퍼 함수
    """
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except:
        return None

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
        default_value="true",
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
            .robot_description(file_path=get_package_share_directory("franka_robotiq_description") 
                               + "/urdf/robot.urdf.xacro",
                mappings={
                    "robot_ip": LaunchConfiguration("robot_ip"),
                    "robotiq_gripper": LaunchConfiguration("use_gripper"),
                    "use_fake_hardware": LaunchConfiguration("use_fake_hardware"),
                     })
            .robot_description_semantic("config/panda.srdf.xacro")
            .robot_description_kinematics("config/kinematics.yaml")
            .joint_limits(file_path="config/joint_limits.yaml")
            .to_moveit_configs()
            )

    # panda_control_node   
    panda_controller_config = os.path.join(
        get_package_share_directory("franka_robotiq_moveit_config"),
        "config",
        "panda_controllers.yaml",
    )

    panda_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[moveit_config.robot_description, panda_controller_config],
        output="screen",
        namespace="panda"
    )

    load_panda_controllers = []
    for controller in [
        'joint_trajectory_controller',
        'joint_state_broadcaster',
    ]:
        load_panda_controllers += [
            ExecuteProcess(
                cmd=["ros2 run controller_manager spawner {} -c /panda/controller_manager".format(controller)],
                shell=True,
                output="screen",
            )
        ]    

    # robotiq_control_node
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

    robotiq_controller_config = os.path.join(
        get_package_share_directory("franka_robotiq_moveit_config"),
        "config",
        "robotiq_controllers.yaml",
    )

    robotiq_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robotiq_description_param, robotiq_controller_config],
        output="screen",
        namespace="robotiq",
    )

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

    # robot_state_publisher
    robot_state_publisher = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[moveit_config.robot_description],
    )


    # move group node 
    ompl_planning_pipeline_config = {
        'move_group': {
            'planning_plugin': 'ompl_interface/OMPLPlanner',
            'request_adapters': 'default_planner_request_adapters/AddTimeOptimalParameterization '
                                'default_planner_request_adapters/ResolveConstraintFrames '
                                'default_planner_request_adapters/FixWorkspaceBounds '
                                'default_planner_request_adapters/FixStartStateBounds '
                                'default_planner_request_adapters/FixStartStateCollision '
                                'default_planner_request_adapters/FixStartStatePathConstraints',
            'start_state_max_bounds_error': 0.1,
        }
    }    
    ompl_planning_yaml = load_yaml("franka_robotiq_moveit_config", "config/ompl_planning.yaml")
    ompl_planning_pipeline_config['move_group'].update(ompl_planning_yaml)

    # Trajectory Execution Functionality
    moveit_controllers_yaml = load_yaml(
        'franka_robotiq_moveit_config', 'config/moveit_controllers.yaml'
    )
    trajectory_execution = {
        'moveit_manage_controllers': True,
        'trajectory_execution.allowed_execution_duration_scaling': 1.2,
        'trajectory_execution.allowed_goal_duration_margin': 0.5,
        'trajectory_execution.allowed_start_tolerance': 0.01,
    }

    planning_scene_monitor_parameters = {
        'publish_planning_scene': True,
        'publish_geometry_updates': True,
        'publish_state_updates': True,
        'publish_transforms_updates': True,
        'joint_state_topic': ['/panda/joint_states', '/robotiq/joint_states'],
    }

    move_group_node = Node(
                package="moveit_ros_move_group",
                executable="move_group",
                output="screen",
                parameters=[
                    moveit_config.robot_description,            # 통합 URDF
                    moveit_config.robot_description_semantic,   # SRDF
                    moveit_config.robot_description_kinematics, # kinematics.yaml
                    moveit_config.joint_limits,
                    ompl_planning_pipeline_config,
                    trajectory_execution,
                    moveit_controllers_yaml,
                    planning_scene_monitor_parameters,
                ],
                name="move_group",
            )

    # joint_state_publisher
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[
                {'source_list': ['/panda/joint_states', '/robotiq/joint_states'],
                 'rate': 30}],
    )

    # rviz_node
    rviz_file = os.path.join(get_package_share_directory('franka_robotiq_description'), 'rviz',
                             'visualize_w_mp.rviz')


    rviz_node = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['--display-config', rviz_file],
            parameters=[
                moveit_config.robot_description,
                moveit_config.robot_description_semantic,
                ompl_planning_pipeline_config,
                moveit_config.robot_description_kinematics,
                ],
        )
   
    return LaunchDescription(
        [
            robot_ip,
            use_gripper,
            use_fake_hardware,
            rviz_node,
            robot_state_publisher,
            move_group_node,
            panda_control_node,
            robotiq_control_node,
            joint_state_publisher,
        ]
        + load_panda_controllers
        + load_robotiq_controllers
        )


