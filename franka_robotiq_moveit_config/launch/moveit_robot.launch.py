import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

from moveit_configs_utils import MoveItConfigsBuilder
import yaml

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

    # ---------------------
    # 1) Launch Arguments
    # ---------------------
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_ip",
            default_value="192.168.0.4",
            description="Robot IP address or hostname for the real/sim Franka Panda.",
        )
    )
    # declare parameter for using gripper
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_gripper",
            default_value="false",
            description="Use gripper",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_fake_hardware",
            default_value="false",
            description="Use fake hardware interface for testing (true/false).",
        )
    )

    # ---------------------
    # 2) MoveIt Config
    #    (팔 + 그리퍼 통합 URDF / SRDF)
    # ---------------------
    
    # panda arm config
    moveit_config = (
        MoveItConfigsBuilder(robot_name="panda", package_name="franka_robotiq_moveit_config")
        .robot_description(file_path=get_package_share_directory("franka_robotiq_description") + "/urdf/robot.urdf.xacro",
            mappings={
                "robot_ip": LaunchConfiguration("robot_ip"),
                " robotiq_gripper": LaunchConfiguration("use_gripper"),
                " use_fake_hardware": LaunchConfiguration("use_fake_hardware"),
                })
        .robot_description_semantic("config/panda.srdf.xacro")
         # kinematics, joint_limits, etc.
        .robot_description_kinematics("config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .to_moveit_configs()
        )
        
    # (예) ompl_planning.yaml 로드
    ompl_planning_yaml = load_yaml("franka_robotiq_moveit_config", "config/ompl_planning.yaml")

    # ---------------------
    # 3) Panda ros2_control_node
    #    - panda_controllers.yaml
    #    - 통합 URDF도 가능하지만, 여기서는 "팔 부분"만 제어
    #    - 하지만 질문 코드처럼 통합 URDF를 그대로 써도 됨
    # ---------------------
    panda_controller_config = os.path.join(
        get_package_share_directory("franka_robotiq_moveit_config"),
        "config",
        "panda_controllers.yaml",
    )
    # 여기서는 질문 코드처럼 "moveit_config.robot_description" (통합 URDF) 사용
    panda_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        namespace="panda",
        output="both",
        parameters=[
            moveit_config.robot_description,  # 통합이든, 팔만이든 URDF를 여기 파라미터로
            panda_controller_config
        ],
    )

    # Controller spawner (panda)
    load_panda_controllers = []
    for ctrl_name in ["panda_jtc_controller", "panda_state_broadcaster"]:
        load_panda_controllers.append(
            ExecuteProcess(
                cmd=[f"ros2 run controller_manager spawner {ctrl_name} -c /panda/controller_manager"],
                shell=True,
                output="both",
            )
        )

    # (선택) panda_state_publisher - 질문 코드처럼 쓰려면:
    panda_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace="panda",
        output="both",
        parameters=[moveit_config.robot_description],
    )

    # ---------------------
    # 4) Robotiq ros2_control_node
    #    - robotiq_controllers.yaml
    # ---------------------
    robotiq_controllers_yaml = os.path.join(
        get_package_share_directory("franka_robotiq_moveit_config"),
        "config",
        "robotiq_controllers.yaml",
    )

    # 예: robotiq_2f_85_gripper.urdf.xacro (gripper만)
    # 하지만 질문 코드처럼 굳이 별도 xacro를 다시 읽어서 param에 넣어도 됩니다.
    # 아래는 질문의 형태를 유지한 예.
    robotiq_xacro_path = os.path.join(
        get_package_share_directory("robotiq_description"),  # 또는 franka_robotiq_description
        "urdf",
        "robotiq_2f_85_gripper.urdf.xacro",
    )
    robotiq_urdf_cmd = Command([
        FindExecutable(name="xacro"), " ",
        robotiq_xacro_path, " ",
        "use_fake_hardware:=", LaunchConfiguration("use_fake_hardware"),
    ])
    robotiq_description_param = {"robot_description": robotiq_urdf_cmd}

    robotiq_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        namespace="robotiq",
        output="screen",
        parameters=[
            robotiq_description_param,
            robotiq_controllers_yaml,
        ],
    )

    # Controller spawner (robotiq)
    load_robotiq_controllers = []
    for ctrl_name in ["robotiq_state_broadcaster", "robotiq_gripper_controller", "robotiq_activation_controller"]:
        load_robotiq_controllers.append(
            ExecuteProcess(
                cmd=[f"ros2 run controller_manager spawner {ctrl_name} -c /robotiq/controller_manager"],
                shell=True,
                output="both",
            )
        )

    # (선택) robotiq_state_publisher
    robotiq_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace="robotiq",
        name="robotiq_state_publisher",
        output="both",
        parameters=[robotiq_description_param],
    )

    # ---------------------
    # 5) MoveIt (move_group)
    # ---------------------
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="both",
        parameters=[
            moveit_config.robot_description,            # 통합 URDF
            moveit_config.robot_description_semantic,   # SRDF
            moveit_config.robot_description_kinematics, # kinematics.yaml
            moveit_config.joint_limits,
            ompl_planning_yaml,
            moveit_config.trajectory_execution,
        ],
        name="move_group",
    )

    # ---------------------
    # 6) robot_state_publisher (통합 URDF)
    #    -> 중복 가능 (질문 코드와 동일하게)
    # ---------------------
    # 질문 코드엔 "combined_robot_state_pub"가 있음
    # robot.urdf.xacro를 xacro로 변환해서 파라미터에 넣음
    franka_xacro_file = os.path.join(
        get_package_share_directory("franka_robotiq_description"),
        "urdf",
        "robot.urdf.xacro",
    )
    combined_robot_description_cmd = Command(
        [FindExecutable(name='xacro'), ' ', franka_xacro_file, ' hand:=', "False",
         ' robot_ip:=', "192.168.0.4", ' use_fake_hardware:=', "False",
         ' fake_sensor_commands:=', "False"])
    combined_robot_state_pub = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[{"robot_description": combined_robot_description_cmd}],
    )

    # ---------------------
    # 7) RViz (MotionPlanning)
    # ---------------------
    rviz_file = os.path.join(
        get_package_share_directory("franka_robotiq_description"),
        "rviz",
        "visualize.rviz",
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="both",
        arguments=["--display-config", rviz_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
        ],
    )

    # ---------------------
    # LaunchDescription
    # ---------------------
    return LaunchDescription(
        declared_arguments
        + [
            # (3) Panda ros2_control
            panda_control_node,
            panda_state_publisher,  # 선택
            # (4) Robotiq ros2_control
            robotiq_control_node,
            robotiq_state_publisher,  # 선택

            # (5) MoveIt
            move_group_node,

            # (6) Combined RSP + (7) RViz
            combined_robot_state_pub,
            rviz_node,
        ]
        + load_panda_controllers
        + load_robotiq_controllers
    )

