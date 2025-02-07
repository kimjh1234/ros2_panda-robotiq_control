import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.task import Future
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from moveit_msgs.srv import GetPositionFK, GetPositionIK
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from std_msgs.msg import Header
from control_msgs.action import GripperCommand

import numpy as np
import math
import time
from scipy.spatial.transform import Rotation as R
import triad_openvr


class TeleopArm(Node):

    def __init__(self):
        super().__init__('teleop_arm_node')
        self.get_logger().info("__init__")
        self.joint_state = None

        # parameter setting
        self.declare_parameter("control_frequency", 200.0)
        self.control_frequency = self.get_parameter("control_frequency").value
        self.control_period = 1.0 / self.control_frequency  # 초 단위
        self.declare_parameter("move_velocity_factor", 0.2)
        move_velocity_factor = self.get_parameter("move_velocity_factor").value
        self.declare_parameter("gripper_interval", 1.5) # 초 단위
        self.gripper_interval = self.get_parameter("gripper_interval").value

        # velocity factor setting (translation, rotation)
        self.tra_scale_factor = self.control_frequency * move_velocity_factor
        self.rot_scale_factor = self.control_frequency * move_velocity_factor * 1.5 

        # input device: triad_openvr를 통해 VR 컨트롤러 인터페이스 초기화
        self.teleop_interface = triad_openvr.triad_openvr()

        # panda setting
        # panda joint subscriber from a panda arm
        self.create_subscription(JointState, '/joint_states', self.joint_state_cb, 10)
        # panda joint publisher to move a panda arm
        self.trajectory_pub = self.create_publisher(
            JointTrajectory, '/panda/joint_trajectory_controller/joint_trajectory', 10)
        # panda forward/inverse kinematics setting
        self.fk_client = self.create_client(GetPositionFK, '/compute_fk')
        self.ik_client = self.create_client(GetPositionIK, '/compute_ik')
        while not self.fk_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for FK service...')
        while not self.ik_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for IK service...')

        # robotiq setting
        self.robotiq_client = ActionClient(self, GripperCommand, '/robotiq/robotiq_gripper_controller/gripper_cmd')
        self.start_grip_time_stamp = time.time()
        self.grip_on = True

        self.get_logger().info("__init__end")

    def joint_state_cb(self, msg):
        self.joint_state = msg.position

    def call_fk_service(self, joint_positions):
        request = GetPositionFK.Request()
        request.header.frame_id = ''
        request.fk_link_names = ['panda_link8']
        request.robot_state.joint_state.name = [
            'panda_joint1', 'panda_joint2', 'panda_joint3',
            'panda_joint4', 'panda_joint5', 'panda_joint6', 'panda_joint7'
        ]
        request.robot_state.joint_state.position = joint_positions
        future = self.fk_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            return future.result()
        else:
            self.get_logger().error('Failed to call FK service')
            return None

    def call_ik_service(self, pose):
        request = GetPositionIK.Request()
        request.ik_request.group_name = 'panda_arm'
        request.ik_request.pose_stamped.header.frame_id = ''
        request.ik_request.pose_stamped.pose.position = pose.pose.position
        request.ik_request.pose_stamped.pose.orientation = pose.pose.orientation
        request.ik_request.robot_state.joint_state.name = [
            'panda_joint1', 'panda_joint2', 'panda_joint3',
            'panda_joint4', 'panda_joint5', 'panda_joint6', 'panda_joint7'
        ]
        request.ik_request.robot_state.joint_state.position = self.joint_state if self.joint_state else [0] * 7
        future = self.ik_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            return future.result()
        else:
            self.get_logger().error('Failed to call IK service')
            return None

    def publish_trajectory(self, joint_positions):
        traj_msg = JointTrajectory()
        traj_msg.joint_names = [
            'panda_joint1', 'panda_joint2', 'panda_joint3',
            'panda_joint4', 'panda_joint5', 'panda_joint6', 'panda_joint7'
        ]
        point = JointTrajectoryPoint()
        point.positions = joint_positions
        point.time_from_start.sec = 1
        traj_msg.points.append(point)
        self.trajectory_pub.publish(traj_msg)

    def gripper_send_goal(self, position: float, max_effort: float):
        goal_msg = GripperCommand.Goal()
        goal_msg.command.position = position
        goal_msg.command.max_effort = max_effort
        
        self.get_logger().info(f'Sending goal: position={position}, max_effort={max_effort}')
        self.robotiq_client.wait_for_server()
        future = self.robotiq_client.send_goal_async(goal_msg, feedback_callback=self.gripper_feedback_callback)
        future.add_done_callback(self.gripper_goal_response_callback)

    def gripper_goal_response_callback(self, future: Future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return
        
        self.get_logger().info('Goal accepted, waiting for result...')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.gripper_result_callback)
    
    def gripper_feedback_callback(self, feedback_msg):
        self.get_logger().info(f'Feedback: {feedback_msg.feedback}')

    def gripper_result_callback(self, future: Future):
        result = future.result().result
        self.get_logger().info(f'Result: {result}')
  
    def run_control_cycle(self):
        cycle_start = time.time()

        if self.joint_state is None:
            return

        vr_inputs = self.teleop_interface.devices["controller_1"].get_controller_inputs()

        if vr_inputs['grip_button']:
            if self.gripper_interval - (time.time() - self.start_grip_time_stamp) > 0:
                pass
            else:
                self.start_grip_time_stamp = time.time()
                if self.grip_on == True:
                    # gripper closing
                    self.gripper_send_goal(position=1.0, max_effort=5.0)
                    rclpy.spin_once(self)
                    self.grip_on = False
                else:
                    # gripper opening
                    self.gripper_send_goal(position=0.0, max_effort=5.0)
                    rclpy.spin_once(self)
                    self.grip_on = True

        if vr_inputs['trigger']:
            # trigger가 눌린 경우 동기식으로 FK와 IK 호출
            fk_result = self.call_fk_service(self.joint_state)
            if fk_result is None:
                return
            current_pose = fk_result.pose_stamped[0].pose
            current_sensor_pose = self.get_vr_pose()
            new_pose = self.compute_new_pose(current_pose, current_sensor_pose)
            ik_result = self.call_ik_service(new_pose)
            if ik_result is None:
                return
            target_joints = ik_result.solution.joint_state.position[:7]
            self.publish_trajectory(target_joints)

        # 사이클 주기를 맞추기 위한 대기
        cycle_elapsed = time.time() - cycle_start
        remaining = self.control_period - cycle_elapsed
        if remaining > 0:
            time.sleep(remaining)

    def get_vr_pose(self):
        """
        triad_openvr의 get_pose_matrix()를 이용하여 현재 VR 컨트롤러의 포즈를 읽어옵니다.
        반환 형식은 {'position': np.array([x,y,z]), 'orientation': np.array([qx,qy,qz,qw])}입니다.
        """
        # OpenVR을 이용하여 현재 포즈 행렬을 가져옴 (3x4 행렬)
        matrix = self.teleop_interface.devices["controller_1"].get_pose_matrix()
        if matrix is None:
            return None

        # matrix는 3x4 배열: 회전은 좌측 3x3, 평행이동은 마지막 열
        translation = np.array([matrix[0][3], matrix[1][3], matrix[2][3]])
        rot_matrix = np.array([
            [matrix[0][0], matrix[0][1], matrix[0][2]],
            [matrix[1][0], matrix[1][1], matrix[1][2]],
            [matrix[2][0], matrix[2][1], matrix[2][2]]
        ])
        # scipy의 Rotation.from_matrix()는 3x3 행렬을 받아 [x, y, z, w] 순서의 쿼터니언을 반환합니다.
        quat = R.from_matrix(rot_matrix).as_quat()  # [qx, qy, qz, qw]
        return {'position': translation, 'orientation': quat}

    def compute_new_pose(self, current_robot_pose, current_sensor_pose):

        # No sensor pose
        if current_sensor_pose is None:
            self.get_logger().warn("현재 VR 포즈를 읽어올 수 없습니다. 기존 포즈를 사용합니다.")
            new_pose = PoseStamped()
            new_pose.header.frame_id = ''
            new_pose.pose = current_robot_pose
            return new_pose

        # No previous sensor pose
        if not hasattr(self, 'prev_sensor_pose') or self.prev_sensor_pose is None:
            self.prev_sensor_pose = current_sensor_pose
            new_pose = PoseStamped()
            new_pose.header.frame_id = ''
            new_pose.pose = current_robot_pose
            return new_pose

        self.get_logger().info(f"current_sensor_pose: {current_sensor_pose}")

        # translation velocity calculation: delta_t * freq ==> m/s
        prev_position = np.array(self.prev_sensor_pose['position'])
        curr_position = np.array(current_sensor_pose['position'])
        delta_translation = (curr_position - prev_position) 

        # orientation velocity calculation: delta_r * freq ==> m/s
        prev_rot = R.from_quat(self.prev_sensor_pose['orientation'])
        curr_rot = R.from_quat(current_sensor_pose['orientation'])
        delta_rot = curr_rot * prev_rot.inv()

        # calibrate the sensor pose to the robot coordinate system
        r_roll = R.from_euler('x', 90, degrees=True)
        transform_rot = r_roll 
        # apply scale
        converted_delta_translation = transform_rot.apply(delta_translation) * self.tra_scale_factor

        # 로봇의 새 위치: 현재 위치에 변환된 증분 translation을 더함
        current_position = np.array([
            current_robot_pose.position.x,
            current_robot_pose.position.y,
            current_robot_pose.position.z
        ])
        new_robot_position = current_position + converted_delta_translation

        # [대안 방식] 증분 회전을 보정 회전으로 변환 (유사변환: 양쪽에 곱셈)
        # 즉, delta_rot_converted = T * delta_rot * T^{-1}
        converted_delta_rot = transform_rot.inv() * delta_rot * transform_rot
        # 회전 벡터에 스케일 적용
        rotvec = converted_delta_rot.as_rotvec()
        scaled_rotvec = self.rot_scale_factor * rotvec
        converted_delta_rot = R.from_rotvec(scaled_rotvec)

        # 현재 로봇 말단 회전 읽기 (쿼터니언 [x, y, z, w])
        current_robot_rot = R.from_quat([
            current_robot_pose.orientation.x,
            current_robot_pose.orientation.y,
            current_robot_pose.orientation.z,
            current_robot_pose.orientation.w
        ])

        # [변경] 증분 회전을 현재 회전에 오른쪽에서 곱하여 업데이트
        new_robot_rot = current_robot_rot * converted_delta_rot
        new_robot_quat = new_robot_rot.as_quat()  # [x, y, z, w]

        # self.get_logger().info(f"converted_delta_translation: {converted_delta_translation}")

        # 새로운 PoseStamped 메시지 생성
        new_pose = PoseStamped()
        new_pose.header.stamp = self.get_clock().now().to_msg()
        new_pose.header.frame_id = ''
        new_pose.pose.position.x = float(new_robot_position[0])
        new_pose.pose.position.y = float(new_robot_position[1])
        new_pose.pose.position.z = float(new_robot_position[2])
        new_pose.pose.orientation.x = float(new_robot_quat[0])
        new_pose.pose.orientation.y = float(new_robot_quat[1])
        new_pose.pose.orientation.z = float(new_robot_quat[2])
        new_pose.pose.orientation.w = float(new_robot_quat[3])

        # 다음 사이클을 위해 센서의 이전 포즈 업데이트
        self.prev_sensor_pose = current_sensor_pose

        return new_pose


def main(args=None):
    rclpy.init(args=args)
    node = TeleopArm()
    # 메인 루프를 이용하여 제어 주기를 맞춤 (여기서는 50Hz로 설정됨)
    try:
        while rclpy.ok():
            # spin_once를 호출하여 구독 콜백 등을 처리
            rclpy.spin_once(node, timeout_sec=0.001)
            node.run_control_cycle()
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
