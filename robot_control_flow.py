import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from moveit_msgs.srv import GetPositionFK, GetPositionIK
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Duration
import time


class RobotControlNode(Node):
    def __init__(self):
        super().__init__('robot_control_node')

        # Publisher to send joint trajectory
        self.joint_trajectory_pub = self.create_publisher(JointTrajectory, '/panda/joint_trajectory_controller/joint_trajectory', 10)

        # Client for the FK and IK services
        self.fk_client = self.create_client(GetPositionFK, '/compute_fk')
        self.ik_client = self.create_client(GetPositionIK, '/compute_ik')

        # Wait for services to be available
        while not self.fk_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('FK service not available, waiting again...')
        while not self.ik_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('IK service not available, waiting again...')

        # Subscriber to listen to the current joint states
        self.joint_state = None
        self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)

    def joint_state_callback(self, msg):
        """ 콜백 함수로 /joint_states 토픽의 데이터를 받아옴 """
        self.joint_state = msg.position  # 조인트 포지션을 저장

    def call_fk_service(self, joint_positions):
        request = GetPositionFK.Request()
        request.header.frame_id = ''
        request.fk_link_names = ['panda_link8']
        request.robot_state.joint_state.name = ['panda_joint1', 'panda_joint2', 'panda_joint3', 'panda_joint4', 'panda_joint5', 'panda_joint6', 'panda_joint7']
        request.robot_state.joint_state.position = joint_positions

        future = self.fk_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info(f"FK response - position: {future.result().pose_stamped[0].pose.position}")
            self.get_logger().info(f"FK response - orientation: {future.result().pose_stamped[0].pose.orientation}")
            return future.result()
        else:
            self.get_logger().error('Failed to call FK service')
            return None

    def call_ik_service(self, pose):
        request = GetPositionIK.Request()
        request.ik_request.group_name = 'panda_arm'
        request.ik_request.pose_stamped.header.frame_id = ''
        request.ik_request.pose_stamped.pose.position = pose.position
        request.ik_request.pose_stamped.pose.orientation = pose.orientation
        request.ik_request.robot_state.joint_state.name = ['panda_joint1', 'panda_joint2', 'panda_joint3', 'panda_joint4', 'panda_joint5', 'panda_joint6', 'panda_joint7']
        request.ik_request.robot_state.joint_state.position = self.joint_state if self.joint_state else [0] * 7

        future = self.ik_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info(f"IK response: {future.result()}")
            return future.result()
        else:
            self.get_logger().error('Failed to call IK service')
            return None

    def send_joint_trajectory(self, joint_positions):
        msg = JointTrajectory()
        msg.joint_names = ['panda_joint1', 'panda_joint2', 'panda_joint3', 'panda_joint4', 'panda_joint5', 'panda_joint6', 'panda_joint7']

        point = JointTrajectoryPoint()
        point.positions = joint_positions
        point.time_from_start = Duration(sec=5)
        msg.points.append(point)

        self.joint_trajectory_pub.publish(msg)
        self.get_logger().info(f"Published joint trajectory: {joint_positions}")


def main(args=None):
    rclpy.init(args=args)
    node = RobotControlNode()

    # 기다려서 joint_state 값을 받을 때까지 대기
    while node.joint_state is None:
        node.get_logger().info("Waiting for joint_state data...")
        rclpy.spin_once(node)
        time.sleep(1)

    # Step 1: Get current joint positions from /joint_states
    joint_positions = node.joint_state  # /joint_states 토픽에서 받은 조인트 위치
    print(joint_positions)

    # Step 2: Call FK service
    fk_result = node.call_fk_service(joint_positions)

    # Step 3: Call IK service with new pose (example pose)
    pose = PoseStamped()
    pose.header = Header(frame_id='')
    pose.pose.position = Point(x=0.31397568480792054, y=0.013867171315823854, z=0.6918499482636833)
    pose.pose.orientation = Quaternion(x=0.8606628422610331, y=0.5089007669300405, z=-0.016477160503883842, w=0.002825694765192247)
    ik_result = node.call_ik_service(pose)

    # # Step 4: Publish the joint trajectory based on IK result
    # if ik_result is not None:
    #     joint_positions = ik_result.solution.joint_state.position[:7]  # Use only the joint positions for the arm
    #     node.send_joint_trajectory(joint_positions)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
