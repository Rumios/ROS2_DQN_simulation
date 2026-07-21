# dqn_autonomous/node/driver_node.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped, Pose
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
import tf2_ros
import math
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

class DriverNode(Node):
    def __init__(self):
        super().__init__('vehicle_driver_node')
        
        self.cmd_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        
        map_qos_profile = QoSProfile(depth=1)
        map_qos_profile.reliability = ReliabilityPolicy.RELIABLE
        map_qos_profile.durability = DurabilityPolicy.TRANSIENT_LOCAL
        
        # Point에서 Pose 메시지 타입으로 변경하여 위치 및 웨이포인트 기반 조향각 수신
        self.init_pose_sub = self.create_subscription(Pose, '/initial_pose', self.initial_pose_callback, map_qos_profile)
        
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        
        scale_factor = 0.5625
        self.wheel_radius = 0.06 * scale_factor
        self.wheel_separation = 0.36 * scale_factor
        
        self.fl_angle = self.fr_angle = self.rl_angle = self.rr_angle = 0.0
        self.linear_x = self.angular_z = 0.0
        self.x = self.y = self.theta = 0.0
        
        self.dt = 0.02
        self.timer = self.create_timer(self.dt, self.update_states)
        self.get_logger().info("드라이버 기동 완료, 초기화 수신 대기 중")
        
    def cmd_vel_callback(self, msg):
        self.linear_x = msg.linear.x
        self.angular_z = msg.angular.z
    
    def initial_pose_callback(self, msg):
        self.x = msg.position.x
        self.y = msg.position.y
        # 2D Z축 회전성분 쿼터니언 값으로부터 Yaw 각도 변환 및 반영
        self.theta = 2.0 * math.atan2(msg.orientation.z, msg.orientation.w)
        self.get_logger().info("초기 위치 및 경로 기반 방향(Heading) 갱신 완료")
        
    def update_states(self):
        current_time = self.get_clock().now().to_msg()

        # 바퀴 물리 회전량 누적 연산 정리
        left_vel = self.linear_x - (self.angular_z * self.wheel_separation / 2.0)
        right_vel = self.linear_x + (self.angular_z * self.wheel_separation / 2.0)
        left_rot_vel = left_vel / self.wheel_radius
        right_rot_vel = right_vel / self.wheel_radius

        self.fl_angle = math.fmod(self.fl_angle + left_rot_vel * self.dt, 2.0 * math.pi)
        self.rl_angle = math.fmod(self.rl_angle + left_rot_vel * self.dt, 2.0 * math.pi)
        self.fr_angle = math.fmod(self.fr_angle + right_rot_vel * self.dt, 2.0 * math.pi)
        self.rr_angle = math.fmod(self.rr_angle + right_rot_vel * self.dt, 2.0 * math.pi)

        # JointState 조립 및 발행
        js = JointState()
        js.header.stamp = current_time
        js.name = ['front_left_wheel_joint', 'front_right_wheel_joint', 'rear_left_wheel_joint', 'rear_right_wheel_joint']
        js.position = [self.fl_angle, self.fr_angle, self.rl_angle, self.rr_angle]
        js.velocity = [left_rot_vel, right_rot_vel, left_rot_vel, right_rot_vel]
        self.joint_pub.publish(js)
        
        # 차체 오도메트리 위치 누적 변환
        self.theta = math.fmod(self.theta + self.angular_z * self.dt, 2.0 * math.pi)
        self.x += self.linear_x * math.cos(self.theta) * self.dt
        self.y += self.linear_x * math.sin(self.theta) * self.dt
        
        qz = math.sin(self.theta / 2.0)
        qw = math.cos(self.theta / 2.0)

        # TF 브로드캐스팅
        t = TransformStamped()
        t.header.frame_id, t.child_frame_id, t.header.stamp = 'odom', 'base_footprint', current_time
        t.transform.translation.x, t.transform.translation.y = self.x, self.y
        t.transform.rotation.z, t.transform.rotation.w = qz, qw
        self.tf_broadcaster.sendTransform(t)

        # Odom 토픽 발행
        odom = Odometry()
        odom.header.frame_id, odom.child_frame_id, odom.header.stamp = 'odom', 'base_footprint', current_time
        odom.pose.pose.position.x, odom.pose.pose.position.y = self.x, self.y
        odom.pose.pose.orientation.z, odom.pose.pose.orientation.w = qz, qw
        odom.twist.twist.linear.x, odom.twist.twist.angular.z = self.linear_x, self.angular_z
        self.odom_pub.publish(odom)

def main(args=None):
    rclpy.init(args=args)
    node = DriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()