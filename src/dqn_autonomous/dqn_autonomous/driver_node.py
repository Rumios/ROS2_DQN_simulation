import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
import tf2_ros
import math

class DriverNode(Node):
    def __init__(self):
        super().__init__('vehicle_driver_node')
        
        self.cmd_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        
        # TF 브로드캐스터 (odom -> base_footprint)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        
        self.wheel_radius = 0.06
        self.wheel_separation = 0.24
        
        # 실시간 누적 각도 저장
        self.fl_angle = 0.0
        self.fr_angle = 0.0
        self.rl_angle = 0.0  
        self.rr_angle = 0.0
        
        # 수신 대기용
        self.linear_x = 0.0
        self.angular_z = 0.0
        
        # 로봇 가상 2D 세계 좌표
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        
        # 0.02초마다 바퀴 상태 실시간 업데이트
        self.dt = 0.02
        self.timer = self.create_timer(self.dt, self.update_states)
        
        self.get_logger().info("드라이버 시작")
        
    def cmd_vel_callback(self, msg):
        self.linear_x = msg.linear.x
        self.angular_z = msg.angular.z
    
    def update_states(self):
        current_time = self.get_clock().now().to_msg()

        # 회전각 누적 및 각도 제한
        self.theta += self.angular_z * self.dt
        self.theta = math.fmod(self.theta, 2.0 * math.pi)
        
        # 전진 방향 삼각함수를 기반으로 X, Y 변화율 계산 후 누적
        self.x += self.linear_x * math.cos(self.theta) * self.dt
        self.y += self.linear_x * math.sin(self.theta) * self.dt
        
        # 3차원 회전을 위한 오일러 각도(Theta) -> 쿼터니언 변환 (Z축 회전 성분만 존재)
        qz = math.sin(self.theta / 2.0)
        qw = math.cos(self.theta / 2.0)


        t = TransformStamped()
        t.header.stamp = current_time
        t.header.frame_id = 'odom'              # 부모 좌표계 (세계 중심)
        t.child_frame_id = 'base_footprint'     # 자식 좌표계 (로봇 중심)
        
        # 위치 입력
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        
        # 회전 입력
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        
        # RViz2로 좌표 전송
        self.tf_broadcaster.sendTransform(t)

        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'
        
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        
        odom.twist.twist.linear.x = self.linear_x
        odom.twist.twist.angular.z = self.angular_z
        
        self.odom_pub.publish(odom)

        left_linear_vel = self.linear_x - (self.angular_z * self.wheel_separation / 2.0)
        right_linear_vel = self.linear_x + (self.angular_z * self.wheel_separation / 2.0)

        left_rot_vel = left_linear_vel / self.wheel_radius
        right_rot_vel = right_linear_vel / self.wheel_radius

        self.fl_angle += left_rot_vel * self.dt 
        self.rl_angle += left_rot_vel * self.dt 
        self.fr_angle += right_rot_vel * self.dt
        self.rr_angle += right_rot_vel * self.dt

        self.fl_angle = math.fmod(self.fl_angle, 2.0 * math.pi)
        self.rl_angle = math.fmod(self.rl_angle, 2.0 * math.pi)
        self.fr_angle = math.fmod(self.fr_angle, 2.0 * math.pi)
        self.rr_angle = math.fmod(self.rr_angle, 2.0 * math.pi)

        joint_state = JointState()
        joint_state.header.stamp = current_time

        joint_state.name = [
            'front_left_wheel_joint',
            'front_right_wheel_joint',
            'rear_left_wheel_joint',
            'rear_right_wheel_joint'
        ]

        joint_state.position = [
            self.fl_angle,
            self.fr_angle,
            self.rl_angle,
            self.rr_angle
        ]

        joint_state.velocity = [
            left_rot_vel,
            right_rot_vel,
            left_rot_vel,
            right_rot_vel
        ]

        self.joint_pub.publish(joint_state)
        

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