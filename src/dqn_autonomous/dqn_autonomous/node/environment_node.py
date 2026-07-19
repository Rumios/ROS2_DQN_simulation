# dqn_autonomous/node/environment_node.py
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from dqn_autonomous.map.map_parser import setup_new_episode, CELL_SIZE
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

class EnvironmentNode(Node):
    def __init__(self):
        super().__init__('environment_node')
        
        # 최신 맵 데이터 1개만 보관
        map_qos_profile = QoSProfile(depth=1)
        # RELIABLE : 데이터 유실 시 재전송 요청하면 반드시 목적지 도달
        map_qos_profile.reliability = ReliabilityPolicy.RELIABLE
        # TRANSIENT_LOCAL : publisher가 데이터를 보관함에 가지고 있다가 켜지면 가져가기
        map_qos_profile.durability = DurabilityPolicy.TRANSIENT_LOCAL
        
        self.marker_pub = self.create_publisher(
            MarkerArray,
            '/visualization_marker_array',
            map_qos_profile
        )
        
        self.init_pos_pub = self.create_publisher(
            Point,
            '/initial_pose',
            map_qos_profile
        )
        
        self.grid, self.walls, self.start_x, self.start_y, self.goal_x, self.goal_y = setup_new_episode()
        
        self.publish_3d_map()
        
        self.publish_initial_pose()
        
        self.get_logger().info('QoS 설정 완료')
    
    def publish_initial_pose(self):
        init_point = Point()
        init_point.x = self.start_x
        init_point.y = self.start_y
        init_point.z = 0.0
        self.init_pos_pub.publish(init_point)
    
    def publish_3d_map(self):
        """2D -> 3D"""
        marker_array = MarkerArray()
        current_time = self.get_clock().now().to_msg()
        
        rows = len(self.grid)
        cols = len(self.grid[0])
        
        total_width = cols * CELL_SIZE
        total_height = rows * CELL_SIZE
        
        tile_id = 0
        for r in range(rows):
            for c in range(cols):
                tile_marker = Marker()
                tile_marker.header.frame_id = 'odom'
                tile_marker.header.stamp = current_time
                tile_marker.ns = 'floor'
                tile_marker.id = tile_id
                tile_marker.type = Marker.CUBE
                tile_marker.action = Marker.ADD
                
                # map_parser와 완벽히 일치하는 정중앙 정렬 좌표 계산 방식 적용
                real_x = (c * CELL_SIZE + (CELL_SIZE / 2.0)) - (total_width / 2.0)
                real_y = ((rows - 1 - r) * CELL_SIZE + (CELL_SIZE / 2.0)) - (total_height / 2.0)
                
                tile_marker.pose.position.x = real_x
                tile_marker.pose.position.y = real_y
                tile_marker.pose.position.z = -0.005  # 바닥선 인터페이스 방지용 미세 다운
                
                # 0.6m x 0.6m 정교한 정사각형 타일 크기 설정
                tile_marker.scale.x = CELL_SIZE
                tile_marker.scale.y = CELL_SIZE
                tile_marker.scale.z = 0.005
                
                # 행과 열의 합이 짝수냐 홀수냐에 따라 색상을 교차시킴 (체스판 무늬 패턴)
                if (r + c) % 2 == 0:
                    # 밝은 회색 타일
                    tile_marker.color.r = 0.90
                    tile_marker.color.g = 0.90
                    tile_marker.color.b = 0.90
                else:
                    # 약간 어두운 회색 타일
                    tile_marker.color.r = 0.75
                    tile_marker.color.g = 0.75
                    tile_marker.color.b = 0.75
                    
                tile_marker.color.a = 1.0
                marker_array.markers.append(tile_marker)
                tile_id += 1

        # 시작점 추가
        start_marker = Marker()
        start_marker.header.frame_id = 'odom'
        start_marker.header.stamp = current_time
        start_marker.ns = 'positions'
        start_marker.id = 99999
        start_marker.type = Marker.CYLINDER
        start_marker.action = Marker.ADD
        
        start_marker.pose.position.x = self.start_x
        start_marker.pose.position.y = self.start_y
        start_marker.pose.position.z = 0.01
        
        start_marker.pose.orientation.x = 0.0
        start_marker.pose.orientation.y = 0.0
        start_marker.pose.orientation.z = 0.0
        start_marker.pose.orientation.w = 1.0
        
        start_marker.scale.x = 0.4
        start_marker.scale.y = 0.4
        start_marker.scale.z = 0.02
        
        start_marker.color.r = 0.0
        start_marker.color.g = 1.0
        start_marker.color.b = 0.0
        start_marker.color.a = 0.9
        marker_array.markers.append(start_marker)
        
        # 목적지 추가
        goal_marker = Marker()
        goal_marker.header.frame_id = 'odom'
        goal_marker.header.stamp = current_time
        goal_marker.ns = 'positions'
        goal_marker.id = 99998  # ID가 겹치지 않도록 차별화
        goal_marker.type = Marker.CYLINDER
        goal_marker.action = Marker.ADD
        
        goal_marker.pose.position.x = self.goal_x
        goal_marker.pose.position.y = self.goal_y
        goal_marker.pose.position.z = 0.01
        
        goal_marker.pose.orientation.x = 0.0
        goal_marker.pose.orientation.y = 0.0
        goal_marker.pose.orientation.z = 0.0
        goal_marker.pose.orientation.w = 1.0
        
        goal_marker.scale.x = 0.4
        goal_marker.scale.y = 0.4
        goal_marker.scale.z = 0.02
        
        # 빛나는 옐로우 톤 (R + G)
        goal_marker.color.r = 1.0
        goal_marker.color.g = 1.0
        goal_marker.color.b = 0.0
        goal_marker.color.a = 0.9
        marker_array.markers.append(goal_marker)
        
        # 마커 패키지 일괄 발송
        self.marker_pub.publish(marker_array)
    
def main(args=None):
    rclpy.init(args=args)
    node = EnvironmentNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()