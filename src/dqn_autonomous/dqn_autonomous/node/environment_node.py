# dqn_autonomous/node/environment_node.py
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
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
        
        # ==========================================
        # [변경] 바닥 전체를 담당할 '단 하나의 마커' 생성
        # ==========================================
        floor_list_marker = Marker()
        floor_list_marker.header.frame_id = 'odom'
        floor_list_marker.header.stamp = current_time
        floor_list_marker.ns = 'floor'
        floor_list_marker.id = 0                   # 단 하나의 고유 ID
        floor_list_marker.type = Marker.CUBE_LIST  # 큐브 리스트 타입으로 변경
        floor_list_marker.action = Marker.ADD
        
        # 각 개별 큐브 요소들의 기본 크기 정의
        floor_list_marker.scale.x = CELL_SIZE
        floor_list_marker.scale.y = CELL_SIZE
        floor_list_marker.scale.z = 0.005
        
        # ==========================================
        # [변경] 벽 전체를 담당할 '단 하나의 마커' 생성
        # ==========================================
        wall_list_marker = Marker()
        wall_list_marker.header.frame_id = 'odom'
        wall_list_marker.header.stamp = current_time
        wall_list_marker.ns = 'walls'
        wall_list_marker.id = 1                    # 단 하나의 고유 ID
        wall_list_marker.type = Marker.CUBE_LIST   # 큐브 리스트 타입으로 변경
        wall_list_marker.action = Marker.ADD
        
        # 각 개별 벽 큐브들의 크기 정의
        wall_list_marker.scale.x = CELL_SIZE
        wall_list_marker.scale.y = CELL_SIZE
        wall_list_marker.scale.z = 0.5
        
        # 벽 무리의 공통 색상 설정 (매트 다크 그레이)
        wall_list_marker.color.r = 0.20
        wall_list_marker.color.g = 0.20
        wall_list_marker.color.b = 0.20
        wall_list_marker.color.a = 1.0

        # 데이터 축적 루프 시작
        for r in range(rows):
            for c in range(cols):
                # 정중앙 정렬 좌표 계산
                real_x = (c * CELL_SIZE + (CELL_SIZE / 2.0)) - (total_width / 2.0)
                real_y = ((rows - 1 - r) * CELL_SIZE + (CELL_SIZE / 2.0)) - (total_height / 2.0)
                
                # 1. 바닥 좌표 객체 생성 후 리스트에 추가
                p_floor = Point()
                p_floor.x = real_x
                p_floor.y = real_y
                p_floor.z = -0.005
                floor_list_marker.points.append(p_floor)
                
                # 2. 바닥 체스판 패턴 색상을 생성 후 리스트에 추가
                color_floor = ColorRGBA()
                if (r + c) % 2 == 0:
                    color_floor.r, color_floor.g, color_floor.b = 0.90, 0.90, 0.90
                else:
                    color_floor.r, color_floor.g, color_floor.b = 0.75, 0.75, 0.75
                color_floor.a = 1.0
                floor_list_marker.colors.append(color_floor)
                
                # 3. 벽 데이터(1)인 경우, 벽 리스트 마커에 좌표만 추가
                if self.grid[r][c] == 1:
                    p_wall = Point()
                    p_wall.x = real_x
                    p_wall.y = real_y
                    p_wall.z = 0.25
                    wall_list_marker.points.append(p_wall)

        # 리스트 마커들을 배열에 탑재
        marker_array.markers.append(floor_list_marker)
        marker_array.markers.append(wall_list_marker)

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
        goal_marker.id = 99998
        goal_marker.type = Marker.CYLINDER
        goal_marker.action = Marker.ADD
        
        goal_marker.pose.position.x = self.goal_x
        goal_marker.pose.position.y = self.goal_y
        goal_marker.pose.position.z = 0.01
        
        goal_marker.pose.orientation.w = 1.0
        goal_marker.scale.x = 0.4
        goal_marker.scale.y = 0.4
        goal_marker.scale.z = 0.02
        
        goal_marker.color.r = 1.0
        goal_marker.color.g = 1.0
        goal_marker.color.b = 0.0
        goal_marker.color.a = 0.9
        marker_array.markers.append(goal_marker)
        
        # 최종 발행 (이제 단 4개의 마커 오브젝트만 통신망을 타고 흐릅니다)
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