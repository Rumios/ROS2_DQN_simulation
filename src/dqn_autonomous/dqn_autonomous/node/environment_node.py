# dqn_autonomous/node/environment_node.py
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from dqn_autonomous.map.map_parser import setup_new_episode, CELL_SIZE
from dqn_autonomous.map.pathfinding import astar  # ★ A* 알고리즘 로드
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

class EnvironmentNode(Node):
    def __init__(self):
        super().__init__('environment_node')
        
        # 최신 맵 데이터 1개만 보관
        map_qos_profile = QoSProfile(depth=1)
        map_qos_profile.reliability = ReliabilityPolicy.RELIABLE
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
        
        self.get_logger().info('QoS 설정 및 A* 경로 시각화 완료')
    
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
        
        # 1. 바닥 전체를 담당할 마커
        floor_list_marker = Marker()
        floor_list_marker.header.frame_id = 'odom'
        floor_list_marker.header.stamp = current_time
        floor_list_marker.ns = 'floor'
        floor_list_marker.id = 0
        floor_list_marker.type = Marker.CUBE_LIST
        floor_list_marker.action = Marker.ADD
        floor_list_marker.scale.x = CELL_SIZE
        floor_list_marker.scale.y = CELL_SIZE
        floor_list_marker.scale.z = 0.005
        
        # 2. 벽 전체를 담당할 마커
        wall_list_marker = Marker()
        wall_list_marker.header.frame_id = 'odom'
        wall_list_marker.header.stamp = current_time
        wall_list_marker.ns = 'walls'
        wall_list_marker.id = 1
        wall_list_marker.type = Marker.CUBE_LIST
        wall_list_marker.action = Marker.ADD
        wall_list_marker.scale.x = CELL_SIZE
        wall_list_marker.scale.y = CELL_SIZE
        wall_list_marker.scale.z = 0.5
        wall_list_marker.color.r = 0.20
        wall_list_marker.color.g = 0.20
        wall_list_marker.color.b = 0.20
        wall_list_marker.color.a = 1.0

        start_grid = None
        goal_grid = None

        # 데이터 축적 및 시작/목적지 격자 인덱스 탐색
        for r in range(rows):
            for c in range(cols):
                real_x = (c * CELL_SIZE + (CELL_SIZE / 2.0)) - (total_width / 2.0)
                real_y = ((rows - 1 - r) * CELL_SIZE + (CELL_SIZE / 2.0)) - (total_height / 2.0)
                
                # A* 연산을 위해 그리드 내부의 출발지(2)와 목적지(3) 위치 확보
                if self.grid[r][c] == 2:
                    start_grid = (r, c)
                elif self.grid[r][c] == 3:
                    goal_grid = (r, c)
                
                p_floor = Point()
                p_floor.x = real_x
                p_floor.y = real_y
                p_floor.z = -0.005
                floor_list_marker.points.append(p_floor)
                
                color_floor = ColorRGBA()
                if (r + c) % 2 == 0:
                    color_floor.r, color_floor.g, color_floor.b = 0.90, 0.90, 0.90
                else:
                    color_floor.r, color_floor.g, color_floor.b = 0.75, 0.75, 0.75
                color_floor.a = 1.0
                floor_list_marker.colors.append(color_floor)
                
                if self.grid[r][c] == 1:
                    p_wall = Point()
                    p_wall.x = real_x
                    p_wall.y = real_y
                    p_wall.z = 0.25
                    wall_list_marker.points.append(p_wall)

        marker_array.markers.append(floor_list_marker)
        marker_array.markers.append(wall_list_marker)

        # --------------------------------------------------------
        # ★ [추가] A* 경로 계산 및 빨간색 실선(LINE_STRIP) 마커 생성
        # --------------------------------------------------------
        if start_grid and goal_grid:
            grid_path = astar(self.grid, start_grid, goal_grid)
            
            if grid_path:
                path_marker = Marker()
                path_marker.header.frame_id = 'odom'
                path_marker.header.stamp = current_time
                path_marker.ns = 'astar_path'
                path_marker.id = 2                 # 고유 ID 부여
                path_marker.type = Marker.LINE_STRIP # 점들을 선으로 이어주는 타입
                path_marker.action = Marker.ADD
                
                # 실선 두께 (5cm 두께의 선)
                path_marker.scale.x = 0.05
                
                # 실선 색상 (완전한 빨간색 실선)
                path_marker.color.r = 1.0
                path_marker.color.g = 0.0
                path_marker.color.b = 0.0
                path_marker.color.a = 1.0
                
                # 격자 경로 좌표를 맵 정렬 공식에 맞춰 실수 좌표계로 변환 후 마커에 주입
                for r, c in grid_path:
                    p_path = Point()
                    p_path.x = (c * CELL_SIZE + (CELL_SIZE / 2.0)) - (total_width / 2.0)
                    p_path.y = ((rows - 1 - r) * CELL_SIZE + (CELL_SIZE / 2.0)) - (total_height / 2.0)
                    p_path.z = 0.02 # 바닥 체스판에 묻히지 않도록 Z축을 살짝(2cm) 띄움
                    path_marker.points.append(p_path)
                
                marker_array.markers.append(path_marker)

        # 시작점 실린더 추가
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
        
        # 목적지 실린더 추가
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
        
        # 최종 발행
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