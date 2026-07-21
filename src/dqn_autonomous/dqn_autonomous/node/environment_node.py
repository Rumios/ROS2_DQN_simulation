# dqn_autonomous/node/environment_node.py
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, Pose
from std_msgs.msg import ColorRGBA
import math
from dqn_autonomous.map.map_parser import setup_new_episode, CELL_SIZE, grid_to_world
from dqn_autonomous.map.pathfinding import astar
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

class EnvironmentNode(Node):
    def __init__(self):
        super().__init__('environment_node')
        
        map_qos_profile = QoSProfile(depth=1)
        map_qos_profile.reliability = ReliabilityPolicy.RELIABLE
        map_qos_profile.durability = DurabilityPolicy.TRANSIENT_LOCAL
        
        self.marker_pub = self.create_publisher(MarkerArray, '/visualization_marker_array', map_qos_profile)
        self.init_pos_pub = self.create_publisher(Pose, '/initial_pose', map_qos_profile)
        
        # 데이터 획득 및 A* 경로 산출
        self.grid, self.walls, self.start_x, self.start_y, self.goal_x, self.goal_y, start_grid, goal_grid = setup_new_episode()
        self.grid_path = astar(self.grid, start_grid, goal_grid)
        
        # [기능 추가] 바로 다음 웨이포인트를 바라보는 초기 Heading(Theta) 연산
        self.initial_theta = 0.0
        if self.grid_path and len(self.grid_path) > 1:
            rows, cols = len(self.grid), len(self.grid[0])
            x0, y0 = grid_to_world(self.grid_path[0][0], self.grid_path[0][1], rows, cols)
            x1, y1 = grid_to_world(self.grid_path[1][0], self.grid_path[1][1], rows, cols)
            self.initial_theta = math.atan2(y1 - y0, x1 - x0)
        
        self.publish_initial_pose()
        self.publish_3d_map()
        self.get_logger().info('환경 데이터 로드 및 초기 포즈 설정 완료')
    
    def publish_initial_pose(self):
        msg = Pose()
        msg.position.x, msg.position.y = self.start_x, self.start_y
        msg.orientation.z = math.sin(self.initial_theta / 2.0)
        msg.orientation.w = math.cos(self.initial_theta / 2.0)
        self.init_pos_pub.publish(msg)
    
    def publish_3d_map(self):
        marker_array = MarkerArray()
        current_time = self.get_clock().now().to_msg()
        rows, cols = len(self.grid), len(self.grid[0])
        
        # 바닥 CUBE_LIST 마커
        floor = Marker(ns='floor', id=0, type=Marker.CUBE_LIST, action=Marker.ADD)
        floor.header.frame_id, floor.header.stamp = 'odom', current_time
        floor.scale.x, floor.scale.y, floor.scale.z = CELL_SIZE, CELL_SIZE, 0.005
        
        # 벽 CUBE_LIST 마커
        walls = Marker(ns='walls', id=1, type=Marker.CUBE_LIST, action=Marker.ADD)
        walls.header.frame_id, walls.header.stamp = 'odom', current_time
        walls.scale.x, walls.scale.y, walls.scale.z = CELL_SIZE, CELL_SIZE, 0.5
        walls.color.r = walls.color.g = walls.color.b = 0.2
        walls.color.a = 1.0

        for r in range(rows):
            for c in range(cols):
                rx, ry = grid_to_world(r, c, rows, cols)
                floor.points.append(Point(x=rx, y=ry, z=-0.005))
                
                # 체스판 패턴 색상
                color = ColorRGBA(r=0.9, g=0.9, b=0.9, a=1.0) if (r + c) % 2 == 0 else ColorRGBA(r=0.75, g=0.75, b=0.75, a=1.0)
                floor.colors.append(color)
                
                if self.grid[r][c] == 1:
                    walls.points.append(Point(x=rx, y=ry, z=0.25))

        marker_array.markers.extend([floor, walls])

        # A* 경로 LINE_STRIP 마커
        if self.grid_path:
            path = Marker(ns='astar_path', id=2, type=Marker.LINE_STRIP, action=Marker.ADD)
            path.header.frame_id, path.header.stamp = 'odom', current_time
            path.scale.x = 0.05
            path.color.r = path.color.a = 1.0
            
            for r, c in self.grid_path:
                rx, ry = grid_to_world(r, c, rows, cols)
                path.points.append(Point(x=rx, y=ry, z=0.02))
            marker_array.markers.append(path)

        # 시작/목적지 실린더 생성 헬퍼
        def build_cylinder(idx, x, y, r, g, b):
            m = Marker(ns='positions', id=idx, type=Marker.CYLINDER, action=Marker.ADD)
            m.header.frame_id, m.header.stamp = 'odom', current_time
            m.pose.position.x, m.pose.position.y, m.pose.position.z = x, y, 0.01
            m.pose.orientation.w = 1.0
            m.scale.x, m.scale.y, m.scale.z = 0.4, 0.4, 0.02
            m.color.r, m.color.g, m.color.b, m.color.a = r, g, b, 0.9
            return m

        marker_array.markers.append(build_cylinder(99999, self.start_x, self.start_y, 0.0, 1.0, 0.0))
        marker_array.markers.append(build_cylinder(99998, self.goal_x, self.goal_y, 1.0, 1.0, 0.0))
        
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