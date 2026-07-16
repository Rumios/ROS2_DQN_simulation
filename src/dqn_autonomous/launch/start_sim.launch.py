import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 1. 패키지 이름과 URDF 파일 이름 정의
    pkg_name = 'dqn_autonomous'
    urdf_file = 'robot.urdf'

    pkg_share = get_package_share_directory(pkg_name)
    urdf_path = os.path.join(pkg_share, 'urdf', urdf_file)

    # 4. 안전하게 절대 경로에서 파일 내용 읽기
    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc}]
    )
    
    driver_node = Node(
        package='dqn_autonomous',
        executable='driver_node',
        name='driver_node', 
        output='screen'
    )

    ui_node = Node(
        package='dqn_autonomous',
        executable='ui_node',
        name='ui_node',
        output='screen'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen'
    )

    # 모든 노드 실행
    return LaunchDescription([
        robot_state_publisher_node,
        driver_node,
        ui_node,
        rviz_node
    ])