# dqn_autonomous/map/map_parser.py
from dqn_autonomous.map.map_data import get_grid_map

CELL_SIZE = 0.6

def grid_to_world(r, c, rows, cols):
    """그리드 인덱스를 실제 3D 세계 메트릭 좌표계로 변환"""
    total_width = cols * CELL_SIZE
    total_height = rows * CELL_SIZE
    x = (c * CELL_SIZE + (CELL_SIZE / 2.0)) - (total_width / 2.0)
    y = ((rows - 1 - r) * CELL_SIZE + (CELL_SIZE / 2.0)) - (total_height / 2.0)
    return x, y

def setup_new_episode():
    """새 에피소드 맵을 생성하고 필요한 모든 물리 좌표 및 인덱스를 단 한번의 스캔으로 파싱"""
    grid = get_grid_map()
    rows, cols = len(grid), len(grid[0])
    
    walls = []
    start_x, start_y = 0.0, 0.0
    goal_x, goal_y = 0.0, 0.0
    start_grid, goal_grid = None, None
    
    for r in range(rows):
        for c in range(cols):
            if grid[r][c] == 1:
                walls.append(grid_to_world(r, c, rows, cols))
            elif grid[r][c] == 2:
                start_grid = (r, c)
                start_x, start_y = grid_to_world(r, c, rows, cols)
            elif grid[r][c] == 3:
                goal_grid = (r, c)
                goal_x, goal_y = grid_to_world(r, c, rows, cols)
                
    return grid, walls, start_x, start_y, goal_x, goal_y, start_grid, goal_grid