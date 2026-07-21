# dqn_autonomous/map/map_parser.py
from dqn_autonomous.map.map_data import get_grid_map

CELL_SIZE = 0.6

def parse_map_to_coordinates(grid):
    walls = []
    rows = len(grid)
    cols = len(grid[0])
    
    total_width = cols * CELL_SIZE
    total_height = rows * CELL_SIZE
    
    for row_idx in range(rows):
        for col_idx in range(cols):
            real_x = (col_idx * CELL_SIZE + (CELL_SIZE / 2.0)) - (total_width / 2.0)
            real_y = ((rows - 1 - row_idx) * CELL_SIZE + (CELL_SIZE / 2.0)) - (total_height / 2.0)
            
            if grid[row_idx][col_idx] == 1:
                walls.append((real_x, real_y))
                
    return walls

def get_positions_by_value(grid, value_to_find):
    """출발지/목적지 좌표도 맵 중심 매프포지션에 연동하여 파싱합니다."""
    rows = len(grid)
    cols = len(grid[0])
    total_width = cols * CELL_SIZE
    total_height = rows * CELL_SIZE
    
    for row_idx in range(rows):
        for col_idx in range(cols):
            if grid[row_idx][col_idx] == value_to_find:
                real_x = (col_idx * CELL_SIZE + (CELL_SIZE / 2.0)) - (total_width / 2.0)
                real_y = ((rows - 1 - row_idx) * CELL_SIZE + (CELL_SIZE / 2.0)) - (total_height / 2.0)
                return real_x, real_y
    return 0.0, 0.0

def setup_new_episode():
    grid = get_grid_map()
    walls = parse_map_to_coordinates(grid)
    start_x, start_y = get_positions_by_value(grid, 2)
    goal_x, goal_y = get_positions_by_value(grid, 3)
    
    return grid, walls, start_x, start_y, goal_x, goal_y