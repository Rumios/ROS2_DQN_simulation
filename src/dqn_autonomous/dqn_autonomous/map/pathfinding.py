import math

class Node:
    def __init__(self, parent=None, position=None):
        self.parent = parent
        self.position = position
        self.g = 0
        self.h = 0
        self.f = 0

    def __eq__(self, other):
        return self.position == other.position

def astar(grid, start, goal):
    """
    grid: 2차원 배열 맵 데이터 (0: 빈길, 1: 벽, 2: 출발지, 3: 목적지)
    start: (row, col) 시작 타일 좌표
    goal: (row, col) 목표 타일 좌표
    """
    if not start or not goal:
        return None

    start_node = Node(None, start)
    goal_node = Node(None, goal)

    open_list = [start_node]
    closed_list = []

    # 8방향 이동 변량 정의 (상하좌우 가중치 1.0, 대각선 가중치 1.414)
    movements = [
        (-1, 0, 1.0, False), (1, 0, 1.0, False), (0, -1, 1.0, False), (0, 1, 1.0, False),
        (-1, -1, 1.414, True), (-1, 1, 1.414, True), (1, -1, 1.414, True), (1, 1, 1.414, True)
    ]

    rows = len(grid)
    cols = len(grid[0])

    while open_list:
        current_node = open_list[0]
        current_index = 0
        for index, item in enumerate(open_list):
            if item.f < current_node.f:
                current_node = item
                current_index = index

        open_list.pop(current_index)
        closed_list.append(current_node)

        if current_node == goal_node:
            path = []
            current = current_node
            while current is not None:
                path.append(current.position)
                current = current.parent
            return path[::-1]

        # 주변 8방향 확장
        for r_move, c_move, move_cost, is_diagonal in movements:
            nr = current_node.position[0] + r_move
            nc = current_node.position[1] + c_move

            # 맵 범위를 벗어나는지 체크
            if nr < 0 or nr >= rows or nc < 0 or nc >= cols:
                continue

            # 이동하려는 타일 자체가 벽(1)이면 패스
            if grid[nr][nc] == 1:
                continue

            # 대각선 이동일 때, 모서리를 긁지 않도록 필터링
            if is_diagonal:
                side_tile_1 = grid[current_node.position[0]][nc]
                side_tile_2 = grid[nr][current_node.position[1]]
                
                if side_tile_1 == 1 or side_tile_2 == 1:
                    continue

            # --- [추가된 로직] 1칸 마진 페널티 (안전 거리 확보) ---
            margin_penalty = 0.0
            # 이동하려는 타일(nr, nc)을 기준으로 주변 8칸을 다시 검사
            for dr in [-1, 0, 1]:
                for dc in [-1, 0, 1]:
                    if dr == 0 and dc == 0:
                        continue
                    ar, ac = nr + dr, nc + dc
                    if 0 <= ar < rows and 0 <= ac < cols:
                        if grid[ar][ac] == 1:
                            # 주변 1칸 내에 벽이 하나 있을 때마다 가중치 10 추가
                            # 경로가 외곽(벽)이 아닌 도로의 중앙을 선호하도록 강제함
                            margin_penalty += 10.0
            # -----------------------------------------------------

            new_node = Node(current_node, (nr, nc))

            if new_node in closed_list:
                continue

            # 이동 비용(g)에 물리적 이동 거리뿐만 아니라 '마진 페널티'를 합산
            new_node.g = current_node.g + move_cost + margin_penalty
            new_node.h = math.sqrt((new_node.position[0] - goal_node.position[0])**2 + 
                                   (new_node.position[1] - goal_node.position[1])**2)
            new_node.f = new_node.g + new_node.h

            in_open = False
            for open_node in open_list:
                if new_node == open_node:
                    in_open = True
                    if new_node.g < open_node.g:
                        open_node.g = new_node.g
                        open_node.f = new_node.f
                        open_node.parent = new_node.parent
                    break

            if not in_open:
                open_list.append(new_node)

    return None