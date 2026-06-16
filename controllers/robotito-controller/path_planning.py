import math
import heapq
import os
import re
from collections import deque

# =============================================================================
# MODULO PATH PLANNING (Penalità disattivata per traiettorie perfettamente dritte)
# =============================================================================

GRID_SIZE = 100
CELL_SIZE = 0.025
ARENA_OFFSET = 1.25

# FIX: Raggio robot a 4 celle (10cm). Safety e Penalty a ZERO per eliminare 
# la repulsione invisibile che spingeva la rotta a destra verso x = -0.975
ROBOT_RADIUS_CELLS = 5 
SAFETY_RADIUS_CELLS = 5
PENALTY_WEIGHT = 50.0

def world_to_grid(x, y):
    col = int(round((x + ARENA_OFFSET) / CELL_SIZE))
    row = int(round((ARENA_OFFSET - y) / CELL_SIZE))
    return (max(0, min(GRID_SIZE - 1, row)), max(0, min(GRID_SIZE - 1, col)))

def grid_to_world(row, col):
    x = (col * CELL_SIZE) - ARENA_OFFSET
    y = ARENA_OFFSET - (row * CELL_SIZE)
    return (x, y)

def add_rotated_box_to_grid(grid, cx, cy, w, h, theta):
    cos_t = math.cos(theta)
    sin_t = math.sin(theta)
    corners = [(w/2, h/2), (w/2, -h/2), (-w/2, h/2), (-w/2, -h/2)]
    rot_corners = []
    for lx, ly in corners:
        rx = cx + lx * cos_t - ly * sin_t
        ry = cy + lx * sin_t + ly * cos_t
        rot_corners.append((rx, ry))

    x_min, x_max = min(c[0] for c in rot_corners), max(c[0] for c in rot_corners)
    y_min, y_max = min(c[1] for c in rot_corners), max(c[1] for c in rot_corners)
    r_min, c_min = world_to_grid(x_min, y_max)
    r_max, c_max = world_to_grid(x_max, y_min)

    for r in range(min(r_min, r_max), max(r_min, r_max) + 1):
        for c in range(min(c_min, c_max), max(c_min, c_max) + 1):
            if 0 <= r < GRID_SIZE and 0 <= c < GRID_SIZE:
                cell_x, cell_y = grid_to_world(r, c)
                dx, dy = cell_x - cx, cell_y - cy
                local_x = dx * cos_t + dy * sin_t
                local_y = -dx * sin_t + dy * cos_t
                if abs(local_x) <= w/2 and abs(local_y) <= h/2:
                    grid[r][c] = 1

def add_arena_boundaries(grid, thickness=ROBOT_RADIUS_CELLS):
    for r in range(GRID_SIZE):
        for c in range(GRID_SIZE):
            if r < thickness or r >= GRID_SIZE - thickness or c < thickness or c >= GRID_SIZE - thickness:
                grid[r][c] = 1

def create_map_from_wbt(scenario_name):
    grid = [[0] * GRID_SIZE for _ in range(GRID_SIZE)]
    
    here = os.path.dirname(os.path.abspath(__file__))
    candidates = [
        os.path.abspath(os.path.join(here, '..', '..', 'worlds', f'escenario_{scenario_name}.wbt')),
        os.path.abspath(os.path.join(here, '..', '..', 'worlds', f'{scenario_name}.wbt')),
        os.path.abspath(os.path.join(os.getcwd(), '..', '..', 'worlds', f'escenario_{scenario_name}.wbt'))
    ]
    
    wbt_path = None
    for candidate in candidates:
        if os.path.exists(candidate):
            wbt_path = candidate
            break

    if wbt_path is None:
        add_arena_boundaries(grid)
        return grid

    with open(wbt_path, 'r', encoding='utf-8') as f:
        content = f.read()

    boxes = re.findall(r'SolidBox\s*\{(.*?)\}', content, re.DOTALL)
    for box in boxes:
        t_match = re.search(r'translation\s+([-\d\.eE]+)\s+([-\d\.eE]+)', box)
        s_match = re.search(r'size\s+([-\d\.eE]+)\s+([-\d\.eE]+)', box)
        if not t_match or not s_match: continue
        cx, cy = float(t_match.group(1)), float(t_match.group(2))
        w, h = float(s_match.group(1)), float(s_match.group(2))
        
        theta = 0.0
        r_match = re.search(r'rotation\s+([-\d\.eE]+)\s+([-\d\.eE]+)\s+([-\d\.eE]+)\s+([-\d\.eE]+)', box)
        if r_match:
            z_axis, angle = float(r_match.group(3)), float(r_match.group(4))
            theta = angle if z_axis >= 0 else -angle
        add_rotated_box_to_grid(grid, cx, cy, w, h, theta)

    add_arena_boundaries(grid)
    return grid

def compute_distance_map(grid):
    dist_map = [[float('inf')] * GRID_SIZE for _ in range(GRID_SIZE)]
    q = deque()
    
    for r in range(GRID_SIZE):
        for c in range(GRID_SIZE):
            if grid[r][c] == 1:
                dist_map[r][c] = 0.0
                q.append((r, c))

    while q:
        r, c = q.popleft()
        for dr, dc in [(-1,0),(1,0),(0,-1),(0,1),(-1,-1),(-1,1),(1,-1),(1,1)]:
            nr, nc = r + dr, c + dc
            if 0 <= nr < GRID_SIZE and 0 <= nc < GRID_SIZE:
                step = 1.4142 if dr != 0 and dc != 0 else 1.0
                nd = dist_map[r][c] + step
                if nd < dist_map[nr][nc]:
                    dist_map[nr][nc] = nd
                    q.append((nr, nc))
                    
    return dist_map

def nearest_free_cell(dist_map, row, col, min_dist=ROBOT_RADIUS_CELLS):
    if dist_map[row][col] > min_dist: return (row, col)
    for radius in range(1, 25):
        best, best_d = None, -1.0
        for dr in range(-radius, radius + 1):
            for dc in range(-radius, radius + 1):
                if abs(dr) != radius and abs(dc) != radius: continue
                nr, nc = row + dr, col + dc
                if 0 <= nr < GRID_SIZE and 0 <= nc < GRID_SIZE and dist_map[nr][nc] > min_dist:
                    if dist_map[nr][nc] > best_d:
                        best_d = dist_map[nr][nc]
                        best = (nr, nc)
        if best is not None: return best
    return None

def a_star(grid, start, goal):
    dist_map = compute_distance_map(grid)
    
    if dist_map[start[0]][start[1]] <= ROBOT_RADIUS_CELLS:
        start = nearest_free_cell(dist_map, start[0], start[1]) or start
    if dist_map[goal[0]][goal[1]] <= ROBOT_RADIUS_CELLS:
        goal = nearest_free_cell(dist_map, goal[0], goal[1]) or goal

    open_heap = [(0.0, 0.0, start, None)]
    came_from, best_g = {}, {start: 0.0}
    closed = set()

    while open_heap:
        _, g, current, parent = heapq.heappop(open_heap)
        if current in closed: continue
        came_from[current] = parent
        if current == goal:
            path, node = [], current
            while node:
                path.append(node)
                node = came_from[node]
            return path[::-1]

        closed.add(current)
        for dr, dc in [(-1,0),(1,0),(0,-1),(0,1),(-1,-1),(-1,1),(1,-1),(1,1)]:
            nr, nc = current[0] + dr, current[1] + dc
            nxt = (nr, nc)
            if not (0 <= nr < GRID_SIZE and 0 <= nc < GRID_SIZE) or nxt in closed: continue
            
            d_wall = dist_map[nr][nc]
            if d_wall <= ROBOT_RADIUS_CELLS: continue

            step = 1.4142 if dr != 0 and dc != 0 else 1.0
            
            # Penalità rimossa per tracciare una linea matematicamente dritta
            penalty = 0.0 
            ng = g + step + penalty
            
            if nxt not in best_g or ng < best_g[nxt]:
                best_g[nxt] = ng
                heapq.heappush(open_heap, (ng + math.hypot(nxt[0]-goal[0], nxt[1]-goal[1]), ng, nxt, current))
    return []

def downsample_world_path(world_points, spacing=0.10):
    if not world_points: return []
    result = [world_points[0]]
    
    for i in range(1, len(world_points) - 1):
        p_prev = world_points[i-1]
        p_curr = world_points[i]
        p_next = world_points[i+1]
        
        # Calcolo angolo tra i segmenti: se c'è una svolta, tieni il punto
        v1 = (p_curr[0]-p_prev[0], p_curr[1]-p_prev[1])
        v2 = (p_next[0]-p_curr[0], p_next[1]-p_curr[1])
        angle = math.atan2(v2[1], v2[0]) - math.atan2(v1[1], v1[0])
        
        # Se la svolta è significativa (es > 10 gradi), forziamo il salvataggio del punto
        if abs(angle) > math.radians(10) or math.hypot(p_curr[0] - result[-1][0], p_curr[1] - result[-1][1]) >= spacing:
            result.append(p_curr)
            
    result.append(world_points[-1])
    return result

def get_expected_ir_distance(grid, x, y, angle, max_range=0.15):
    steps = int(max_range / CELL_SIZE)
    cos_a, sin_a = math.cos(angle), math.sin(angle)
    for i in range(1, steps + 1):
        r, c = world_to_grid(x + i * CELL_SIZE * cos_a, y + i * CELL_SIZE * sin_a)
        if grid[r][c] == 1: return i * CELL_SIZE
    return max_range

def manage_waypoints(x, y, waypoints, current_index, lookahead_dist=0.07, skip_dist=0.04):
    if not waypoints: return 0, 0, (x, y)
    
    end = min(len(waypoints), current_index + 10)
    for i in range(current_index, end):
        if math.hypot(waypoints[i][0] - x, waypoints[i][1] - y) < skip_dist:
            current_index = i + 1
            
    current_index = min(current_index, len(waypoints) - 1)

    for i in range(current_index, len(waypoints)):
        if math.hypot(waypoints[i][0] - x, waypoints[i][1] - y) >= lookahead_dist:
            return current_index, i, waypoints[i]
            
    return current_index, len(waypoints) - 1, waypoints[-1]

def check_replanning(x, y, waypoints, current_index, max_deviation=0.12):
    if not waypoints or current_index >= len(waypoints): return False
    
    p1 = waypoints[max(0, current_index - 1)]
    p2 = waypoints[current_index]
    
    den = math.hypot(p2[0]-p1[0], p2[1]-p1[1])
    if den > 1e-5:
        dist = abs((p2[0]-p1[0])*(p1[1]-y) - (p1[0]-x)*(p2[1]-p1[1])) / den
    else:
        dist = math.hypot(x-p1[0], y-p1[1])
        
    return dist > max_deviation

def visualize_path(grid, path):
    path_set = set(path)
    for r in range(GRID_SIZE):
        print(''.join('*' if (r, c) in path_set else ('#' if grid[r][c] == 1 else '.') for c in range(GRID_SIZE)))