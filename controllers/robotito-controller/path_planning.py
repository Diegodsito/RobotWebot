import math
import heapq
import os
import re
from collections import deque

GRID_SIZE = 100
CELL_SIZE = 0.025
ARENA_OFFSET = 1.25

ROBOT_RADIUS_CELLS = 4    # Raggio fisico "duro" a 7.5 cm (margine perfetto per l'e-puck)
SAFETY_RADIUS_CELLS = 8   # Raggio fittizio ridotto a 20 cm
PENALTY_WEIGHT = 5.0      # FIX: aumentato da 2.0 → 5.0 per forzare A* più al centro dei corridoi

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

def add_arena_boundaries(grid, thickness=None):
    """
    Marca i bordi della griglia come ostacoli con uno spessore pari a
    ROBOT_RADIUS_CELLS (default), così la distance map li tratta esattamente
    come i muri interni e A* mantiene lo stesso margine di sicurezza.

    Senza questa funzione il robot ignora i confini del mondo Webots
    e ci va a sbattere direttamente.
    """
    if thickness is None:
        thickness = ROBOT_RADIUS_CELLS

    for r in range(GRID_SIZE):
        for c in range(GRID_SIZE):
            if r < thickness or r >= GRID_SIZE - thickness:
                grid[r][c] = 1
            elif c < thickness or c >= GRID_SIZE - thickness:
                grid[r][c] = 1

def create_map_from_wbt(scenario_name):
    """
    Legge dinamicamente il file .wbt e mappa i SolidBox.
    Cerca il file nella cartella 'worlds' standard di Webots.
    """
    grid = [[0] * GRID_SIZE for _ in range(GRID_SIZE)]

    wbt_path = f"../../worlds/escenario_{scenario_name}.wbt"
    if not os.path.exists(wbt_path):
        wbt_path = f"../../worlds/{scenario_name}.wbt"
        if not os.path.exists(wbt_path):
            print(f"[ERROR] Impossibile trovare il file mondo per '{scenario_name}' nel percorso {wbt_path}.")
            print("[INFO] Verrà creata una mappa vuota con soli bordi arena.")
            add_arena_boundaries(grid)
            return grid

    with open(wbt_path, 'r', encoding='utf-8') as f:
        content = f.read()

    box_pattern = re.compile(r'SolidBox\s*\{(.*?)\}', re.DOTALL)
    boxes = box_pattern.findall(content)

    # FIX (consiglio): log del numero di box trovati per verificare il parsing
    print(f"[Init] Trovati {len(boxes)} SolidBox nel file .wbt")

    for box in boxes:
        t_match = re.search(r'translation\s+([-\d\.eE]+)\s+([-\d\.eE]+)', box)
        if not t_match:
            continue
        cx, cy = float(t_match.group(1)), float(t_match.group(2))

        s_match = re.search(r'size\s+([-\d\.eE]+)\s+([-\d\.eE]+)', box)
        if not s_match:
            continue
        w, h = float(s_match.group(1)), float(s_match.group(2))

        theta = 0.0
        r_match = re.search(
            r'rotation\s+([-\d\.eE]+)\s+([-\d\.eE]+)\s+([-\d\.eE]+)\s+([-\d\.eE]+)',
            box
        )
        if r_match:
            z_axis = float(r_match.group(3))
            angle  = float(r_match.group(4))
            theta  = angle if z_axis > 0 else -angle

        add_rotated_box_to_grid(grid, cx, cy, w, h, theta)

    # FIX: aggiunge i bordi dell'arena come ostacoli
    # Prima mancava questa chiamata — il robot non "vedeva" i confini del mondo
    # e ci andava a sbattere direttamente.
    add_arena_boundaries(grid)
    print(f"[Init] Bordi arena aggiunti alla mappa (spessore: {ROBOT_RADIUS_CELLS} celle = {ROBOT_RADIUS_CELLS * CELL_SIZE * 100:.1f} cm).")

    return grid

def compute_distance_map(grid):
    dist_map = [[float('inf')] * GRID_SIZE for _ in range(GRID_SIZE)]
    queue = deque()

    for r in range(GRID_SIZE):
        for c in range(GRID_SIZE):
            if grid[r][c] == 1:
                dist_map[r][c] = 0
                queue.append((r, c))

    directions = [(-1,0),(1,0),(0,-1),(0,1),(-1,-1),(-1,1),(1,-1),(1,1)]
    while queue:
        r, c = queue.popleft()
        current_dist = dist_map[r][c]
        for dr, dc in directions:
            nr, nc = r + dr, c + dc
            if 0 <= nr < GRID_SIZE and 0 <= nc < GRID_SIZE:
                cost_step = 1.414 if dr != 0 and dc != 0 else 1.0
                if current_dist + cost_step < dist_map[nr][nc]:
                    dist_map[nr][nc] = current_dist + cost_step
                    queue.append((nr, nc))
    return dist_map

class Node:
    def __init__(self, position, parent=None):
        self.position = position
        self.parent   = parent
        self.g = 0.0
        self.h = 0.0
        self.f = 0.0

    def __lt__(self, other):
        return self.f < other.f

def heuristic(current, goal):
    return math.sqrt((current[0] - goal[0]) ** 2 + (current[1] - goal[1]) ** 2)

def nearest_free_cell(dist_map, row, col, min_dist=None):
    """
    Cerca la cella libera più vicina a (row, col) con dist_muro > min_dist.
    Usato per il replan quando la posizione stimata cade dentro una zona ostacolo
    o troppo vicina al bordo (ad es. per drift di odometria).
    Esplora a spirale con raggio crescente fino a max 10 celle.
    """
    if min_dist is None:
        min_dist = ROBOT_RADIUS_CELLS

    if dist_map[row][col] > min_dist:
        return (row, col)

    for radius in range(1, 11):
        best = None
        best_d = -1.0
        for dr in range(-radius, radius + 1):
            for dc in range(-radius, radius + 1):
                if abs(dr) != radius and abs(dc) != radius:
                    continue  # solo il perimetro del quadrato
                nr, nc = row + dr, col + dc
                if 0 <= nr < GRID_SIZE and 0 <= nc < GRID_SIZE:
                    d = dist_map[nr][nc]
                    if d > min_dist and d > best_d:
                        best_d = d
                        best = (nr, nc)
        if best is not None:
            return best
    return None  # nessuna cella libera trovata nel raggio

def a_star(grid, start, goal):
    dist_map = compute_distance_map(grid)

    # Se lo start è troppo vicino a un muro (es. per drift di odometria),
    # sposta il punto di partenza alla cella libera più vicina.
    if dist_map[start[0]][start[1]] <= ROBOT_RADIUS_CELLS:
        snapped = nearest_free_cell(dist_map, start[0], start[1])
        if snapped is None:
            return None
        start = snapped

    start_node = Node(start)
    open_list  = []
    closed_set = set()

    # FIX: dizionario best_g per controllo duplicati in O(1)
    # Prima era "any(open_node for open_node in open_list ...)" → O(n) per ogni nodo
    best_g = {start: 0.0}

    heapq.heappush(open_list, start_node)
    directions = [(-1,0),(1,0),(0,-1),(0,1),(-1,-1),(-1,1),(1,-1),(1,1)]

    while open_list:
        current_node = heapq.heappop(open_list)

        if current_node.position == goal:
            path = []
            while current_node:
                path.append(current_node.position)
                current_node = current_node.parent
            return path[::-1]

        if current_node.position in closed_set:
            continue
        closed_set.add(current_node.position)

        for dir in directions:
            next_pos = (
                current_node.position[0] + dir[0],
                current_node.position[1] + dir[1]
            )
            nr, nc = next_pos

            if not (0 <= nr < GRID_SIZE and 0 <= nc < GRID_SIZE):
                continue
            if next_pos in closed_set:
                continue

            dist_to_wall = dist_map[nr][nc]
            if dist_to_wall <= ROBOT_RADIUS_CELLS:
                continue

            step_cost = 1.414 if dir[0] != 0 and dir[1] != 0 else 1.0

            penalty = 0.0
            if dist_to_wall < SAFETY_RADIUS_CELLS:
                penalty = (SAFETY_RADIUS_CELLS - dist_to_wall) * PENALTY_WEIGHT

            new_g = current_node.g + step_cost + penalty

            # FIX: controllo O(1) — aggiunge il nodo solo se è un percorso migliore
            if next_pos in best_g and best_g[next_pos] <= new_g:
                continue

            best_g[next_pos] = new_g

            neighbor   = Node(next_pos, current_node)
            neighbor.g = new_g
            neighbor.h = heuristic(next_pos, goal)
            neighbor.f = neighbor.g + neighbor.h
            heapq.heappush(open_list, neighbor)

    return None

def visualize_path(grid, path):
    path_set = set(path) if path else set()
    for r in range(GRID_SIZE):
        line = ""
        for c in range(GRID_SIZE):
            if (r, c) in path_set:
                line += "*"
            elif grid[r][c] == 1:
                line += "#"
            else:
                line += "."
        print(line)

def expected_distance(grid, x, y, theta, max_range=0.25, step=0.005):
    """
    Restituisce la distanza prevista dal muro lungo la direzione theta.
    """
    d = 0.0
    while d <= max_range:
        px = x + d * math.cos(theta)
        py = y + d * math.sin(theta)
        row, col = world_to_grid(px, py)
        if row < 0 or row >= GRID_SIZE:
            return d
        if col < 0 or col >= GRID_SIZE:
            return d
        if grid[row][col] == 1:
            return d
        d += step
    return max_range