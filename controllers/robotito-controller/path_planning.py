"""
Modulo di Path Planning (A*) - Risoluzione Alta (5cm)
"""
import math
import heapq

GRID_SIZE = 100
CELL_SIZE = 0.0025
ARENA_OFFSET = 1.25  

def world_to_grid(x, y):
    col = int(round((x + ARENA_OFFSET) / CELL_SIZE))
    row = int(round((ARENA_OFFSET - y) / CELL_SIZE))
    return (max(0, min(GRID_SIZE - 1, row)), max(0, min(GRID_SIZE - 1, col)))

def grid_to_world(row, col):
    x = (col * CELL_SIZE) - ARENA_OFFSET
    y = ARENA_OFFSET - (row * CELL_SIZE)
    return (x, y)

def visualize_path(grid, path):
    """Stampa la mappa nel terminale con il percorso segnato con asterischi."""
    for r in range(len(grid)):
        line = ""
        for c in range(len(grid[0])):
            if (r, c) in path:
                line += " * " # Il percorso
            elif grid[r][c] == 1:
                line += " # " # Muro
            else:
                line += " . " # Spazio libero
        print(line)

def add_box_to_grid(grid, cx, cy, w, h):
    x_min, x_max = cx - w/2, cx + w/2
    y_min, y_max = cy - h/2, cy + h/2
    r_min, c_min = world_to_grid(x_min, y_max) 
    r_max, c_max = world_to_grid(x_max, y_min) 
    for r in range(min(r_min, r_max), max(r_min, r_max) + 1):
        for c in range(min(c_min, c_max), max(c_min, c_max) + 1):
            if 0 <= r < GRID_SIZE and 0 <= c < GRID_SIZE:
                grid[r][c] = 1

def create_map():
    grid = [[0 for _ in range(GRID_SIZE)] for _ in range(GRID_SIZE)]
    
    # --- SPINE ---
    # spine1 (Ruotato: swap dimensioni)
    add_box_to_grid(grid, -0.79, -0.52, 0.1, 0.6)
    # spine2 (Diagonale: manterrà l'ingombro rettangolare)
    add_box_to_grid(grid, -0.425, -0.425, 0.1, 0.7)
    # spine2(1)
    add_box_to_grid(grid, 0.175, -0.389, 0.1, 0.7)
    # spine3
    add_box_to_grid(grid, 0.9, 0.17, 0.7, 0.1)
    # spine3(1)
    add_box_to_grid(grid, 0.45, 0.66, 0.7, 0.1)

    # --- TRAPPOLE ---
    # trap_b3(1)
    add_box_to_grid(grid, 0.58, -0.79, 0.4, 0.1)
    # trap_a1 (Ruotato: swap dimensioni)
    add_box_to_grid(grid, -0.79, 0.18, 0.1, 0.7)
    # trap_b1 (Ruotato: swap dimensioni)
    add_box_to_grid(grid, -0.48, -0.77, 0.5, 0.1)
    # trap_b3 (Ruotato: swap dimensioni)
    add_box_to_grid(grid, -0.25, 1.05, 0.1, 0.4)
    # trap_b3(3) (Ruotato: swap dimensioni)
    add_box_to_grid(grid, -0.08, 0.09, 0.1, 0.4)
    # trap_b3(2) (Ruotato: swap dimensioni)
    add_box_to_grid(grid, 0.75, -0.5, 0.1, 0.4)

    # --- OSTACOLI ---
    add_box_to_grid(grid, -0.8, 0.68, 0.1, 0.1)   # obs1
    add_box_to_grid(grid, 0.44, -0.34, 0.1, 0.1)  # obs2
    add_box_to_grid(grid, -0.58, -0.03, 0.1, 0.1) # obs3

    return grid

def inflate_map(grid):
    inflated = [[0 for _ in range(GRID_SIZE)] for _ in range(GRID_SIZE)]
    INFLATION_RADIUS = 1
    for r in range(GRID_SIZE):
        for c in range(GRID_SIZE):
            if grid[r][c] == 1:
                for dr in range(-INFLATION_RADIUS, INFLATION_RADIUS + 1):
                    for dc in range(-INFLATION_RADIUS, INFLATION_RADIUS + 1):
                        nr, nc = r + dr, c + dc
                        if 0 <= nr < GRID_SIZE and 0 <= nc < GRID_SIZE:
                            inflated[nr][nc] = 1
    return inflated

class Node:
    def __init__(self, position, parent=None):
        self.position = position
        self.parent = parent
        self.g = 0
        self.h = 0
        self.f = 0
    def __lt__(self, other):
        return self.f < other.f

def heuristic(current, goal):
    return math.sqrt((current[0] - goal[0])**2 + (current[1] - goal[1])**2)

def a_star(grid, start, goal):
    start_node = Node(start)
    goal_node = Node(goal)
    open_list = []
    closed_set = set()
    heapq.heappush(open_list, start_node)
    directions = [(-1,0), (1,0), (0,-1), (0,1), (-1,-1), (-1,1), (1,-1), (1,1)]
    
    while open_list:
        current_node = heapq.heappop(open_list)
        closed_set.add(current_node.position)
        
        if current_node.position == goal_node.position:
            path = []
            while current_node:
                path.append(current_node.position)
                current_node = current_node.parent
            return path[::-1]
            
        for direction in directions:
            nr = current_node.position[0] + direction[0]
            nc = current_node.position[1] + direction[1]
            if not (0 <= nr < GRID_SIZE and 0 <= nc < GRID_SIZE): continue
            if grid[nr][nc] == 1: continue
            if (nr, nc) in closed_set: continue
                
            # Controllo spigoli allentato per garantire la rotta
            if direction[0] != 0 and direction[1] != 0:
                r1, c1 = current_node.position[0] + direction[0], current_node.position[1]
                r2, c2 = current_node.position[0], current_node.position[1] + direction[1]
                obs1 = grid[r1][c1] == 1 if (0 <= r1 < GRID_SIZE and 0 <= c1 < GRID_SIZE) else True
                obs2 = grid[r2][c2] == 1 if (0 <= r2 < GRID_SIZE and 0 <= c2 < GRID_SIZE) else True
                if obs1 and obs2: continue
                    
            move_cost = 1 if direction[0] == 0 or direction[1] == 0 else 1.414
            neighbor_node = Node((nr, nc), current_node)
            neighbor_node.g = current_node.g + move_cost
            neighbor_node.h = heuristic((nr, nc), goal_node.position)
            neighbor_node.f = neighbor_node.g + neighbor_node.h
            
            existing_node = next((n for n in open_list if n.position == neighbor_node.position), None)
            if existing_node and neighbor_node.g >= existing_node.g: continue
            heapq.heappush(open_list, neighbor_node)
            
    return None

if __name__ == '__main__':
    base_map = create_map()
    safe_map = inflate_map(base_map)
    start_grid = world_to_grid(-1.1, -1.1)
    goal_grid = world_to_grid(1.1, 1.1)
    path = a_star(safe_map, start_grid, goal_grid)
    if path: print(f"[Sistema di Navigazione] Percorso calcolato: {len(path)} nodi.")
    else: print("[Errore] Fallimento pianificazione rotta.")
    path = a_star(safe_map, start_grid, goal_grid)
    if path:
        visualize_path(safe_map, path)