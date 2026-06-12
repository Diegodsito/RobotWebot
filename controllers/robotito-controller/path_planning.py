"""
path_planning.py — Módulo de planificación de rutas (A*)
Proyecto Final ICI 4150 — Robótica y Sistemas Autónomos 2026-01

Grilla: 100×100 celdas, 2.5 cm/celda → cubre arena 2.5×2.5 m.
Basado en el trabajo de María Paganetti (branch proyecto-final).
Correcciones aplicadas:
  - CELL_SIZE corregido de 0.0025 → 0.025 (la grilla ahora cubre los 2.5 m reales)
  - Obstáculos rotados 90° con dimensiones corregidas (w/h intercambiados)
  - Obstáculos rotados 45° aproximados con bounding-box cuadrada inflada
  - Añadido mapa del escenario simple
"""
import math
import heapq

GRID_SIZE    = 100
CELL_SIZE    = 0.025       # 2.5 cm por celda  [CORRECCIÓN: era 0.0025]
ARENA_OFFSET = 1.25        # mitad de la arena (2.5 m / 2)

# ─────────────────────────────────────────────
# Conversión mundo ↔ grilla
# ─────────────────────────────────────────────

def world_to_grid(x, y):
    col = int(round((x + ARENA_OFFSET) / CELL_SIZE))
    row = int(round((ARENA_OFFSET - y) / CELL_SIZE))
    return (max(0, min(GRID_SIZE - 1, row)), max(0, min(GRID_SIZE - 1, col)))

def grid_to_world(row, col):
    x = (col * CELL_SIZE) - ARENA_OFFSET
    y = ARENA_OFFSET - (row * CELL_SIZE)
    return (x, y)

# ─────────────────────────────────────────────
# Construcción del mapa
# ─────────────────────────────────────────────

def add_box_to_grid(grid, cx, cy, w, h):
    """Marca celdas ocupadas por un SolidBox eje-alineado centrado en (cx, cy)."""
    x_min, x_max = cx - w / 2, cx + w / 2
    y_min, y_max = cy - h / 2, cy + h / 2
    r_min, c_min = world_to_grid(x_min, y_max)
    r_max, c_max = world_to_grid(x_max, y_min)
    for r in range(min(r_min, r_max), max(r_min, r_max) + 1):
        for c in range(min(c_min, c_max), max(c_min, c_max) + 1):
            if 0 <= r < GRID_SIZE and 0 <= c < GRID_SIZE:
                grid[r][c] = 1

def create_map_complejo():
    """
    Mapa del escenario complejo (escenario_complejo.wbt).
    Nota sobre rotaciones:
      - rot 90° (1.5708 rad): dimensiones size(w,h) se intercambian en la grilla → swap w/h
      - rot 45° (2.3561 rad): se aproxima con bounding-box cuadrada de lado = max(w,h)*√2
    """
    grid = [[0] * GRID_SIZE for _ in range(GRID_SIZE)]

    # Todos los valores extraídos directamente del escenario_complejo.wbt.
    # Regla de rotación en grilla 2D (eje Z):
    #   rot 0        → size(w, h) se usa tal cual:  w=x, h=y
    #   rot ±90°     → dimensiones intercambiadas:  w=h_orig, h=w_orig
    #   rot ±45°     → bbox diagonal: w=h=largo*√2  (aproximación conservadora)

    # --- SPINES ---
    # spine1: size 0.6×0.1, rot -90° → en grilla w=0.1, h=0.6
    add_box_to_grid(grid, -0.79,   -0.52,  0.1,  0.6)
    # spine2: size 0.1×0.7, rot -135° (≈-2.356 rad, diagonal) → bbox 0.7*√2 ≈ 0.50
    add_box_to_grid(grid, -0.4253, -0.4253, 0.50, 0.50)
    # spine2(1): size 0.1×0.7, rot -135° → bbox 0.50×0.50
    add_box_to_grid(grid,  0.1758, -0.3899, 0.50, 0.50)
    # spine3: size 0.7×0.1, rot 0° → w=0.7, h=0.1
    add_box_to_grid(grid,  0.9,     0.17,   0.7,  0.1)
    # spine3(1): size 0.7×0.1, rot 0° → w=0.7, h=0.1
    add_box_to_grid(grid,  0.45,    0.66,   0.7,  0.1)

    # --- TRAMPAS ---
    # trap_b3(1): size 0.4×0.1, rot 0° → w=0.4, h=0.1
    add_box_to_grid(grid,  0.58,   -0.79,   0.4,  0.1)
    # trap_a1: size 0.7×0.1, rot +90° → w=0.1, h=0.7
    add_box_to_grid(grid, -0.79,    0.18,   0.1,  0.7)
    # trap_b1: size 0.1×0.5, rot -90° → w=0.5, h=0.1
    add_box_to_grid(grid, -0.48,   -0.77,   0.5,  0.1)
    # trap_b3: size 0.4×0.1, rot +90° → w=0.1, h=0.4
    add_box_to_grid(grid, -0.25,    1.05,   0.1,  0.4)
    # trap_b3(3): size 0.4×0.1, rot +90° → w=0.1, h=0.4
    add_box_to_grid(grid, -0.08,    0.09,   0.1,  0.4)
    # trap_b3(2): size 0.4×0.1, rot +90° → w=0.1, h=0.4
    add_box_to_grid(grid,  0.75,   -0.5,    0.1,  0.4)

    # --- OBSTÁCULOS pequeños (sin rotación) ---
    add_box_to_grid(grid, -0.8,     0.68,   0.1,  0.1)   # obs1
    add_box_to_grid(grid,  0.44,   -0.34,   0.1,  0.1)   # obs2
    add_box_to_grid(grid, -0.58,   -0.03,   0.1,  0.1)   # obs3

    # --- PAREDES DE LA ARENA (RectangleArena 2.5×2.5 m) ---
    # Marcar los bordes para que A* nunca planifique cerca de ellos
    add_box_to_grid(grid,  0.0,  -1.25, 2.5, 0.05)   # pared sur
    add_box_to_grid(grid,  0.0,   1.25, 2.5, 0.05)   # pared norte
    add_box_to_grid(grid, -1.25,  0.0,  0.05, 2.5)   # pared oeste
    add_box_to_grid(grid,  1.25,  0.0,  0.05, 2.5)   # pared este

    return grid

def create_map_simple():
    """
    Mapa del escenario simple (escenario_simple.wbt).
    Arena 2.5×2.5 m, robot en (-0.9, 0), meta en (0.9, 0).
    3 obstáculos SolidBox de 0.15×0.15 m.
    """
    grid = [[0] * GRID_SIZE for _ in range(GRID_SIZE)]
    add_box_to_grid(grid,  0.00,  0.00, 0.15, 0.15)   # obstaculo_central
    add_box_to_grid(grid,  0.70,  0.50, 0.15, 0.15)   # obstaculo_superior
    add_box_to_grid(grid,  0.50, -0.70, 0.15, 0.15)   # obstaculo_inferior

    # --- PAREDES DE LA ARENA ---
    add_box_to_grid(grid,  0.0,  -1.25, 2.5, 0.05)   # pared sur
    add_box_to_grid(grid,  0.0,   1.25, 2.5, 0.05)   # pared norte
    add_box_to_grid(grid, -1.25,  0.0,  0.05, 2.5)   # pared oeste
    add_box_to_grid(grid,  1.25,  0.0,  0.05, 2.5)   # pared este

    return grid

def inflate_map(grid, radius=3):
    """
    Infla obstáculos 'radius' celdas de margen.
    Con CELL_SIZE=0.025 y radius=2 → margen de 5 cm (suficiente para el e-puck de 3.7 cm de radio).
    """
    inflated = [[0] * GRID_SIZE for _ in range(GRID_SIZE)]
    for r in range(GRID_SIZE):
        for c in range(GRID_SIZE):
            if grid[r][c] == 1:
                for dr in range(-radius, radius + 1):
                    for dc in range(-radius, radius + 1):
                        nr, nc = r + dr, c + dc
                        if 0 <= nr < GRID_SIZE and 0 <= nc < GRID_SIZE:
                            inflated[nr][nc] = 1
    return inflated

# ─────────────────────────────────────────────
# Algoritmo A*  (estructura original de María)
# ─────────────────────────────────────────────

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

def a_star(grid, start, goal):
    start_node = Node(start)
    open_list  = []
    closed_set = set()
    heapq.heappush(open_list, start_node)
    directions = [(-1,0),(1,0),(0,-1),(0,1),(-1,-1),(-1,1),(1,-1),(1,1)]

    while open_list:
        current_node = heapq.heappop(open_list)
        closed_set.add(current_node.position)

        if current_node.position == goal:
            path = []
            while current_node:
                path.append(current_node.position)
                current_node = current_node.parent
            return path[::-1]

        for direction in directions:
            nr = current_node.position[0] + direction[0]
            nc = current_node.position[1] + direction[1]
            if not (0 <= nr < GRID_SIZE and 0 <= nc < GRID_SIZE): continue
            if grid[nr][nc] == 1:                                    continue
            if (nr, nc) in closed_set:                               continue

            # Evitar cortar esquinas diagonalmente
            if direction[0] != 0 and direction[1] != 0:
                r1 = current_node.position[0] + direction[0]
                c1 = current_node.position[1]
                r2 = current_node.position[0]
                c2 = current_node.position[1] + direction[1]
                obs1 = grid[r1][c1] == 1 if (0 <= r1 < GRID_SIZE and 0 <= c1 < GRID_SIZE) else True
                obs2 = grid[r2][c2] == 1 if (0 <= r2 < GRID_SIZE and 0 <= c2 < GRID_SIZE) else True
                if obs1 and obs2: continue

            move_cost = 1.0 if (direction[0] == 0 or direction[1] == 0) else 1.414
            neighbor_node = Node((nr, nc), current_node)
            neighbor_node.g = current_node.g + move_cost
            neighbor_node.h = heuristic((nr, nc), goal)
            neighbor_node.f = neighbor_node.g + neighbor_node.h

            existing = next((n for n in open_list if n.position == neighbor_node.position), None)
            if existing and neighbor_node.g >= existing.g: continue
            heapq.heappush(open_list, neighbor_node)

    return None

def visualize_path(grid, path):
    """Imprime la grilla en consola con la ruta marcada (útil para debug)."""
    path_set = set(path) if path else set()
    for r in range(GRID_SIZE):
        line = ""
        for c in range(GRID_SIZE):
            if   (r, c) in path_set: line += "*"
            elif grid[r][c] == 1:    line += "#"
            else:                    line += "."
        print(line)

# ─────────────────────────────────────────────
# Prueba standalone
# ─────────────────────────────────────────────

if __name__ == '__main__':
    import sys
    esc = sys.argv[1] if len(sys.argv) > 1 else "complejo"

    if esc == "simple":
        base_map = create_map_simple()
        start    = world_to_grid(-0.9,  0.0)
        goal     = world_to_grid( 0.9,  0.0)
    else:
        base_map = create_map_complejo()
        start    = world_to_grid(-1.1, -1.1)
        goal     = world_to_grid( 1.1,  1.1)

    safe_map = inflate_map(base_map)
    path     = a_star(safe_map, start, goal)

    if path:
        print(f"[OK] Ruta encontrada: {len(path)} nodos.")
        visualize_path(safe_map, path)
    else:
        print("[ERROR] No se encontró ruta. Revisa el mapa.")