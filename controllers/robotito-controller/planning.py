"""
planning.py - Pianificazione globale: A* su grid + smoothing del percorso.

Riusa path_planning.py (mappa, A*, distance map) e aggiunge:
  - smoothing "line-of-sight" che PRESERVA UN MARGINE DI SICUREZZA: una
    scorciatoia diritta viene accettata solo se ogni cella che attraversa e'
    lontana dai muri almeno 'smooth_clearance' celle. Vicino ai muri (es. il
    muro obliquo) il percorso NON viene raddrizzato, quindi resta lontano
    dagli ostacoli; nelle zone aperte viene semplificato.
  - ricampionamento: punti equidistanti per un Pure Pursuit fluido.
"""

import math
import path_planning as pp


class Planner:
    def __init__(self, scenario_name, smooth_clearance=None):
        self.grid = pp.create_map_from_wbt(scenario_name)
        self.dist_map = pp.compute_distance_map(self.grid)
        self.radius = pp.ROBOT_RADIUS_CELLS
        # margine usato per lo smoothing: piu' grande del raggio fisico, cosi'
        # le scorciatoie non passano rasenti ai muri. Default = a meta' strada
        # tra il raggio fisico (4) e il raggio di sicurezza (8) -> 6 celle (15 cm).
        if smooth_clearance is None:
            smooth_clearance = (pp.ROBOT_RADIUS_CELLS + pp.SAFETY_RADIUS_CELLS) // 2
        self.smooth_clearance = smooth_clearance

    # ---- linea di vista tra due celle (Bresenham) con margine ----
    def _line_of_sight(self, a, b):
        (r0, c0), (r1, c1) = a, b
        dr, dc = abs(r1 - r0), abs(c1 - c0)
        sr = 1 if r1 > r0 else -1
        sc = 1 if c1 > c0 else -1
        err = dr - dc
        r, c = r0, c0
        while True:
            # la scorciatoia e' valida solo se restiamo lontani dai muri
            if self.dist_map[r][c] <= self.smooth_clearance:
                return False
            if (r, c) == (r1, c1):
                return True
            e2 = 2 * err
            if e2 > -dc:
                err -= dc
                r += sr
            if e2 < dr:
                err += dr
                c += sc

    # ---- riduce il path ai soli vertici (corner) ----
    def _smooth(self, cell_path):
        if len(cell_path) < 3:
            return cell_path
        out = [cell_path[0]]
        i, n = 0, len(cell_path)
        while i < n - 1:
            j = n - 1
            while j > i + 1 and not self._line_of_sight(cell_path[i], cell_path[j]):
                j -= 1
            out.append(cell_path[j])
            i = j
        return out

    # ---- rimette punti equidistanti lungo la polilinea ----
    def _resample(self, pts, spacing=0.03):
        if len(pts) < 2:
            return list(pts)
        dense = [pts[0]]
        for i in range(len(pts) - 1):
            (x0, y0), (x1, y1) = pts[i], pts[i + 1]
            seg = math.hypot(x1 - x0, y1 - y0)
            steps = max(1, int(seg / spacing))
            for k in range(1, steps + 1):
                t = k / steps
                dense.append((x0 + t * (x1 - x0), y0 + t * (y1 - y0)))
        return dense

    def plan(self, start_world, goal_world, spacing=0.03):
        """Lista di waypoint (x, y) smussati e ricampionati. Vuota se nessun path."""
        start = pp.world_to_grid(*start_world)
        goal = pp.world_to_grid(*goal_world)
        cell_path = pp.a_star(self.grid, start, goal)
        if not cell_path:
            return []
        cell_path = self._smooth(cell_path)
        world_pts = [pp.grid_to_world(r, c) for (r, c) in cell_path]
        return self._resample(world_pts, spacing=spacing)
