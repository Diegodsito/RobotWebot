"""
control.py - Controllo locale: Pure Pursuit + override reattivo di sicurezza.

Pure Pursuit: insegue un punto "carota" a distanza di lookahead lungo il
percorso. Bersaglio continuo -> nessuna oscillazione sui waypoint.

Novita' rispetto alla versione precedente:
  - lookahead piu' corto (0.06): meno "taglio di curva" vicino ai muri obliqui.
  - OVERRIDE reattivo: se un sensore frontale rileva il muro da vicino, il
    robot INDIETREGGIA e ruota verso il lato piu' libero invece di spingere.
    Questo evita la collisione E lo slittamento delle ruote che corrompe
    l'odometria (le ruote che girano contro il muro fanno "scappare" la stima).
"""

import math


class PurePursuitController:
    def __init__(self, wheel_radius, axle_length,
                 max_wheel_speed=6.28,
                 lookahead=0.06, v_max=0.08, v_min=0.012,
                 goal_tolerance=0.05, w_max=3.0,
                 big_angle_deg=55.0,
                 reverse_speed=0.02):
        self.r = wheel_radius
        self.L = axle_length
        self.max_w = max_wheel_speed
        self.Ld = lookahead
        self.v_max = v_max
        self.v_min = v_min
        self.goal_tol = goal_tolerance
        self.w_max = w_max
        self.big_angle = math.radians(big_angle_deg)
        self.reverse_speed = reverse_speed

    def _closest_index(self, path, x, y):
        best_i, best_d = 0, float('inf')
        for i, (px, py) in enumerate(path):
            d = (px - x) ** 2 + (py - y) ** 2
            if d < best_d:
                best_d, best_i = d, i
        return best_i

    def _lookahead_point(self, path, x, y, start_i):
        for i in range(start_i, len(path)):
            px, py = path[i]
            if math.hypot(px - x, py - y) >= self.Ld:
                return path[i]
        return path[-1]

    def _to_wheels(self, v, w):
        left = (v - w * self.L / 2.0) / self.r
        right = (v + w * self.L / 2.0) / self.r
        left = max(-self.max_w, min(self.max_w, left))
        right = max(-self.max_w, min(self.max_w, right))
        return left, right, False

    def compute(self, path, pose, reactive_bias=0.0, front_blocked=False):
        if not path:
            return 0.0, 0.0, True

        x, y, theta = pose
        gx, gy = path[-1]
        dist_goal = math.hypot(gx - x, gy - y)
        if dist_goal < self.goal_tol:
            return 0.0, 0.0, True

        # --- OVERRIDE di sicurezza: muro frontale ravvicinato ---
        if front_blocked:
            # gira verso il lato piu' libero; se e' un urto frontale pieno
            # (reactive_bias ~ 0) gira a sinistra di default.
            if abs(reactive_bias) < 0.05:
                turn = 1.0
            else:
                turn = 1.0 if reactive_bias > 0 else -1.0
            # indietreggia leggermente mentre ruota, per staccarsi dal muro
            return self._to_wheels(-self.reverse_speed, turn * self.w_max)

        # --- Pure Pursuit normale ---
        ci = self._closest_index(path, x, y)
        lx, ly = self._lookahead_point(path, x, y, ci)

        alpha = math.atan2(ly - y, lx - x) - theta
        alpha = math.atan2(math.sin(alpha), math.cos(alpha))

        if abs(alpha) > self.big_angle:
            # forte disallineamento: ruota quasi sul posto
            v = self.v_min
            w = max(-self.w_max, min(self.w_max, 2.5 * alpha))
        else:
            angle_factor = 1.0 / (1.0 + 2.5 * abs(alpha))
            v = max(self.v_min, self.v_max * angle_factor)
            v = min(v, max(self.v_min, dist_goal))                 # rallenta al goal
            v = max(self.v_min, v / (1.0 + 1.5 * abs(reactive_bias)))  # rallenta vicino ai muri
            kappa = 2.0 * math.sin(alpha) / max(self.Ld, 1e-3)
            w = v * kappa + 0.6 * reactive_bias                    # sterzata + bias di sicurezza
            w = max(-self.w_max, min(self.w_max, w))

        return self._to_wheels(v, w)
