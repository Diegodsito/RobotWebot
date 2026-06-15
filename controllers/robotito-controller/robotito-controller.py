from controller import Robot
import math
import csv
import os
import path_planning

# ==============================================================================
# ODOMETRIA
# ==============================================================================
def update_odometry(x, y, theta, dl, dr, wheel_radius, axle_length):
    v = (dr + dl) / 2.0
    dtheta = (dr - dl) / axle_length
    theta_mid = theta + dtheta / 2.0
    x += v * math.cos(theta_mid)
    y += v * math.sin(theta_mid)
    theta += dtheta
    theta = math.atan2(math.sin(theta), math.cos(theta))
    return x, y, theta

# ==============================================================================
# STIMA POSA DA IR
# FIX BUG 1: rimosso il primo loop vuoto e spostato l'inizializzazione di
#            valid_measures DENTRO la funzione (non sovrascriveva tra i call).
#            Usato SENSOR_ANGLES.items() direttamente, rimossa sensor_map ridondante.
# ==============================================================================
def estimate_pose_from_ir(mapa, x_est, y_est, theta_est):
    valid_measures = []   # ← inizializzata UNA VOLTA per chiamata, non per step

    for idx, ang in SENSOR_ANGLES.items():
        measured = ir_to_distance(ps[idx].getValue())
        if measured is None:
            # Sotto il noise floor: nessun ostacolo rilevato.
            # Non possiamo confrontare con expected_distance → saltiamo.
            continue
        expected = path_planning.expected_distance(mapa, x_est, y_est, theta_est + ang)
        err = measured - expected
        if abs(err) < SENSOR_GATE:
            valid_measures.append((ang, err))

    if not valid_measures:
        return None

    dx, dy, dth = 0.0, 0.0, 0.0
    for ang, err in valid_measures:
        dx  += -err * math.cos(theta_est + ang)
        dy  += -err * math.sin(theta_est + ang)
        dth +=  err * 0.2

    n = len(valid_measures)
    return [x_est + dx / n, y_est + dy / n, theta_est + dth / n]

# ==============================================================================
# OBSTACLE AVOIDANCE
# ==============================================================================
def _ir_dist_safe(sensor):
    """Wrapper: None (no obstacle) → 0.20 m sentinel per obstacle_avoidance."""
    d = ir_to_distance(sensor.getValue())
    return d if d is not None else 0.20

def obstacle_avoidance(v_cmd, w_cmd):
    d_front = min(_ir_dist_safe(ps[0]), _ir_dist_safe(ps[7]))
    d_left  = min(_ir_dist_safe(ps[1]), _ir_dist_safe(ps[2]))
    d_right = min(_ir_dist_safe(ps[6]), _ir_dist_safe(ps[5]))

    SAFE = 0.10
    if d_front < SAFE:
        error   = SAFE - d_front
        v_cmd  *= max(0.2, d_front / SAFE)
        w_cmd  += 12.0 * error * (d_left - d_right)

    return v_cmd, w_cmd

# ==============================================================================
# KALMAN FILTER (semplificato, diagonale)
# ==============================================================================
def kalman_predict(x, p, q):
    for i in range(3):
        p[i][i] += q[i][i]
    return x, p

def kalman_update(x, p, z, r):
    for i in range(3):
        k    = p[i][i] / (p[i][i] + r[i][i])
        x[i] = x[i] + k * (z[i] - x[i])
        p[i][i] = (1 - k) * p[i][i]
    return x, p

# ==============================================================================
# CONVERSIONE IR → DISTANZA  (e-puck Webots, sensori ps0-ps7)
#
# Caratteristica reale del sensore in Webots:
#   value ~  0-100  → nessun ostacolo (rumore di fondo)
#   value ~  300    → ostacolo a ~20 cm
#   value ~ 2000    → ostacolo a ~5 cm
#   value ~ 4000+   → ostacolo vicinissimo (<2 cm)
#
# PROBLEMA PRECEDENTE: la formula esponenziale con value=65 dava 0.133 m,
# come se ci fosse un muro a 13 cm in ogni direzione → Kalman corrompeva
# la stima di posa anche in campo aperto.
#
# FIX: soglia IR_NOISE_FLOOR=150: sotto questa soglia il sensore non ha
# rilevato nulla → restituiamo None per saltare la lettura.
# Formula calibrata: dist = 600/(value+300), punti di calibrazione:
#   (150→0.20m), (300→0.12m), (800→0.07m), (2000→0.03m), (4000→0.01m)
# ==============================================================================
IR_NOISE_FLOOR = 150  # raw sotto questa soglia = nessun ostacolo affidabile

def ir_to_distance(value):
    """
    Converte valore raw IR e-puck in distanza [m].
    Restituisce None se sotto il noise floor (campo libero).
    """
    if value < IR_NOISE_FLOOR:
        return None
    value = min(value, 4000.0)
    dist = 600.0 / (value + 300.0)
    return max(0.01, min(0.20, dist))

# ==============================================================================
# CONFIGURAZIONE SCENARIO
# ==============================================================================
ESCENARIO = "complejo"

CONFIG = {
    "simple": {
        "start": (-0.9, 0.0),
        "goal":  ( 0.9, 0.0),
    },
    "complejo": {
        "start": (-1.0, -1.0),
        "goal":  ( 1.0,  1.0),
    },
}

# ==============================================================================
# INIT ROBOT
# ==============================================================================
robotito = Robot()
timestep = int(robotito.getBasicTimeStep())

motor_izq = robotito.getDevice("left wheel motor")
motor_der = robotito.getDevice("right wheel motor")
motor_izq.setPosition(float('inf'))
motor_der.setPosition(float('inf'))
motor_izq.setVelocity(0.0)
motor_der.setVelocity(0.0)

# ── Sensori IR ─────────────────────────────────────────────────────────────────
ps_names = ["ps0", "ps1", "ps2", "ps3", "ps4", "ps5", "ps6", "ps7"]
SENSOR_ANGLES = {
    0: math.radians( 17),
    1: math.radians( 45),
    2: math.radians( 90),
    3: math.radians(150),
    4: math.radians(-150),
    5: math.radians(-90),
    6: math.radians(-45),
    7: math.radians(-17),
}
ps = [robotito.getDevice(name) for name in ps_names]
for sensor in ps:
    sensor.enable(timestep)

# ── Encoder ────────────────────────────────────────────────────────────────────
left_encoder  = robotito.getDevice('left wheel sensor')
right_encoder = robotito.getDevice('right wheel sensor')
left_encoder.enable(timestep)
right_encoder.enable(timestep)

# ==============================================================================
# COSTANTI
# ==============================================================================
WHEEL_RADIUS       = 0.0205
AXLE_LENGTH        = 0.052
MAX_SPEED          = 6.28
# 0.04 m: abbastanza larga da compensare il drift odometrico,
# abbastanza stretta da non bruciare WP a distanza 8 cm senza muoversi.
WAYPOINT_TOLERANCE = 0.04

K_W   = 2.0
V_MAX = 0.12
MAX_W = 2.5

SENSOR_GATE = 0.05



# LOOK-AHEAD: invece di puntare al prossimo WP, il robot punta al primo WP
# che si trova a più di LOOKAHEAD_DIST dalla sua posizione corrente.
# Questo rende il path-following molto più robusto al drift odometrico:
# piccoli errori di posizione non causano angoli di sterzata estremi.
LOOKAHEAD_DIST = 0.15   # [m] raggio look-ahead

DEBUG_EVERY = 20

# ==============================================================================
# INIT MAPPA & PERCORSO
# ==============================================================================
print(f"[Init] Costruzione mappa dal file .wbt — scenario: {ESCENARIO}")
mapa = path_planning.create_map_from_wbt(ESCENARIO)

start_cfg = CONFIG[ESCENARIO]["start"]
goal_cfg  = CONFIG[ESCENARIO]["goal"]

# ── Kalman state ───────────────────────────────────────────────────────────────
X = [start_cfg[0], start_cfg[1], 0.0]
P = [[0.1, 0.0, 0.0], [0.0, 0.1, 0.0], [0.0, 0.0, 0.1]]
Q = [[0.001, 0.0, 0.0], [0.0, 0.001, 0.0], [0.0, 0.0, 0.001]]
R = [[0.05,  0.0, 0.0], [0.0, 0.05,  0.0], [0.0, 0.0,  0.02]]

# ── Pianificazione iniziale ────────────────────────────────────────────────────
ruta_ini = path_planning.a_star(
    mapa,
    path_planning.world_to_grid(*start_cfg),
    path_planning.world_to_grid(*goal_cfg)
)

if ruta_ini:
    waypoints_bruti = [path_planning.grid_to_world(p[0], p[1]) for p in ruta_ini]
    waypoints_bruti.pop(0)  # rimuove il punto di start

    # Filtra waypoint troppo vicini (< 8 cm)
    filtered = [waypoints_bruti[0]]
    last = waypoints_bruti[0]
    for p in waypoints_bruti[1:]:
        if math.hypot(p[0] - last[0], p[1] - last[1]) > 0.15:
            filtered.append(p)
            last = p
    waypoints = filtered
    print(f"[Init] Percorso filtrato: {len(waypoints)} waypoint.")
else:
    waypoints = []
    print("[ERROR] Nessun percorso trovato! Controlla la mappa e i punti start/goal.")

print("\n[INFO] --- MAPPA ASCII ---")
path_planning.visualize_path(mapa, ruta_ini)
print("[INFO] -----------------\n")

# ==============================================================================
# STATO ODOMETRICO & CSV
# ==============================================================================
robotito.step(timestep)
prev_left  = left_encoder.getValue()
prev_right = right_encoder.getValue()

os.makedirs('../../docs', exist_ok=True)
csv_path   = f'../../docs/datos_{ESCENARIO}.csv'
csv_file   = open(csv_path, 'w', newline='')
csv_writer = csv.writer(csv_file)
csv_writer.writerow(['tiempo_s', 'x_est', 'y_est', 'theta_deg', 'wp_restantes',
                     'v_cmd', 'w_cmd', 'angle_error_deg', 'kalman_uncertainty'])

# ==============================================================================
# LOOP PRINCIPALE
# ==============================================================================
step_count = 0
# Z inizializzata a None prima del loop per evitare NameError al primo step
Z = None

while robotito.step(timestep) != -1:
    step_count += 1

    # ── 1. ODOMETRIA ──────────────────────────────────────────────────────────
    cur_left  = left_encoder.getValue()
    cur_right = right_encoder.getValue()

    dl = (cur_left  - prev_left)  * WHEEL_RADIUS
    dr = (cur_right - prev_right) * WHEEL_RADIUS

    prev_left  = cur_left
    prev_right = cur_right

    X[0], X[1], X[2] = update_odometry(X[0], X[1], X[2], dl, dr, WHEEL_RADIUS, AXLE_LENGTH)

    # ── 2. KALMAN PREDICT ─────────────────────────────────────────────────────
    X, P = kalman_predict(X, P, Q)

    # ── 2b. CORREZIONE IR (ogni 5 step) ───────────────────────────────────────
    if step_count % 5 == 0:
        Z = estimate_pose_from_ir(mapa, X[0], X[1], X[2])

    # FIX BUG 2: Z è sempre definita (None o lista), nessun NameError possibile
    if Z is not None:
        X, P = kalman_update(X, P, Z, R)
        Z = None  # consuma la misura, non riapplicare al prossimo step

    robot_x     = X[0]
    robot_y     = X[1]
    robot_theta = X[2]

    # ── 3. REPLAN DISABILITATO ───────────────────────────────────────────────
    # Il replan dai log peggiorava sempre la situazione: partiva da una
    # posizione già deviata per drift, generava centinaia di WP, e i
    # "skip WP superato" in cascata mandavano il robot in loop fuori traiettoria.
    # Il look-ahead (15 cm) è sufficiente per assorbire il drift normale.

    # ── 4. INSEGUIMENTO WAYPOINT ──────────────────────────────────────────────
    v_cmd       = 0.0
    w_cmd       = 0.0
    angle_error = 0.0

    # ── Consuma i WP già passati (dentro WAYPOINT_TOLERANCE) ─────────────────
    while waypoints:
        tx, ty = waypoints[0]
        if math.hypot(tx - robot_x, ty - robot_y) < WAYPOINT_TOLERANCE:
            waypoints.pop(0)
            print(f"[WP] Step {step_count}: waypoint raggiunto. Rimangono {len(waypoints)}.")
        else:
            break

    if not waypoints:
        motor_izq.setVelocity(0.0)
        motor_der.setVelocity(0.0)
        print("[INFO] Meta raggiunta con successo!")
        break

    # ── Look-ahead: punta al primo WP oltre LOOKAHEAD_DIST ────────────────────
    # Scorre la lista e sceglie il primo waypoint sufficientemente lontano.
    # Se tutti i WP rimasti sono entro LOOKAHEAD_DIST, usa l'ultimo (goal vicino).
    target_x, target_y = waypoints[-1]
    for wx, wy in waypoints:
        if math.hypot(wx - robot_x, wy - robot_y) >= LOOKAHEAD_DIST:
            target_x, target_y = wx, wy
            break

    dx   = target_x - robot_x
    dy   = target_y - robot_y
    dist = math.hypot(dx, dy)

    target_angle = math.atan2(dy, dx)
    angle_error  = (target_angle - robot_theta + math.pi) % (2 * math.pi) - math.pi

    w_cmd        = max(-MAX_W, min(MAX_W, K_W * angle_error))
    angle_factor = max(0.0, 1.0 - abs(angle_error) / math.radians(60))

    # Velocità lineare: proporzionale all'allineamento, mai ferma del tutto.
    # Usa dist al look-ahead target per l'ultimo tratto.
    if len(waypoints) > 1:
        v_cmd = V_MAX * angle_factor
    else:
        v_cmd = min(V_MAX * angle_factor, 1.5 * dist)

    v_cmd = max(0.02, v_cmd)
    v_cmd, w_cmd = obstacle_avoidance(v_cmd, w_cmd)

    left_v  = max(-MAX_SPEED, min(MAX_SPEED, (v_cmd - w_cmd * AXLE_LENGTH / 2.0) / WHEEL_RADIUS))
    right_v = max(-MAX_SPEED, min(MAX_SPEED, (v_cmd + w_cmd * AXLE_LENGTH / 2.0) / WHEEL_RADIUS))

    motor_izq.setVelocity(left_v)
    motor_der.setVelocity(right_v)

    # ── 5. DEBUG PRINT ────────────────────────────────────────────────────────
    # FIX BUG 4: rimossa la stringa f con `if waypoints else ''` letterale che
    #            crashava quando waypoints era vuoto.
    if step_count % DEBUG_EVERY == 0:
        wp_str   = f"({waypoints[0][0]:.3f}, {waypoints[0][1]:.3f})" if waypoints else "—"
        dist_str = f"{math.hypot(waypoints[0][0]-robot_x, waypoints[0][1]-robot_y):.3f}m" if waypoints else "—"
        print(
            f"[DBG t={robotito.getTime():.2f}s] "
            f"pos=({robot_x:.3f}, {robot_y:.3f}) θ={math.degrees(robot_theta):.1f}° | "
            f"target={wp_str} dist={dist_str} | "
            f"angle_err={math.degrees(angle_error):.1f}° | "
            f"v={v_cmd:.3f} w={w_cmd:.3f} | wp_rimasti={len(waypoints)}"
        )
        raw   = [ps[i].getValue() for i in range(4)]
        dists = [f'{ir_to_distance(v):.3f}' if ir_to_distance(v) is not None else 'free' for v in raw]
        print(f"         IR_raw={[f'{v:.0f}' for v in raw]} → dist_m={dists}")

    # ── 6. CSV ────────────────────────────────────────────────────────────────
    kalman_uncertainty = P[0][0] + P[1][1]
    csv_writer.writerow([
        robotito.getTime(),
        robot_x,
        robot_y,
        math.degrees(robot_theta),
        len(waypoints),
        v_cmd,
        w_cmd,
        math.degrees(angle_error),
        kalman_uncertainty,
    ])

csv_file.close()
print(f"[INFO] CSV salvato in {csv_path}")