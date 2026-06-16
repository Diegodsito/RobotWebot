import sys
import os
import math
import csv
from controller import Robot

# Forza Python a leggere path_planning.py da QUESTA cartella locale
current_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, current_dir)

import path_planning

# =============================================================================
# CONTROLADOR E-PUCK V2 (PURA ODOMETRIA E PATH TRACKING)
# =============================================================================

ESCENARIO = "complejo"
START = (-1.0, -1.0)
GOAL = (1.0, 1.0)

# Costanti fisiche dell'e-puck (TARATE PER ODOMETRIA)
WHEEL_RADIUS = 0.0205
AXLE_LENGTH = 0.057  # Aumentato da 0.052 per compensare lo strisciamento laterale
MAX_SPEED = 6.28
ROBOT_RADIUS = 0.037

# Parametri di controllo (ADDOLCITI PER EVITARE SLITTAMENTO)
V_MAX = 0.105
V_MIN = 0.015
K_W = 2.0            # Ridotto (era 3.5) per curve più dolci
MAX_W = 0.9         # Ridotto (era 4.0) per non far slittare le ruote
LOOKAHEAD_DIST = 0.08 
WAYPOINT_REACHED = 0.07
GOAL_TOL = 0.08
TURN_IN_PLACE_ANGLE = math.radians(45)

DEBUG_EVERY = 25

# =============================================================================
# FUNZIONI DI SUPPORTO
# =============================================================================

def clamp(value, lo, hi):
    return max(lo, min(hi, value))

def norm_angle(a):
    return math.atan2(math.sin(a), math.cos(a))

def make_motor_speeds(v, w):
    left = (v - w * AXLE_LENGTH / 2.0) / WHEEL_RADIUS
    right = (v + w * AXLE_LENGTH / 2.0) / WHEEL_RADIUS
    return clamp(left, -MAX_SPEED, MAX_SPEED), clamp(right, -MAX_SPEED, MAX_SPEED)

def update_odometry(x, y, theta, dl, dr):
    """
    Aggiorna la posa del robot basandosi unicamente sullo spostamento delle ruote.
    Utilizza un'approssimazione di Runge-Kutta del 2° ordine (midpoint).
    """
    v_dt = (dr + dl) / 2.0
    w_dt = (dr - dl) / AXLE_LENGTH
    
    x_new = x + v_dt * math.cos(theta + w_dt / 2.0)
    y_new = y + v_dt * math.sin(theta + w_dt / 2.0)
    theta_new = norm_angle(theta + w_dt)
    
    return x_new, y_new, theta_new

def plan_route(grid, start_pos, goal_pos):
    start_grid = path_planning.world_to_grid(*start_pos)
    goal_grid = path_planning.world_to_grid(*goal_pos)
    route = path_planning.a_star(grid, start_grid, goal_grid)
    
    if not route: return []
    
    print("\n--- VISUALIZZAZIONE MAPPA E PERCORSO A* ---")
    path_planning.visualize_path(grid, route)
    print("-------------------------------------------\n")
    
    world_points = [path_planning.grid_to_world(r, c) for r, c in route]
    wpts = path_planning.downsample_world_path(world_points, spacing=0.10)
    
    # Rimuovi il primo waypoint se siamo già abbastanza vicini
    if wpts and math.hypot(wpts[0][0]-start_pos[0], wpts[0][1]-start_pos[1]) < 0.05:
        wpts.pop(0)
    return wpts

# =============================================================================
# INIZIALIZZAZIONE DISPOSITIVI
# =============================================================================
robot = Robot()
timestep = int(robot.getBasicTimeStep())

left_motor = robot.getDevice("left wheel motor")
right_motor = robot.getDevice("right wheel motor")
left_motor.setPosition(float("inf"))
right_motor.setPosition(float("inf"))
left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)

# Manteniamo i sensori IR solo per la scrittura nel CSV, se ti servono i dati bruti
ps_names = ["ps0", "ps1", "ps2", "ps3", "ps4", "ps5", "ps6", "ps7"]
ps = [robot.getDevice(name) for name in ps_names]
for sensor in ps: 
    sensor.enable(timestep)

left_encoder = robot.getDevice("left wheel sensor")
right_encoder = robot.getDevice("right wheel sensor")
left_encoder.enable(timestep)
right_encoder.enable(timestep)

print("[SISTEMA] Avvio Controller: PURA ODOMETRIA.")

# Creazione mappa e pianificazione percorso iniziale
grid_map = path_planning.create_map_from_wbt(ESCENARIO)
waypoints = plan_route(grid_map, START, GOAL)

robot.step(timestep)
prev_left = left_encoder.getValue()
prev_right = right_encoder.getValue()

# Stato iniziale
x_est, y_est, theta_est = START[0], START[1], 0.0

# Setup file CSV per registrazione dati
os.makedirs(os.path.abspath(os.path.join(current_dir, "..", "..", "docs")), exist_ok=True)
csv_path = os.path.abspath(os.path.join(current_dir, "..", "..", "docs", f"datos_{ESCENARIO}_pure_odom.csv"))
csv_file = open(csv_path, "w", newline="")
csv_writer = csv.writer(csv_file)
csv_writer.writerow(["t", "x", "y", "theta_deg", "idx", "target_x", "target_y", "v", "w", "ps0", "ps1", "ps2", "ps3", "ps4", "ps5", "ps6", "ps7"])

path_index = 0
step_count = 0

# =============================================================================
# LOOP PRINCIPALE
# =============================================================================
while robot.step(timestep) != -1:
    step_count += 1

    # 1. LETTURA SENSORI
    cur_left = left_encoder.getValue()
    cur_right = right_encoder.getValue()
    raw_ir = [sensor.getValue() for sensor in ps]

    # 2. CALCOLO SPOSTAMENTO RUOTE
    dl = (cur_left - prev_left) * WHEEL_RADIUS
    dr = (cur_right - prev_right) * WHEEL_RADIUS
    prev_left, prev_right = cur_left, cur_right
    
    # 3. AGGIORNAMENTO ODOMETRIA (Sostituisce completamente l'EKF)
    x_est, y_est, theta_est = update_odometry(x_est, y_est, theta_est, dl, dr)

    # 4. CONTROLLO RAGGIUNGIMENTO OBIETTIVO
    dist_goal = math.hypot(GOAL[0] - x_est, GOAL[1] - y_est)
    if dist_goal < GOAL_TOL:
        left_motor.setVelocity(0.0)
        right_motor.setVelocity(0.0)
        print(f"[OK] Meta raggiunta! pos=({x_est:.3f}, {y_est:.3f})")
        break

    # 5. GESTIONE REPLANNING (Opzionale: calcola se il robot "crede" di essersi allontanato troppo)
    if path_planning.check_replanning(x_est, y_est, waypoints, path_index, max_deviation=0.12):
        print(f"[REPLAN] Ricalcolo percorso da odometria ({x_est:.2f}, {y_est:.2f})")
        waypoints = plan_route(grid_map, (x_est, y_est), GOAL)
        path_index = 0

    if not waypoints:
        left_motor.setVelocity(0.0)
        right_motor.setVelocity(0.0)
        continue

    # 6. PURE PURSUIT (Inseguimento Traiettoria)
    path_index, target_index, (tx, ty) = path_planning.manage_waypoints(
        x_est, y_est, waypoints, path_index, LOOKAHEAD_DIST, WAYPOINT_REACHED
    )

    dx, dy = tx - x_est, ty - y_est
    dist_target = math.hypot(dx, dy)
    target_angle = math.atan2(dy, dx)
    angle_error = norm_angle(target_angle - theta_est)

    # Logica di Sterzo e Avanzamento
    if abs(angle_error) > TURN_IN_PLACE_ANGLE:
        v_cmd = 0.0 # Ruota sul posto se l'angolo è troppo acuto
    else:
        align = max(0.0, 1.0 - abs(angle_error) / TURN_IN_PLACE_ANGLE)
        v_cmd = V_MIN + (V_MAX - V_MIN) * align
        # Rallenta in prossimità della meta finale
        if dist_goal < 0.25:
            v_cmd = min(v_cmd, 0.6 * dist_goal)

    w_cmd = clamp(K_W * angle_error, -MAX_W, MAX_W)

    # 7. APPLICAZIONE VELOCITÀ MOTORI
    left_v, right_v = make_motor_speeds(v_cmd, w_cmd)
    left_motor.setVelocity(left_v)
    right_motor.setVelocity(right_v)

    # 8. DEBUG E LOGGING
    if step_count % DEBUG_EVERY == 0:
        print(
            f"[DBG t={robot.getTime():.2f}s] pos=({x_est:.3f},{y_est:.3f}) "
            f"theta={math.degrees(theta_est):.1f}° idx={path_index}->{target_index} "
            f"err={math.degrees(angle_error):.1f}° w={w_cmd:.3f}"
        )

    csv_writer.writerow([
        robot.getTime(), x_est, y_est, math.degrees(theta_est), path_index, tx, ty,
        v_cmd, w_cmd, *[int(v) for v in raw_ir]
    ])

csv_file.close()