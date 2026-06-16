import sys
import os
import math
from controller import Robot

current_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, current_dir)

import path_planning

# =============================================================================
# CONTROLADOR E-PUCK V2 — ODOMETRIA + EVITAMENTO CON RICALCOLO E STAMPA MAPPA
# =============================================================================

SCENARIOS = {
    "semplice": {
        "wbt_name": "semplice",
        "start":    (-1.0,  0.0),
        "goal":     ( 1.0,  0.0),
    },
    "complejo": {
        "wbt_name": "complejo",
        "start":    (-1.0, -1.0),
        "goal":     ( 1.0,  1.0),
    },
}

scenario_key = sys.argv[1] if len(sys.argv) > 1 else "complejo"
if scenario_key not in SCENARIOS:
    print(f"[WARN] Scenario '{scenario_key}' non trovato. Uso 'complejo'.")
    scenario_key = "complejo"

cfg       = SCENARIOS[scenario_key]
ESCENARIO = cfg["wbt_name"]
START     = cfg["start"]
GOAL      = cfg["goal"]

WHEEL_RADIUS  = 0.0205
AXLE_LENGTH   = 0.057
MAX_SPEED     = 6.28

V_MAX              = 0.105
V_MIN              = 0.015
K_W                = 2.0
MAX_W              = 0.9
LOOKAHEAD_DIST     = 0.08
WAYPOINT_REACHED   = 0.07
GOAL_TOL           = 0.08
TURN_IN_PLACE_ANGLE = math.radians(45)

IR_DANGER_THRESHOLD  = 150   

# FIX: Angoli rad esatti del modello fisico E-puck in Webots
# (Negativi = Lato Destro, Positivi = Lato Sinistro)
PS_ANGLES = [
     -0.2967, # ps0 (approx -17°)
     -0.8726, # ps1 (approx -50°)
     -1.5708, # ps2 (approx -90°)
     -2.6179, # ps3 (approx -150°)
      2.6179, # ps4 (approx +150°)
      1.5708, # ps5 (approx +90°)
      0.8726, # ps6 (approx +50°)
      0.2967  # ps7 (approx +17°)
]

class KalmanIR:
    def __init__(self, q=1.0, r=50.0, initial=0.0):
        self.x = initial 
        self.p = 100.0   
        self.q = q       
        self.r = r       
    
    def update(self, z):
        p_pred = self.p + self.q
        k = p_pred / (p_pred + self.r)
        self.x = self.x + k * (z - self.x)
        self.p = (1.0 - k) * p_pred
        return self.x

def clamp(value, lo, hi):
    return max(lo, min(hi, value))

def norm_angle(a):
    return math.atan2(math.sin(a), math.cos(a))

def make_motor_speeds(v, w):
    left  = (v - w * AXLE_LENGTH / 2.0) / WHEEL_RADIUS
    right = (v + w * AXLE_LENGTH / 2.0) / WHEEL_RADIUS
    return clamp(left, -MAX_SPEED, MAX_SPEED), clamp(right, -MAX_SPEED, MAX_SPEED)

def update_odometry(x, y, theta, dl, dr):
    v_dt = (dr + dl) / 2.0
    w_dt = (dr - dl) / AXLE_LENGTH
    x_new     = x + v_dt * math.cos(theta + w_dt / 2.0)
    y_new     = y + v_dt * math.sin(theta + w_dt / 2.0)
    theta_new = norm_angle(theta + w_dt)
    return x_new, y_new, theta_new

def plan_route(grid, start_pos, goal_pos, robot_radius=path_planning.ROBOT_RADIUS_CELLS):
    start_grid = path_planning.world_to_grid(*start_pos)
    goal_grid  = path_planning.world_to_grid(*goal_pos)
    route      = path_planning.a_star(grid, start_grid, goal_grid, robot_radius=robot_radius)
    
    if not route: return [], 0.0, []
    
    world_points = [path_planning.grid_to_world(r, c) for r, c in route]
    planned_length = sum(
        math.hypot(world_points[i][0] - world_points[i-1][0], world_points[i][1] - world_points[i-1][1])
        for i in range(1, len(world_points))
    )
    
    wpts = path_planning.downsample_world_path(world_points, spacing=0.10)
    if wpts and math.hypot(wpts[0][0]-start_pos[0], wpts[0][1]-start_pos[1]) < 0.05:
        wpts.pop(0)
    return wpts, planned_length, route

def check_obstacle(filtered_ir):
    trigger_idx = -1
    max_val = 0
    for i, val in enumerate(filtered_ir):
        if val > max_val:
            max_val = val
            trigger_idx = i
            
    danger = max_val > IR_DANGER_THRESHOLD
    return danger, trigger_idx

# =============================================================================
# INIZIALIZZAZIONE DISPOSITIVI
# =============================================================================
robot    = Robot()
timestep = int(robot.getBasicTimeStep())

left_motor  = robot.getDevice("left wheel motor")
right_motor = robot.getDevice("right wheel motor")
left_motor.setPosition(float("inf"))
right_motor.setPosition(float("inf"))
left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)

ps_names = ["ps0", "ps1", "ps2", "ps3", "ps4", "ps5", "ps6", "ps7"]
ps = [robot.getDevice(name) for name in ps_names]
for sensor in ps:
    sensor.enable(timestep)

left_encoder  = robot.getDevice("left wheel sensor")
right_encoder = robot.getDevice("right wheel sensor")
left_encoder.enable(timestep)
right_encoder.enable(timestep)

kalman_filters = [KalmanIR(q=1.0, r=50.0) for _ in range(8)]

print(f"[SISTEMA] Avvio Controller — scenario: {scenario_key}")

grid_map = path_planning.create_map_from_wbt(ESCENARIO)

waypoints, planned_length, raw_route = plan_route(grid_map, START, GOAL)

print("\n[MAPPA] --- VISUALIZZAZIONE ROTTA INIZIALE ---")
path_planning.visualize_path(grid_map, raw_route)
print("--------------------------------------------\n")

path_index = 0

robot.step(timestep)
prev_left  = left_encoder.getValue()
prev_right = right_encoder.getValue()

x_est, y_est, theta_est = START[0], START[1], 0.0

# Variabili FSM e Retromarcia
STATE_TRACKING = 0
STATE_REVERSING = 1
state = STATE_TRACKING
reverse_start_x = 0.0
reverse_start_y = 0.0
REVERSE_DIST_TARGET = 0.06 

# =============================================================================
# LOOP PRINCIPALE
# =============================================================================
while robot.step(timestep) != -1:

    cur_left  = left_encoder.getValue()
    cur_right = right_encoder.getValue()
    raw_ir    = [sensor.getValue() for sensor in ps]
    filtered_ir = [kalman_filters[i].update(raw_ir[i]) for i in range(8)]

    dl = (cur_left  - prev_left)  * WHEEL_RADIUS
    dr = (cur_right - prev_right) * WHEEL_RADIUS
    prev_left, prev_right = cur_left, cur_right

    x_est, y_est, theta_est = update_odometry(x_est, y_est, theta_est, dl, dr)

    dist_goal = math.hypot(GOAL[0] - x_est, GOAL[1] - y_est)
    if dist_goal < GOAL_TOL:
        left_motor.setVelocity(0.0)
        right_motor.setVelocity(0.0)
        print(f"[OK] Meta raggiunta con successo in pos=({x_est:.3f},{y_est:.3f})")
        break

    if state == STATE_TRACKING:
        
        danger, trigger_idx = check_obstacle(filtered_ir)
        
        if danger:
            val = filtered_ir[trigger_idx]
            print(f"\n[EVITAMENTO ATTIVATO] Ostacolo imprevisto rilevato dal sensore ps{trigger_idx} (Valore IR: {val:.1f})!")
            print("[AZIONE] Arresto motori e calcolo posizione ostacolo...")
            left_motor.setVelocity(0.0)
            right_motor.setVelocity(0.0)
            
            # FIX: Calcolo dinamico della distanza in base alla lettura del sensore
            # L'E-puck ha raggio 0.035m. 
            # A 1500 IR, l'ostacolo è quasi attaccato -> ~0.04m dal centro
            # A 250 IR (soglia), l'ostacolo è a ~3cm dal bordo -> ~0.065m dal centro
            obs_dist = 0.035 + (0.03 * (1.0 - min(val, 1500) / 1500.0))
            
            obs_angle = norm_angle(theta_est + PS_ANGLES[trigger_idx])
            obs_x = x_est + obs_dist * math.cos(obs_angle) 
            obs_y = y_est + obs_dist * math.sin(obs_angle)
            
            path_planning.add_dynamic_obstacle(grid_map, obs_x, obs_y, radius_m=0.015)
            print(f"[MAPPA] Ostacolo dinamico posizionato a {obs_dist:.3f}m dal centro robot in ({obs_x:.2f}, {obs_y:.2f}).")
            
            state = STATE_REVERSING
            reverse_start_x = x_est
            reverse_start_y = y_est
            print("[AZIONE] Avvio retromarcia di sicurezza...")
            continue
            
        if path_planning.check_replanning(x_est, y_est, waypoints, path_index, max_deviation=0.12):
            waypoints, _, raw_route = plan_route(grid_map, (x_est, y_est), GOAL)
            path_index = 0
            if not waypoints: continue

        path_index, target_index, (tx, ty) = path_planning.manage_waypoints(
            x_est, y_est, waypoints, path_index, LOOKAHEAD_DIST, WAYPOINT_REACHED
        )

        dx, dy       = tx - x_est, ty - y_est
        target_angle = math.atan2(dy, dx)
        angle_error  = norm_angle(target_angle - theta_est)

        if abs(angle_error) > TURN_IN_PLACE_ANGLE:
            v_cmd = 0.0
        else:
            align = max(0.0, 1.0 - abs(angle_error) / TURN_IN_PLACE_ANGLE)
            v_cmd = V_MIN + (V_MAX - V_MIN) * align
            if dist_goal < 0.25:
                v_cmd = min(v_cmd, 0.6 * dist_goal)

        w_cmd = clamp(K_W * angle_error, -MAX_W, MAX_W)
        left_v, right_v = make_motor_speeds(v_cmd, w_cmd)
        
        left_motor.setVelocity(left_v)
        right_motor.setVelocity(right_v)

    elif state == STATE_REVERSING:
        left_v, right_v = make_motor_speeds(-V_MAX * 0.7, 0.0)
        
        left_motor.setVelocity(left_v)
        right_motor.setVelocity(right_v)
        
        dist_rev = math.hypot(x_est - reverse_start_x, y_est - reverse_start_y)
        
        if dist_rev >= REVERSE_DIST_TARGET:
            left_motor.setVelocity(0.0)
            right_motor.setVelocity(0.0)
            print("[REPLAN] Retromarcia completata. Ricalcolo della rotta aggiornata...")
            
            waypoints, _, raw_route = plan_route(grid_map, (x_est, y_est), GOAL, robot_radius=2)
            
            print("\n[MAPPA] --- VISUALIZZAZIONE DOPO EVITAMENTO ---")
            path_planning.visualize_path(grid_map, raw_route)
            print("-----------------------------------------------\n")
            
            path_index = 0
            state = STATE_TRACKING
            print("[AZIONE] Ripresa del tracking verso l'obiettivo.\n")