"""
Controller Principale - Progetto Finale: Path Planning (A*) + Evitamento Reattivo
"""
from controller import Robot
import math
import path_planning

robotito = Robot()
timestep = int(robotito.getBasicTimeStep())

# 1. SETUP HARDWARE E COSTANTI
motor_izq = robotito.getDevice("left wheel motor")
motor_der = robotito.getDevice("right wheel motor")
motor_izq.setPosition(float('inf'))
motor_der.setPosition(float('inf'))
motor_izq.setVelocity(0.0)
motor_der.setVelocity(0.0)

ps_names = ['ps0', 'ps1', 'ps6', 'ps7', 'ps2', 'ps5']
ps = [robotito.getDevice(name) for name in ps_names]
for sensor in ps: sensor.enable(timestep)

left_encoder = robotito.getDevice('left wheel sensor')
right_encoder = robotito.getDevice('right wheel sensor')
left_encoder.enable(timestep)
right_encoder.enable(timestep)

WHEEL_RADIUS = 0.0205
AXLE_LENGTH = 0.052
MAX_SPEED = 6.28

ENTER_AVOID_DIST = 0.12  
EXIT_AVOID_DIST  = 0.15 
Q, R, P, d_est = 0.005, 0.07, 1.0, 0.15

# 2. CALCOLO DELLA ROTTA GLOBALE (A*)
print("[Sistema Globale] Inizializzazione mappa e pianificazione rotta...")
base_map = path_planning.create_map()
safe_map = path_planning.inflate_map(base_map)

start_grid = path_planning.world_to_grid(-1.1, -1.1)
goal_grid = path_planning.world_to_grid(1.1, 1.1)
grid_path = path_planning.a_star(safe_map, start_grid, goal_grid)

waypoints = []
if grid_path:
    for p in grid_path:
        wx, wy = path_planning.grid_to_world(p[0], p[1])
        waypoints.append((wx, wy))
    print(f"[Sistema Globale] Percorso calcolato: {len(waypoints)} waypoint generati.")
    waypoints.pop(0) 
    estado = "FOLLOW_PATH"
else:
    print("[Errore Sistema] Impossibile trovare una rotta sicura. Il robot rimane fermo.")
    estado = "ERROR" # Blocca il robot!

# 3. VARIABILI DI STATO E ODOMETRIA
robot_x, robot_y, robot_theta = -1.1, -1.1, 0.0 
robotito.step(timestep)
prev_left = left_encoder.getValue()
prev_right = right_encoder.getValue()
waypoint_tolerance = 0.03

def sensor_a_metros(valor_crudo):
    if valor_crudo < 65: return 0.15
    try: return max(0.01, min(0.15, 1.16 / math.sqrt(valor_crudo)))
    except ZeroDivisionError: return 0.15

# 4. LOOP PRINCIPALE
while robotito.step(timestep) != -1:
    
    # Odometria
    left_pos = left_encoder.getValue()
    right_pos = right_encoder.getValue()
    delta_left = left_pos - prev_left
    delta_right = right_pos - prev_right
    prev_left, prev_right = left_pos, right_pos
    
    ds_l = WHEEL_RADIUS * delta_left
    ds_r = WHEEL_RADIUS * delta_right
    ds = (ds_r + ds_l) / 2.0
    dtheta = (ds_r - ds_l) / AXLE_LENGTH
    theta_mid = robot_theta + dtheta / 2.0

    robot_x += ds * math.cos(theta_mid)
    robot_y += ds * math.sin(theta_mid)
    robot_theta += dtheta
    robot_theta = math.atan2(math.sin(robot_theta), math.cos(robot_theta))
    
    # Kalman
    valor_crudo_massimo = max([ps[i].getValue() for i in range(4)])
    z_k_crudo = sensor_a_metros(valor_crudo_massimo)
    d_pred = d_est - ds
    P_pred = P + Q
    K = P_pred / (P_pred + R)
    d_est = d_pred + K * (z_k_crudo - d_pred)
    P = (1.0 - K) * P_pred
    
    # Controllo
    v_izq, v_der = 0.0, 0.0
    
    if estado == "ERROR" or estado == "DONE":
        v_izq, v_der = 0.0, 0.0

    elif estado == "FOLLOW_PATH":
        if d_est < ENTER_AVOID_DIST:
            estado = "AVOID"
            print("[Navigazione Reattiva] Ostacolo imprevisto. Transizione a stato AVOID.")
        elif len(waypoints) == 0:
            print("[Controllo] Navigazione completata con successo. Meta raggiunta.")
            estado = "DONE" # Cambio stato per evitare lo spam
        else:
            target_x, target_y = waypoints[0]
            dx, dy = target_x - robot_x, target_y - robot_y
            distanza = math.hypot(dx, dy)
            angolo_target = math.atan2(dy, dx)
            
            errore_angolo = angolo_target - robot_theta
            errore_angolo = math.atan2(math.sin(errore_angolo), math.cos(errore_angolo))
            
            if distanza < waypoint_tolerance:
                waypoints.pop(0)
                if len(waypoints) % 10 == 0: 
                    print(f"[Controllo] Waypoint raggiunto. {len(waypoints)} nodi rimanenti.")
            else:
                if abs(errore_angolo) > 0.4:
                    velocita_base = 0.5 
                else:
                    velocita_base = 3.5 
                
                correzione_sterzo = 2.5 * errore_angolo
                v_izq = velocita_base - correzione_sterzo
                v_der = velocita_base + correzione_sterzo

    elif estado == "AVOID":
        if d_est > EXIT_AVOID_DIST:
            estado = "FOLLOW_PATH"
            print("[Navigazione Reattiva] Ostacolo superato. Ripresa del tracciato globale.")
        else:
            if max(ps[0].getValue(), ps[1].getValue()) > max(ps[2].getValue(), ps[3].getValue()):
                v_izq, v_der = -2.0, 2.0 
            else:
                v_izq, v_der = 2.0, -2.0 
            
    v_izq = max(min(v_izq, MAX_SPEED), -MAX_SPEED)
    v_der = max(min(v_der, MAX_SPEED), -MAX_SPEED)

    motor_izq.setVelocity(v_izq)
    motor_der.setVelocity(v_der)