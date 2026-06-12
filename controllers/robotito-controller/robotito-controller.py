"""
robotito-controller.py — Controlador Principal: Path Planning (A*) + Replanning dinámico
Proyecto Final ICI 4150 — Robótica y Sistemas Autónomos 2026-01

Integrantes:
  - Maria Paganetti
  - Ignacia Brahim
  - Diego Alvarado
  - Sean Jamen
  - Ariel Villar

Pipeline de navegación:
  1. Construir grilla inicial con obstáculos conocidos del .wbt
  2. A* desde posición actual hasta meta → lista de waypoints
  3. Seguir waypoints con control proporcional (Lab 1)
  4. Si Kalman detecta obstáculo imprevisto (celda libre en grilla pero sensor activo):
       a. Marcar celda frente al robot como ocupada
       b. Girar para alejarse de la pared
       c. Recalcular A* desde posición actual → nueva ruta
  5. Cooldown entre replanificaciones para evitar loops
"""

from controller import Robot
import math
import csv
import os
import path_planning

# =============================================================================
# CONFIGURACIÓN
# =============================================================================
ESCENARIO = "complejo"   # "simple" o "complejo"

CONFIG = {
    "simple":   {"start": (-0.9,  0.0), "goal": ( 0.9,  0.0)},
    "complejo": {"start": (-1.0, -1.0), "goal": ( 1.0,  1.0)},
}

# =============================================================================
# INICIALIZACIÓN DEL ROBOT
# =============================================================================

robotito = Robot()
timestep = int(robotito.getBasicTimeStep())

motor_izq = robotito.getDevice("left wheel motor")
motor_der = robotito.getDevice("right wheel motor")
motor_izq.setPosition(float('inf'))
motor_der.setPosition(float('inf'))
motor_izq.setVelocity(0.0)
motor_der.setVelocity(0.0)

ps_names = ['ps0', 'ps1', 'ps6', 'ps7', 'ps2', 'ps5']
ps = [robotito.getDevice(name) for name in ps_names]
for sensor in ps:
    sensor.enable(timestep)

left_encoder  = robotito.getDevice('left wheel sensor')
right_encoder = robotito.getDevice('right wheel sensor')
left_encoder.enable(timestep)
right_encoder.enable(timestep)

# =============================================================================
# CONSTANTES
# =============================================================================

WHEEL_RADIUS       = 0.0205
AXLE_LENGTH        = 0.052
MAX_SPEED          = 6.28
OBSTACLE_DIST      = 0.09    # distancia Kalman para detectar obstáculo
Q, R, P, d_est     = 0.005, 0.07, 1.0, 0.15
WAYPOINT_TOLERANCE = 0.06
INFLATE_RADIUS     = 3
COOLDOWN_PASOS     = 60      # pasos mínimos entre replanificaciones (~1 seg)
GIRO_ESCAPE_PASOS  = 40      # pasos girando para alejarse antes de replanificar

def sensor_a_metros(valor_crudo):
    if valor_crudo < 65:
        return 0.15
    try:
        return max(0.01, min(0.15, 1.16 / math.sqrt(valor_crudo)))
    except ZeroDivisionError:
        return 0.15

# =============================================================================
# FUNCIONES DE MAPA Y REPLANNING
# =============================================================================

def celda_es_libre(mapa, x, y):
    r, c = path_planning.world_to_grid(x, y)
    return mapa[r][c] == 0

def marcar_obstaculo(mapa, robot_x, robot_y, robot_theta, distancia):
    obs_x = robot_x + distancia * math.cos(robot_theta)
    obs_y = robot_y + distancia * math.sin(robot_theta)
    r, c = path_planning.world_to_grid(obs_x, obs_y)
    for dr in range(-INFLATE_RADIUS, INFLATE_RADIUS + 1):
        for dc in range(-INFLATE_RADIUS, INFLATE_RADIUS + 1):
            nr, nc = r + dr, c + dc
            if 0 <= nr < path_planning.GRID_SIZE and \
               0 <= nc < path_planning.GRID_SIZE:
                mapa[nr][nc] = 1
    return obs_x, obs_y

def replanificar(mapa, robot_x, robot_y, goal_cfg):
    start_cell = path_planning.world_to_grid(robot_x, robot_y)
    goal_cell  = path_planning.world_to_grid(*goal_cfg)
    # Si celda actual ocupada, buscar celda libre cercana
    if mapa[start_cell[0]][start_cell[1]] == 1:
        for dr in range(-4, 5):
            for dc in range(-4, 5):
                nr, nc = start_cell[0]+dr, start_cell[1]+dc
                if 0<=nr<path_planning.GRID_SIZE and \
                   0<=nc<path_planning.GRID_SIZE and \
                   mapa[nr][nc] == 0:
                    start_cell = (nr, nc)
                    break
            else:
                continue
            break
    path = path_planning.a_star(mapa, start_cell, goal_cell)
    if path:
        wps = [path_planning.grid_to_world(p[0], p[1]) for p in path]
        wps.pop(0)
        return wps
    return []

# =============================================================================
# PLANIFICACIÓN INICIAL
# =============================================================================

print(f"[Init] Construyendo mapa — escenario: {ESCENARIO}")
base_map  = path_planning.create_map_simple() if ESCENARIO == "simple" \
            else path_planning.create_map_complejo()
mapa      = path_planning.inflate_map(base_map, radius=INFLATE_RADIUS)
goal_cfg  = CONFIG[ESCENARIO]["goal"]
start_cfg = CONFIG[ESCENARIO]["start"]

waypoints = replanificar(mapa, start_cfg[0], start_cfg[1], goal_cfg)
if waypoints:
    print(f"[Init] Ruta inicial: {len(waypoints)} waypoints.")
    estado = "FOLLOW_PATH"
else:
    print("[ERROR] No se encontró ruta inicial.")
    estado = "ERROR"

# Longitud planificada
ruta_ini = path_planning.a_star(mapa,
    path_planning.world_to_grid(*start_cfg),
    path_planning.world_to_grid(*goal_cfg)) or []
longitud_planificada = sum(
    math.hypot(path_planning.grid_to_world(ruta_ini[i][0],ruta_ini[i][1])[0] -
               path_planning.grid_to_world(ruta_ini[i-1][0],ruta_ini[i-1][1])[0],
               path_planning.grid_to_world(ruta_ini[i][0],ruta_ini[i][1])[1] -
               path_planning.grid_to_world(ruta_ini[i-1][0],ruta_ini[i-1][1])[1])
    for i in range(1, len(ruta_ini)))

# =============================================================================
# ESTADO INICIAL
# =============================================================================

robot_x, robot_y, robot_theta = start_cfg[0], start_cfg[1], 0.0
robotito.step(timestep)
prev_left  = left_encoder.getValue()
prev_right = right_encoder.getValue()

paso               = 0
longitud_ejecutada = 0.0
n_replanificaciones = 0
dist_wp            = 0.0
cooldown_restante  = 0      # pasos hasta que se permite replanificar de nuevo
giro_escape_restante = 0    # pasos de giro de escape activos
dir_giro_escape    = 1      # dirección del giro de escape

# =============================================================================
# CSV
# =============================================================================

os.makedirs('../../docs', exist_ok=True)
csv_path   = f'../../docs/datos_{ESCENARIO}.csv'
csv_file   = open(csv_path, 'w', newline='')
csv_writer = csv.writer(csv_file)
csv_writer.writerow([
    'tiempo_s', 'x_est', 'y_est', 'theta_deg',
    'wp_restantes', 'dist_wp', 'estado',
    'crudo_m', 'kalman_m',
    'enc_izq_rad', 'enc_der_rad',
    'long_ejecutada_m', 'n_replanificaciones'
])

# =============================================================================
# LOOP PRINCIPAL
# =============================================================================

while robotito.step(timestep) != -1:

    # --- 1. Odometría (Lab 1) ---
    left_pos  = left_encoder.getValue()
    right_pos = right_encoder.getValue()
    dl = left_pos  - prev_left
    dr = right_pos - prev_right
    prev_left, prev_right = left_pos, right_pos

    ds_l   = WHEEL_RADIUS * dl
    ds_r   = WHEEL_RADIUS * dr
    ds     = (ds_r + ds_l) / 2.0
    dtheta = (ds_r - ds_l) / AXLE_LENGTH

    theta_mid    = robot_theta + dtheta / 2.0
    robot_x     += ds * math.cos(theta_mid)
    robot_y     += ds * math.sin(theta_mid)
    robot_theta += dtheta
    robot_theta  = math.atan2(math.sin(robot_theta), math.cos(robot_theta))
    longitud_ejecutada += abs(ds)

    # --- 2. Kalman (Lab 2) ---
    z_crudo = sensor_a_metros(max(ps[0].getValue(), ps[1].getValue(),
                                  ps[2].getValue(), ps[3].getValue()))
    d_pred = d_est - ds
    P_pred = P + Q
    K      = P_pred / (P_pred + R)
    d_est  = d_pred + K * (z_crudo - d_pred)
    P      = (1.0 - K) * P_pred

    if cooldown_restante > 0:
        cooldown_restante -= 1

    # --- 3. Máquina de estados ---
    v_izq, v_der = 0.0, 0.0

    if estado in ("ERROR", "DONE"):
        pass

    elif estado == "ESCAPE":
        # Girar en el lugar para alejarse de la pared antes de replanificar
        v_izq = -2.0 * dir_giro_escape
        v_der =  2.0 * dir_giro_escape
        giro_escape_restante -= 1
        if giro_escape_restante <= 0:
            # Ahora replanificar
            nuevos_wps = replanificar(mapa, robot_x, robot_y, goal_cfg)
            if nuevos_wps:
                waypoints = nuevos_wps
                print(f"  Nueva ruta tras escape: {len(waypoints)} waypoints.")
            estado = "FOLLOW_PATH"
            cooldown_restante = COOLDOWN_PASOS

    elif estado == "FOLLOW_PATH":

        # Obstáculo detectado + cooldown expirado
        if d_est < OBSTACLE_DIST and cooldown_restante == 0:

            # Solo replanificar si la celda proyectada es LIBRE en la grilla
            # (obstáculo genuinamente imprevisto)
            obs_x = robot_x + d_est * math.cos(robot_theta)
            obs_y = robot_y + d_est * math.sin(robot_theta)
            obs_libre = celda_es_libre(mapa, obs_x, obs_y)

            if obs_libre:
                # Obstáculo imprevisto real → marcar y replanificar
                n_replanificaciones += 1
                ox, oy = marcar_obstaculo(mapa, robot_x, robot_y, robot_theta, d_est)
                print(f"[Replanning #{n_replanificaciones}] "
                      f"Obstáculo imprevisto en ({ox:.2f},{oy:.2f}) — replanificando...")
                nuevos_wps = replanificar(mapa, robot_x, robot_y, goal_cfg)
                if nuevos_wps:
                    waypoints = nuevos_wps
                    print(f"  Nueva ruta: {len(waypoints)} waypoints.")
                cooldown_restante = COOLDOWN_PASOS
            else:
                # Obstáculo ya conocido → la ruta pasa muy cerca de una pared
                # Girar para alejarse antes de replanificar
                frontal_der = max(ps[0].getValue(), ps[1].getValue())
                frontal_izq = max(ps[2].getValue(), ps[3].getValue())
                dir_giro_escape    = 1 if frontal_der > frontal_izq else -1
                giro_escape_restante = GIRO_ESCAPE_PASOS
                estado = "ESCAPE"
                print(f"[Escape] Pared conocida detectada — girando para alejarse...")
                cooldown_restante = COOLDOWN_PASOS

        elif len(waypoints) == 0:
            dist_meta = math.hypot(goal_cfg[0]-robot_x, goal_cfg[1]-robot_y)
            if dist_meta < 0.15:
                print("[✓] ¡Meta alcanzada!")
                print(f"  Long. planificada : {longitud_planificada:.3f} m")
                print(f"  Long. ejecutada   : {longitud_ejecutada:.3f} m")
                print(f"  Replanificaciones : {n_replanificaciones}")
                estado = "DONE"
            else:
                nuevos_wps = replanificar(mapa, robot_x, robot_y, goal_cfg)
                if nuevos_wps:
                    waypoints = nuevos_wps

        else:
            target_x, target_y = waypoints[0]
            dx = target_x - robot_x
            dy = target_y - robot_y
            dist_wp       = math.hypot(dx, dy)
            angulo_target = math.atan2(dy, dx)
            error_angulo  = math.atan2(
                math.sin(angulo_target - robot_theta),
                math.cos(angulo_target - robot_theta))

            if dist_wp < WAYPOINT_TOLERANCE:
                waypoints.pop(0)
                if len(waypoints) % 10 == 0:
                    print(f"[Nav] Quedan {len(waypoints)} waypoints.")
            else:
                if abs(error_angulo) > 0.15:
                    giro  = 2.0 * (1 if error_angulo > 0 else -1)
                    v_izq = -giro
                    v_der =  giro
                else:
                    correccion = 1.5 * error_angulo
                    v_izq = 3.5 - correccion
                    v_der = 3.5 + correccion

    v_izq = max(min(v_izq, MAX_SPEED), -MAX_SPEED)
    v_der = max(min(v_der, MAX_SPEED), -MAX_SPEED)
    motor_izq.setVelocity(v_izq)
    motor_der.setVelocity(v_der)

    # --- 4. CSV ---
    tiempo_s = paso * timestep / 1000.0
    csv_writer.writerow([
        round(tiempo_s, 3),
        round(robot_x, 4), round(robot_y, 4), round(math.degrees(robot_theta), 2),
        len(waypoints), round(dist_wp, 4), estado,
        round(z_crudo, 4), round(d_est, 4),
        round(left_pos, 4), round(right_pos, 4),
        round(longitud_ejecutada, 4), n_replanificaciones
    ])
    paso += 1

csv_file.close()
print(f"[INFO] CSV guardado en {csv_path}")