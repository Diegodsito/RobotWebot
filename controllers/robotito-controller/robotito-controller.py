"""
robotito-controller.py — Controlador Principal: Path Planning (A*) + Evitación Reactiva
Proyecto Final ICI 4150 — Robótica y Sistemas Autónomos 2026-01

Integrantes:
  - Maria Paganetti
  - Ignacia Brahim
  - Diego Alvarado
  - Sean Jamen
  - Ariel Villar

Línea A: Planificación de rutas con A* sobre grilla de ocupación.
  Lab 1 → cinemática diferencial (seguimiento de waypoints con control proporcional)
  Lab 2 → encoders + filtro de Kalman (odometría y estimación de distancia frontal)
  Nuevo  → grilla de ocupación + A* (path_planning.py)
"""

from controller import Robot
import math
import csv
import os
import path_planning

# =============================================================================
# CONFIGURACIÓN — cambiar según escenario
# =============================================================================
ESCENARIO = "simple"   # "simple" o "complejo"

CONFIG = {
    "simple":   {"start": (-0.9,  0.0), "goal": ( 0.9,  0.0)},
    "complejo": {"start": (-1.1, -1.1), "goal": ( 1.1,  1.1)},
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

# ps[0]=ps0, ps[1]=ps1  → frontal derecho
# ps[2]=ps6, ps[3]=ps7  → frontal izquierdo
# ps[4]=ps2             → lateral izquierdo
# ps[5]=ps5             → lateral derecho
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

WHEEL_RADIUS     = 0.0205
AXLE_LENGTH      = 0.052
MAX_SPEED        = 6.28
ENTER_AVOID_DIST = 0.10
EXIT_AVOID_DIST  = 0.13
Q, R, P, d_est   = 0.005, 0.07, 1.0, 0.15
WAYPOINT_TOLERANCE = 0.05

def sensor_a_metros(valor_crudo):
    if valor_crudo < 65:
        return 0.15
    try:
        return max(0.01, min(0.15, 1.16 / math.sqrt(valor_crudo)))
    except ZeroDivisionError:
        return 0.15

# =============================================================================
# PLANIFICACIÓN DE RUTA (A*)
# =============================================================================

print(f"[Sistema Global] Construyendo mapa y calculando ruta — escenario: {ESCENARIO}")

base_map   = path_planning.create_map_simple() if ESCENARIO == "simple" \
             else path_planning.create_map_complejo()
safe_map   = path_planning.inflate_map(base_map)
start_cfg  = CONFIG[ESCENARIO]["start"]
goal_cfg   = CONFIG[ESCENARIO]["goal"]
start_grid = path_planning.world_to_grid(*start_cfg)
goal_grid  = path_planning.world_to_grid(*goal_cfg)
grid_path  = path_planning.a_star(safe_map, start_grid, goal_grid)

# Ruta planificada en coordenadas del mundo (para métricas)
ruta_mundo = []
waypoints  = []

if grid_path:
    for p in grid_path:
        wx, wy = path_planning.grid_to_world(p[0], p[1])
        ruta_mundo.append((wx, wy))
        waypoints.append((wx, wy))
    print(f"[Sistema Global] Ruta calculada: {len(waypoints)} waypoints.")
    waypoints.pop(0)
    estado = "FOLLOW_PATH"
else:
    print("[ERROR] No se encontró ruta. El robot permanecerá detenido.")
    estado = "ERROR"

# Longitud de ruta planificada
longitud_planificada = 0.0
for i in range(1, len(ruta_mundo)):
    dx = ruta_mundo[i][0] - ruta_mundo[i-1][0]
    dy = ruta_mundo[i][1] - ruta_mundo[i-1][1]
    longitud_planificada += math.hypot(dx, dy)

# =============================================================================
# ESTADO INICIAL
# =============================================================================

robot_x, robot_y, robot_theta = start_cfg[0], start_cfg[1], 0.0

robotito.step(timestep)
prev_left  = left_encoder.getValue()
prev_right = right_encoder.getValue()

# Métricas acumuladas
paso              = 0
longitud_ejecutada = 0.0
n_activaciones_avoid = 0
en_avoid_prev     = False

# =============================================================================
# CSV
# =============================================================================

os.makedirs('../../docs', exist_ok=True)
csv_path = f'../../docs/datos_{ESCENARIO}.csv'
csv_file   = open(csv_path, 'w', newline='')
csv_writer = csv.writer(csv_file)
csv_writer.writerow([
    'tiempo_s',
    'x_est', 'y_est', 'theta_deg',
    'wp_restantes', 'dist_wp',
    'estado',
    'crudo_m', 'kalman_m',
    'enc_izq_rad', 'enc_der_rad',
    'long_ejecutada_m'
])

# =============================================================================
# LOOP PRINCIPAL
# =============================================================================

while robotito.step(timestep) != -1:

    # --- 1. Encoders → odometría ---
    left_pos  = left_encoder.getValue()
    right_pos = right_encoder.getValue()
    delta_left  = left_pos  - prev_left
    delta_right = right_pos - prev_right
    prev_left, prev_right = left_pos, right_pos

    ds_l   = WHEEL_RADIUS * delta_left
    ds_r   = WHEEL_RADIUS * delta_right
    ds     = (ds_r + ds_l) / 2.0
    dtheta = (ds_r - ds_l) / AXLE_LENGTH

    theta_mid    = robot_theta + dtheta / 2.0
    robot_x     += ds * math.cos(theta_mid)
    robot_y     += ds * math.sin(theta_mid)
    robot_theta += dtheta
    robot_theta  = math.atan2(math.sin(robot_theta), math.cos(robot_theta))

    longitud_ejecutada += abs(ds)

    # --- 2. Kalman ---
    valor_crudo_max = max(ps[0].getValue(), ps[1].getValue(),
                          ps[2].getValue(), ps[3].getValue())
    z_k_crudo = sensor_a_metros(valor_crudo_max)

    d_pred = d_est - ds
    P_pred = P + Q
    K      = P_pred / (P_pred + R)
    d_est  = d_pred + K * (z_k_crudo - d_pred)
    P      = (1.0 - K) * P_pred

    # --- 3. Máquina de estados ---
    v_izq, v_der = 0.0, 0.0
    dist_wp = 0.0

    if estado in ("ERROR", "DONE"):
        v_izq, v_der = 0.0, 0.0

    elif estado == "FOLLOW_PATH":
        if d_est < ENTER_AVOID_DIST:
            estado = "AVOID"
            print(f"[Reactivo] Obstáculo detectado (d_est={d_est:.3f}m). Activando AVOID.")

        elif len(waypoints) == 0:
            print("[Control] ¡Meta alcanzada! Navegación completada.")
            print(f"  Longitud planificada : {longitud_planificada:.3f} m")
            print(f"  Longitud ejecutada   : {longitud_ejecutada:.3f} m")
            print(f"  Activaciones AVOID   : {n_activaciones_avoid}")
            estado = "DONE"

        else:
            target_x, target_y = waypoints[0]
            dx = target_x - robot_x
            dy = target_y - robot_y
            dist_wp       = math.hypot(dx, dy)
            angulo_target = math.atan2(dy, dx)
            error_angulo  = math.atan2(
                math.sin(angulo_target - robot_theta),
                math.cos(angulo_target - robot_theta)
            )

            if dist_wp < WAYPOINT_TOLERANCE:
                waypoints.pop(0)
                remaining = len(waypoints)
                if remaining % 10 == 0:
                    print(f"[Control] Waypoint alcanzado. Quedan {remaining} nodos.")
            else:
                velocidad_base = 0.8 if abs(error_angulo) > 0.4 else 3.5
                correccion     = 2.5 * error_angulo
                v_izq = velocidad_base - correccion
                v_der = velocidad_base + correccion

    elif estado == "AVOID":
        # Contar transiciones FOLLOW_PATH → AVOID
        if not en_avoid_prev:
            n_activaciones_avoid += 1

        if d_est > EXIT_AVOID_DIST:
            estado = "FOLLOW_PATH"
            print("[Reactivo] Obstáculo superado. Retomando ruta planificada.")
        else:
            frontal_der = max(ps[0].getValue(), ps[1].getValue())
            frontal_izq = max(ps[2].getValue(), ps[3].getValue())
            if frontal_der > frontal_izq:
                v_izq, v_der = -2.0,  2.0
            else:
                v_izq, v_der =  2.0, -2.0

    en_avoid_prev = (estado == "AVOID")

    # Clamp
    v_izq = max(min(v_izq, MAX_SPEED), -MAX_SPEED)
    v_der = max(min(v_der, MAX_SPEED), -MAX_SPEED)

    motor_izq.setVelocity(v_izq)
    motor_der.setVelocity(v_der)

    # --- 4. Registro CSV ---
    tiempo_s = paso * timestep / 1000.0
    csv_writer.writerow([
        round(tiempo_s, 3),
        round(robot_x, 4), round(robot_y, 4), round(math.degrees(robot_theta), 2),
        len(waypoints), round(dist_wp, 4),
        estado,
        round(z_k_crudo, 4), round(d_est, 4),
        round(left_pos, 4), round(right_pos, 4),
        round(longitud_ejecutada, 4)
    ])
    paso += 1

csv_file.close()
print(f"[INFO] CSV guardado en {csv_path}")
