#                ROBOTITO CONTROLLER
"""
Integrantes/Membri:
* Maria Paganetti
* Ignacia Brahim
* Diego Alvarado
* Sean Jamen
* Ariel Villar
"""
#---------------------------------------------------
from controller import Robot
import math
import csv

robotito = Robot()
timestep = int(robotito.getBasicTimeStep())

# --- Modulos ---
def sensor_a_metros(valor_crudo):
    if valor_crudo < 65:
        return 0.15
    try:
        distancia_metros = 1.16 / (math.sqrt(valor_crudo))
        return max(0.01, min(0.15, distancia_metros))
    except ZeroDivisionError:
        return 0.15

#-----------------------------------------------------
# --- Inicializacion del Hardware ---
motor_izq = robotito.getDevice("left wheel motor")
motor_der = robotito.getDevice("right wheel motor")
motor_izq.setPosition(float('inf'))
motor_der.setPosition(float('inf'))

motor_izq.setAcceleration(4.0)
motor_der.setAcceleration(4.0)

# Sensori
ps_names = ['ps0', 'ps1', 'ps6', 'ps7', 'ps2', 'ps5']
ps = [robotito.getDevice(name) for name in ps_names]
for sensor in ps:
    sensor.enable(timestep)

# Encoders
left_encoder = robotito.getDevice('left wheel sensor')
right_encoder = robotito.getDevice('right wheel sensor')
left_encoder.enable(timestep)
right_encoder.enable(timestep)

# --- Constantes ---
WHEEL_RADIUS = 0.0205

# --- SOGLIE ISTERESI (in metros, usadas con d_est de Kalman) ---
ENTER_AVOID_DIST = 0.128
EXIT_AVOID_DIST  = 0.133

# Quanti "step" di simulazione deve durare la frenata (15 steps * 16ms = ~240 millisecondi)
PASOS_FRENADO = 15

# --- Parametros Filtrado Kalman ---
Q = 0.005
R = 0.07
P = 1.0
d_est = 0.15

# --- Variabili Filtro Semplice ---
ALPHA = 0.25
valor_filtrado_prev = 0.15

# --- Loop Variables ---
robotito.step(timestep)
prev_left = left_encoder.getValue()
prev_right = right_encoder.getValue()

estado = "FORWARD"
contador_freno = 0
paso = 0

# --- Registro de datos ---
csv_file = open('../../docs/datos_simulacion.csv', 'w', newline='')
csv_writer = csv.writer(csv_file)
csv_writer.writerow(['tiempo_s', 'crudo_metros', 'filtrado_metros', 'kalman_metros', 'encoder_izq_rad', 'encoder_der_rad'])

# --- Main Simulation ---
while robotito.step(timestep) != -1:

    # 1. Odometria (Δd)
    left_pos = left_encoder.getValue()
    right_pos = right_encoder.getValue()

    delta_left = left_pos - prev_left
    delta_right = right_pos - prev_right

    prev_left = left_pos
    prev_right = right_pos

    left_distance = WHEEL_RADIUS * delta_left
    right_distance = WHEEL_RADIUS * delta_right
    robot_advance = (left_distance + right_distance) / 2.0

    # 2. Lettura dei Sensori
    val_ps0 = ps[0].getValue()
    val_ps1 = ps[1].getValue()
    val_ps6 = ps[2].getValue()
    val_ps7 = ps[3].getValue()

    left_side = ps[4].getValue()
    right_side = ps[5].getValue()

    valor_crudo_massimo = max(val_ps0, val_ps1, val_ps6, val_ps7)
    z_k_crudo = sensor_a_metros(valor_crudo_massimo)

    # === FILTRO SEMPLICE ===
    z_k = (ALPHA * z_k_crudo) + ((1.0 - ALPHA) * valor_filtrado_prev)
    valor_filtrado_prev = z_k

    # =================================================================
    # 3. ALGORITMO FILTRO KALMAN
    # =================================================================
    delta_d = -robot_advance
    d_pred = d_est + delta_d
    P_pred = P + Q

    y = z_k_crudo - d_pred
    S = P_pred + R
    K = P_pred / S

    d_est = d_pred + K * y
    P = (1.0 - K) * P_pred

    # =================================================================
    # 4. Logica di Navegación a 3 Stati
    # =================================================================

    if estado == "FORWARD":
        if d_est < ENTER_AVOID_DIST:
            estado = "FRENANDO"
            contador_freno = PASOS_FRENADO

    elif estado == "FRENANDO":
        if contador_freno > 0:
            contador_freno -= 1
        else:
            estado = "AVOID"

    elif estado == "AVOID":
        if d_est > EXIT_AVOID_DIST:
            estado = "FORWARD"

    # Attuazione motori basata sullo stato
    if estado == "FORWARD":
        v_izq = 2.0
        v_der = 2.0
    elif estado == "FRENANDO":
        v_izq = 0.0
        v_der = 0.0
    elif estado == "AVOID":
        if max(val_ps0, val_ps1) > max(val_ps6, val_ps7):
            v_izq = -1.2
            v_der = 1.2
        else:
            v_izq = 1.2
            v_der = -1.2

    # --- Guardar fila en CSV ---
    tiempo_s = paso * timestep / 1000.0
    csv_writer.writerow([
        round(tiempo_s, 3),
        round(z_k_crudo, 4),
        round(z_k, 4),
        round(d_est, 4),
        round(left_pos, 4),
        round(right_pos, 4)
    ])
    paso += 1

    print(
        "Estado:", estado,
        "| Kalman d_est:", round(d_est, 3),
        "| Crudo Max:", round(valor_crudo_massimo, 1)
    )

    motor_izq.setVelocity(v_izq)
    motor_der.setVelocity(v_der)

csv_file.close()
