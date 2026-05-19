"""robotito-controller controller."""
"""
“Se implementó una arquitectura modular basada en secuencia de comportamientos, 
permitiendo ejecutar acciones de forma ordenada y extensible.”
"""
#importes
        
from controller import Robot
import random

robotito = Robot()

# el tiempo de los step (=! tiempo transcurrido)
timestep = int(robotito.getBasicTimeStep())

#motor
motor_izq = robotito.getDevice("left wheel motor")
motor_der = robotito.getDevice("right wheel motor")

#pos
motor_izq.setPosition(float('inf'))
motor_der.setPosition(float('inf'))

# sensores de proximidad
ps_names = ['ps0', 'ps7', 'ps5', 'ps2']

ps = []

for name in ps_names:
    sensor = robotito.getDevice(name)
    sensor.enable(timestep)
    ps.append(sensor)

# encoders
left_encoder = robotito.getDevice('left wheel sensor')
right_encoder = robotito.getDevice('right wheel sensor')

left_encoder.enable(timestep)
right_encoder.enable(timestep)

#tipos de movidas del robotito

def avanzar(v):
    return v, v

def curva(v1, v2):
    return v1, v2

def girar(v):
    return -v, v

def circulo(v1, v2):
    return v1, v2

def cuadrado(tiempo_local):
    lado = 5.0
    giro = 1.0  # a calibrar

    ciclo = lado + giro
    fase = tiempo_local % ciclo

    if fase < lado:
        return avanzar(3)
    else:
        return girar(2)

def ejecutar_accion(nombre, tiempo_local):
    if nombre == "curva":
        return curva(3.5, 2)
    elif nombre == "recto":
        return avanzar(3)
    elif nombre == "giro":
        return girar(4)
    elif nombre == "circulo":
        return circulo(3, 1.5)
    elif nombre == "cuadrado":
        return cuadrado(tiempo_local)
        
#prog

#cambiar si es necesario

indice = 0
tiempo_inicio = 0
pausa = False
duracion_pausa = 3
acciones = [
    ("curva", 4.5),
    ("recto", 7),   
    ("giro", 10),
    ("circulo", 12),
    ("cuadrado", 999),

]
robotito.step(timestep)
prev_left = left_encoder.getValue()
prev_right = right_encoder.getValue()
WHEEL_RADIUS = 0.0205

filtered_front = 0
alpha = 0.7

while robotito.step(timestep) != -1:

    tiempo = robotito.getTime()
    left_pos = left_encoder.getValue()
    right_pos = right_encoder.getValue()

    delta_left = left_pos - prev_left
    delta_right = right_pos - prev_right
    prev_left = left_pos
    prev_right = right_pos

    left_distance = WHEEL_RADIUS * delta_left
    right_distance = WHEEL_RADIUS * delta_right
    robot_advance = (left_distance + right_distance) / 2.0


    front_right = ps[0].getValue()
    front_left = ps[1].getValue()
    front_measure = (front_left + front_right) / 2.0
    filtered_front = alpha * filtered_front + (1 - alpha) * front_measure

    left_side = ps[2].getValue()
    right_side = ps[3].getValue()

    print(
        f"ADV: {robot_advance*1000:.2f} | "
        f"FILTERED F: {filtered_front:.2f} | "
        f"RAW FF: {front_measure:.2f} | "
        f"FL: {front_left:.2f} | "
        f"FR: {front_right:.2f} | "
        f"LS: {left_side:.2f} | "
        f"RS: {right_side:.2f}",
        flush=True
    )

    # para aplicar ruidos
    noise_left = random.uniform(-0.1, 0.1)
    noise_right = random.uniform(-0.1, 0.1)

    tiempo_local = tiempo - tiempo_inicio

    if pausa:
        # robot detenido
        motor_izq.setVelocity(0)
        motor_der.setVelocity(0)

        if tiempo_local > duracion_pausa:
            pausa = False
            tiempo_inicio = tiempo

    else:
        nombre, duracion = acciones[indice]
        
        v_izq, v_der = ejecutar_accion(nombre, tiempo_local)

        # aplica velocidad con ruido
        motor_izq.setVelocity(v_izq + noise_left)
        motor_der.setVelocity(v_der + noise_right)

        # cambio azione
        if tiempo_local > duracion:
            indice += 1
            pausa = True
            tiempo_inicio = tiempo

            if indice >= len(acciones):
                indice = len(acciones) - 1