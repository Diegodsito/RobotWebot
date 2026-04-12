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

while robotito.step(timestep) != -1:

    tiempo = robotito.getTime()

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