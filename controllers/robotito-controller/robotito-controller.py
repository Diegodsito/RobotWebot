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

robotito = Robot()
#render == tiempo
timestep = int(robotito.getBasicTimeStep())


# --- Modulos ---

#transforma valores de sensor p (e-puck), osea valores de 0 a 4096
#a valores en metros/cm "reales". Esto es estimado, se puede
#calibrar mejor. 
def sensor_a_metros(valor_crudo):
    if valor_crudo < 50: 
        #si el valor es muy bajo, asumimos que el obstaculo está 
        #lejos (ej. 15 cm)
        return 0.15
        
    #aproximación clásica para el e-puck:
    #a mayor valor crudo, menor es la distancia estimada.
    try:
    
        distancia_metros = 0.4 / (math.sqrt(valor_crudo))
        
        #limitamos el rango de lectura lógico del sensor 
        #máximo 15 cm, mínimo 1 cm
        return max(0.01, min(0.15, distancia_metros))
        
    except ZeroDivisionError:
        #muy cercano a 0 o 0 es que está muy muy lejos.
        return 0.15
        
#-----------------------------------------------------
# --- Inicializacion del Hardware ---
motor_izq = robotito.getDevice("left wheel motor")
motor_der = robotito.getDevice("right wheel motor")
motor_izq.setPosition(float('inf'))
motor_der.setPosition(float('inf'))

#sensores
ps_names = ['ps0', 'ps7', 'ps5', 'ps2']

#enable por sensor
ps = [robotito.getDevice(name) for name in ps_names]
for sensor in ps:
    sensor.enable(timestep)

#encoders
left_encoder = robotito.getDevice('left wheel sensor')
right_encoder = robotito.getDevice('right wheel sensor')
left_encoder.enable(timestep)
right_encoder.enable(timestep)

# --- Constantes ---
#por ahora solo dejaremos el radio de la rueda
WHEEL_RADIUS = 0.0205

# --- Parametros Filtrado Kalman ---
#En el experimento se van a ir calibrando estos params (To do)

Q = 0.01   #varianza/incertidumbre del encoder (Encoder uncertainty)
R = 0.1    #varianza de medicion/sensor (Sensor uncertainty)
P = 1.0    #estimacion inicial del error del sensor (Covariance)
d_est = 0.5 #distancia inicial estimada (Distance/meters)

# --- Loop Variables ---
robotito.step(timestep)
#prev porque debemos estimar la distancia actual en base a eso.
prev_left = left_encoder.getValue()
prev_right = right_encoder.getValue()

# --- Main Simulation ---
#filtrado Kalman
while robotito.step(timestep) != -1:
    
    # 1. leer encoders y estimar distancia (Δd)
    left_pos = left_encoder.getValue()
    right_pos = right_encoder.getValue()
    
    #entonces la distancia es:
    #actual - anterior = diferencia.
    delta_left = left_pos - prev_left
    delta_right = right_pos - prev_right
    
    #actualizar prev
    prev_left = left_pos
    prev_right = right_pos
    
    #delta_x esta en radianes
    #metros == radio rueda * radianes (angulo)
    left_distance = WHEEL_RADIUS * delta_left
    right_distance = WHEEL_RADIUS * delta_right
    
    #las ruedas se pueden mover de forma distinta.
    #advance calcula cuanto se mueve el chasis
    #o el robot "completo". Se promedian.
    robot_advance = (left_distance + right_distance) / 2.0  #Δd_k
    
    # 2. Leer sensores frontales (Measurement z_k)
    front_right = ps[0].getValue()
    front_left = ps[1].getValue()
    
    #Ignacia
    """nota: los sensores de e-puck van a retornar un valor muy 
    alto siestá muy cerca de un obstaculo. 
    
    como los valores van de [0, 4096] se necesita una formula 
    para mapear los valores del sensor a metros (real) (z_k)
    
    
    la formula esta bien; lo que sufre la transformacion es
    front_left/right:
    
    z_k = (front_left + front_right) / 2.0 old formula
    
    SE IMPLEMENTA sensor_a_metros asi que se calcula z_k con eso.
    Leer sensor_a_metros para entender el cambio.
    
    """
    
    
    #el promedio de los valores de p (infrarojo) 
    #se traduce a metros reales
    
    valor_crudo_promedio = (front_left + front_right) / 2.0
    z_k = sensor_a_metros(valor_crudo_promedio) #z_k en metros)
    
    # =================================================================
    # 3. ALGORITMO FILTRO KALMAN (Ignacia)
    # =================================================================
    
    #paso 3.1 -- prediccion (proyectar el estado futuro)
    #mientras el robot avanza, su distancia al obstaculo disminuye
    #por el mismo valor.
    d_pred = d_est - robot_advance 
    
    #inseguridad nueva = inseguridad previa + "ruido"
    P_pred = P + Q
    
    #paso 3.2 -- correccion (actualizar estimado con medida del sensor)
    
    #sorpresa: lo que los sensores vieron - lo que nosotros calculamos
    y = z_k - d_pred 
    
    #ruido total del sistema: prediccion y (des)confianza sensor
    
    S = P_pred + R
    
    #ganancia de Kalman. Va de 0 a 1:
    #le cree al sensor entre 1 y 0.5
    #le cree a las ruedas (encoder) entre 0.5 y 0
    K = P_pred / S
    
    #paso 3.3 -- actualizar estados
    """
    nueva distancia es:
    nuestra prediccion + la "sorpresa"/error * confianza sensor
    """
    d_est = d_pred + K * y
    
    #habia mucha incertidumbre antes de corregir, asi que
    #ahora que estamos mas seguros, baja la incertidumbre proxima
    P = (1.0 - K) * P_pred
    
    # =================================================================
    # 4. Logica de Navegación Reactiva (No sé? Maria creo)
    # =================================================================
    # Use your fused 'd_est' here to decide movement!
    
    # Placeholder: Drive forward safely
    v_izq, v_der = 3.0, 3.0
    
    motor_izq.setVelocity(v_izq)
    motor_der.setVelocity(v_der)