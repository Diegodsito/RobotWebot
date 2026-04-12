# Laboratorio de Robótica: Control de Movimientos en Webots

Este repositorio contiene la implementación de controladores para un robot móvil diferencial en el simulador **Webots**. El objetivo es comprender el comportamiento cinemático de un sistema diferencial mediante simulaciones interactivas donde los actuadores (motores de las ruedas) controlan la trayectoria del robot.

## Contenido
1. [Fundamentos Cinemáticos](#fundamentos-cinemáticos)
2. [Trayectorias Simuladas](#trayectorias-simuladas)
3. [Requisitos](#requisitos)
4. [Instrucciones de Uso](#instrucciones-de-uso)
5. [Simulaciones de Movimiento](#simulaciones-de-movimiento)

---

## Fundamentos Cinemáticos

El movimiento del robot se rige por las ecuaciones de la cinemática diferencial. Los parámetros del modelo son:

| Parámetro | Símbolo | Descripción |
|-----------|:-------:|-------------|
| Radio de rueda | $r$ | Radio de las ruedas del robot |
| Distancia entre ruedas | $L$ | Separación entre rueda izquierda y derecha |
| Velocidad rueda izquierda | $V_l$ | Velocidad angular de la rueda izquierda |
| Velocidad rueda derecha | $V_r$ | Velocidad angular de la rueda derecha |

A partir de estos parámetros se obtienen las velocidades del robot:

| Magnitud | Fórmula | Descripción |
|----------|:-------:|-------------|
| Velocidad Lineal ($v$) | $v = \dfrac{r(V_r + V_l)}{2}$ | Desplazamiento hacia adelante/atrás |
| Velocidad Angular ($\omega$) | $\omega = \dfrac{r(V_r - V_l)}{L}$ | Rotación sobre el eje vertical |

---

## Trayectorias Simuladas

| # | Trayectoria | Condición | Descripción |
|:-:|-------------|:---------:|-------------|
| 1 | **Línea Recta** | $V_l = V_r$ | Desplazamiento uniforme hacia adelante |
| 2 | **Curva** | $V_l \neq V_r$ | Giro con radio proporcional a la diferencia de velocidades |
| 3 | **Giro sobre el eje** | $V_l = -V_r$ | Rotación estática sin desplazamiento lineal ($v = 0,\ \omega \neq 0$) |
| 4 | **Círculo** | $V_l, V_r = \text{cte},\ V_l \neq V_r$ | Trayectoria circular continua de radio fijo |

---

## Requisitos

| Componente | Detalle |
|------------|---------|
| Simulador | [Webots R2023b](https://cyberbotics.com/) o superior |
| Lenguaje | Python 3.14 |
| Robot | Prototipo de robot diferencial ("Robotito") |

---

## Instrucciones de Uso

1. **Clona el repositorio:**
   ```bash
   git clone https://github.com/Diegodsito/RobotWebot.git
   cd RobotWebot
   ```

2. **Abre Webots** y carga el mundo ubicado en `worlds/robot_world.wbt`.

3. **Verifica** que el controlador del robot esté vinculado a la carpeta `controllers/robot_controller/`.

4. **Presiona Play** en el simulador.

> ⚠️ **Nota:** Asegúrate de tener instalada la versión correcta de Webots antes de cargar el mundo.

---

## Controlador

El controlador está implementado en Python y sigue una **arquitectura de secuencia de comportamientos**: ejecuta cada movimiento durante un tiempo determinado, hace una pausa breve, y pasa al siguiente. Se aplica un pequeño ruido aleatorio a las velocidades para simular imperfecciones reales del motor.

Las funciones de movimiento traducen directamente las condiciones cinemáticas a velocidades de rueda:

| Función | $V_l$ | $V_r$ | Resultado |
|---------|:-----:|:-----:|-----------|
| `avanzar(v)` | $v$ | $v$ | Línea recta |
| `curva(v1, v2)` | $v_1$ | $v_2$ | Curva hacia el lado más lento |
| `girar(v)` | $-v$ | $v$ | Giro sobre el propio eje |
| `circulo(v1, v2)` | $v_1$ | $v_2$ | Trayectoria circular continua |

<details>
<summary>Ver código completo del controlador</summary>

```python
from controller import Robot
import random

robotito = Robot()
timestep = int(robotito.getBasicTimeStep())

motor_izq = robotito.getDevice("left wheel motor")
motor_der = robotito.getDevice("right wheel motor")

motor_izq.setPosition(float('inf'))
motor_der.setPosition(float('inf'))

def avanzar(v):
    return v, v

def curva(v1, v2):
    return v1, v2

def girar(v):
    return -v, v

def circulo(v1, v2):
    return v1, v2

def ejecutar_accion(nombre, tiempo_local):
    if nombre == "curva":
        return curva(3.5, 2)
    elif nombre == "recto":
        return avanzar(3)
    elif nombre == "giro":
        return girar(4)
    elif nombre == "circulo":
        return circulo(3, 1.5)

acciones = [
    ("curva",   4.5),
    ("recto",   7.0),
    ("giro",   10.0),
    ("circulo", 12.0),
]

indice = 0
tiempo_inicio = 0
pausa = False
duracion_pausa = 3

while robotito.step(timestep) != -1:
    tiempo = robotito.getTime()
    tiempo_local = tiempo - tiempo_inicio

    noise_left  = random.uniform(-0.1, 0.1)
    noise_right = random.uniform(-0.1, 0.1)

    if pausa:
        motor_izq.setVelocity(0)
        motor_der.setVelocity(0)
        if tiempo_local > duracion_pausa:
            pausa = False
            tiempo_inicio = tiempo
    else:
        nombre, duracion = acciones[indice]
        v_izq, v_der = ejecutar_accion(nombre, tiempo_local)

        motor_izq.setVelocity(v_izq + noise_left)
        motor_der.setVelocity(v_der + noise_right)

        if tiempo_local > duracion:
            indice = min(indice + 1, len(acciones) - 1)
            pausa = True
            tiempo_inicio = tiempo
```

</details>

---

## Simulaciones de movimiento

1. ¿Que ocurre cuando ambas ruedas tienen la misma velocidad?
   
   Como hemos podido observar en las simulaciones, cuando el robot tiene la misma velocidad **Vl = Vr** el robot se moviliza en linea recta.
   ### Recto
   ![Linea Recta](videos/linearecta.gif)
   
2. ¿Como cambia la trayectoria cuando las velocidades son diferentes?
   
   Segun las simulaciones, cuando hay una diferencia de velocidades en las ruedas, el robot curva su movimiento hacia el lado de la rueda con menor velocidad.
   ### Curva
   ![Curva](videos/curva.gif)

3. ¿Que ocurre cuando una rueda gira en sentido opuesto a la otra?

   El robot rota sobre su propio eje (en el lugar). Cuando **vr = -vl** la velocidad lineal v = (vr + vl) / 2 = 0, pero ω = (vr − vl) / L ≠ 0, lo que genera una rotación sin desplazamiento.
   ### Giro
   ![Giro su eje](videos/girosueje.gif)

   
4. ¿Que tipo de movimiento permite dibujar un cırculo?

   Se necesita que vr ≠ vl pero ambas positivas (o negativas) y constantes. Esto genera una ω constante y una v constante, resultando en una trayectoria circular de radio.
   Mientras más parecidas sean las velocidades, mayor será el radio del círculo.
   ### Círculo
   ![Circulo](videos/circulo.gif)
