# Laboratorio de Robótica: Control de Movimientos en Webots

Este repositorio contiene la implementación de controladores para un robot móvil diferencial en el simulador **Webots**. El objetivo es comprender el comportamiento cinemático de un sistema diferencial mediante una simulación interactiva donde los actuadores (motores de las ruedas) controlan la trayectoria del robot.

## Fundamentos Cinemáticos
El movimiento del robot se rige por las ecuaciones de la cinemática diferencial. Siendo $r$ el radio de las ruedas, $L$ la distancia entre ellas, y $V_l, V_r$ las velocidades angulares de las ruedas izquierda y derecha:

* **Velocidad Lineal ($v$):** $v = \frac{r(V_r + V_l)}{2}$
* **Velocidad Angular ($\omega$):** $\omega = \frac{r(V_r - V_l)}{L}$

---

## Estructura del Proyecto
```text
├── controllers/
│   └── robot_controller/
│       └── robot_controller.py  # Lógica de control en Python
├── worlds/
│   └── robot_world.wbt          # Escena y configuración del mundo
├── videos/                      # Gifs demostrativos de las trayectorias
└── README.md

## Contenido de la simulacion. 
El proyecto incluye la simulación de las siguientes trayectorias:
1. **Línea Recta:** Desplazamiento uniforme hacia adelante ($V_l = V_r$).
2. **Curva:** Movimiento con distinto radio de giro ($V_l \neq V_r$)..
3. **Giro sobre su propio eje:** Rotación estática (velocidades opuestas en ruedas) ($V_l = -V_r$).
4. **Círculo:** Trayectoria circular continua ($V_l, V_r$ constantes y distintos de cero).

## Requisitos
* **Simulador:** [Webots R2023b](https://cyberbotics.com/) o superior.
* **Lenguaje:** Python 3.14
* **Robot utilizado:** Prototipo de robot diferencial ("Robotito").

## Instrucciones de Uso
1. Clona este repositorio:
   ```bash git clone [https://github.com/Diegodsito/RobotWebot.git](https://github.com/Diegodsito/RobotWebot.git)
   
2.Abre Webots y carga el mundo ubicado en worlds/robot_world.wbt.

3.Asegúrate de que el controlador del robot esté vinculado a la carpeta controllers/robot_controller/.

3.Presiona el botón Play en el simulador.

## Conclusiones

1. ¿Que ocurre cuando ambas ruedas tienen la misma velocidad?
   
   Como hemos podido observar en las simulaciones, cuando el robot tiene la misma velocidad **Vl = Vr** el robot se moviliza en linea recta.
   ### Retto
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
   ### Circulo
   ![Circulo](videos/circulo.gif)
