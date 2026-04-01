# Laboratorio de Robótica: Control de Movimientos en Webots

Este repositorio contiene la implementación de controladores para un robot móvil en el simulador **Webots**. El objetivo es vomprender el comportamiento cinematico de un robot movil diferencial mediante una simulacion interactiva en Webots, donde los actuadores
(motores de las ruedas) controlan el movimiento del robot.


## Contenido del Proyecto
El proyecto incluye la simulación de las siguientes trayectorias:
1. **Línea Recta:** Desplazamiento uniforme hacia adelante.
2. **Curva:** Movimiento con distinto radio de giro.
3. **Giro sobre su propio eje:** Rotación estática (velocidades opuestas en ruedas).
4. **Círculo:** Trayectoria circular continua.

## Requisitos
* **Simulador:** [Webots R2023b](https://cyberbotics.com/) o superior.
* **Lenguaje:** Python 3.x.
* **Robot utilizado:** [Nombre del robot].

## Instrucciones de Uso
1. Clona este repositorio:
   ```bash git clone [https://github.com/Diegodsito/RobotWebot.git](https://github.com/Diegodsito/RobotWebot.git)
   
2.Abre Webots y carga el mundo ubicado en worlds/robot_world.wbt.

3.Asegúrate de que el controlador del robot esté vinculado a la carpeta controllers/robot_controller/.

3.Presiona el botón Play en el simulador.

## Conclusiones

1. ¿Que ocurre cuando ambas ruedas tienen la misma velocidad?
   
   Como hemos podido observar en las simulaciones, cuando el robot tiene la misma velocidad **Vl = Vr** el robot se moviliza en linea recta.
   
2. ¿Como cambia la trayectoria cuando las velocidades son diferentes?
   
   Segun las simulaciones, cuando hay una diferencia de velocidades en las ruedas, los robots curvan su movimiento hacia el lado opuesto de la rueda con mayor velocidad.
   
3. ¿Que ocurre cuando una rueda gira en sentido opuesto a la otra?

   Cuando una ruedad gira en sentido contrario pero con la misma velocidad **Vl = -Vr** el robot gira en su mismo eje.

   
4. ¿Que tipo de movimiento permite dibujar un cırculo?
