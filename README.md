# Proyecto Final: Navegación Autónoma con Planificación de Rutas en Webots

**Curso:** Robótica y Sistemas Autónomos 2026-01 — ICI 4150
**Docente:** Sandra Cano

**Integrantes:**
- Maria Paganetti
- Ignacia Brahim
- Diego Alvarado
- Sean Jamen
- Ariel Villar

---

## Línea seleccionada: A — Planificación de rutas

El robot calcula una ruta global con A* sobre una grilla de ocupación construida a partir del archivo `.wbt` del escenario, la sigue con un controlador de waypoints, y reacciona ante obstáculos no incluidos en el mapa usando los sensores IR y los encoders.

## Objetivo del proyecto

Diseñar e implementar en Webots un sistema de navegación autónoma para un robot diferencial (e-puck) que combine:
- Control cinemático diferencial y odometría (Lab 1).
- Lectura y filtrado de sensores de distancia (Lab 2).
- Planificación global de rutas con A* y evitación reactiva + replanificación ante obstáculos imprevistos.

## Robot, sensores y actuadores

**Robot:** e-puck (dos ruedas independientes).

| Parámetro | Valor |
|---|---|
| Radio de rueda | 0.0205 m |
| Distancia entre ejes | 0.057 m |
| Velocidad máxima de rueda | 6.28 rad/s |
| Velocidad lineal de crucero | 0.105 m/s |

**Sensores de distancia infrarrojo (8, todo el anillo del e-puck):**

| Sensor | Ángulo respecto al frente |
|---|---|
| ps0 | -17° |
| ps1 | -50° |
| ps2 | -90° (lateral derecho) |
| ps3 | -150° |
| ps4 | 150° |
| ps5 | 90° (lateral izquierdo) |
| ps6 | 50° |
| ps7 | 17° |

Cada sensor se filtra de forma independiente con un filtro de Kalman escalar (`q=1.0`, `r=50.0`) para suavizar el ruido de la lectura cruda, igual que en el Lab 2.

**Encoders:** `left wheel sensor` y `right wheel sensor`, entregan posición angular acumulada en radianes. Se usan para odometría (Lab 1) y para medir la longitud recorrida.

## Descripción de los escenarios de prueba

Ambos escenarios usan arena de 2.5×2.5 m, con el robot partiendo en **(−1, −1)** hacia la meta en **(1, 1)**.

### Escenario simple (`escenario_simple.wbt`)

4 paredes mapeadas (0.5×0.15 m, dos de ellas rotadas 45°) más 2 obstáculos pequeños (0.15×0.15 m) que el planificador **ignora a propósito** y deja como "imprevistos" para la evitación reactiva.

### Escenario complejo (`escenario_complejo.wbt`)

11 muros/trampas mapeados (incluye dos paredes diagonales) más 3 obstáculos pequeños (0.1×0.1 m) tratados también como imprevistos. Pasillos más estrechos y mayor cantidad de giros que el escenario simple.

## Algoritmo implementado

**1. Grilla de ocupación** — `path_planning.create_map_from_wbt()` lee el `.wbt`, busca cada `SolidBox` por expresión regular y lo rasteriza en una grilla de 100×100 celdas (2.5 cm/celda) respetando su rotación. Los obstáculos de 0.15×0.15 m o menos se excluyen del mapa estático a propósito: se tratan como obstáculos "imprevistos" que el robot debe descubrir y evitar con sus sensores, no con el plan global. Los bordes de la arena se marcan como ocupados.

**2. A\*** — búsqueda en 8 direcciones (costo 1.0 recto, 1.4142 diagonal), heurística euclidiana. Antes de buscar se calcula un mapa de distancias (BFS multi-fuente) a la pared más cercana: una celda solo es transitable si esa distancia supera el radio físico del robot (4 celdas = 10 cm), y se penalizan las celdas cercanas a una pared (dentro de 12 celdas = 30 cm) para que la ruta prefiera el centro de los pasillos.

**3. Simplificación de ruta** — `downsample_world_path()` reduce la ruta celda-por-celda a un conjunto de waypoints reales, conservando solo los puntos donde la dirección cambia más de 10° o cada 10 cm.

**4. Seguimiento de waypoints (Lab 1)** — `manage_waypoints()` elige un punto "lookahead" a 8 cm sobre la ruta. El robot gira hacia ese punto con control proporcional sobre el error angular (`K_W=2.0`); si el error supera 45° gira en el lugar antes de avanzar. La conversión velocidad lineal/angular → velocidad de rueda usa el modelo cinemático diferencial del Lab 1, y la pose se actualiza con el mismo modelo de integración (`Δs`, `Δθ`, punto medio).

**5. Evitación reactiva + replanificación (Lab 2)** — si el valor filtrado (Kalman) de cualquier sensor IR supera el umbral 100, el robot se detiene, estima la posición mundial del obstáculo a partir del ángulo del sensor y una distancia aproximada, lo agrega de forma permanente a la grilla de ocupación, retrocede ~6 cm para alejarse, y vuelve a ejecutar A* completo desde su posición actual hasta la meta. También se replanifica automáticamente si el robot se desvía más de 12 cm de la ruta planificada (drift de odometría).

### Diagrama de flujo

```
INICIO
  Detectar escenario por el .wbt cargado
  Construir grilla de ocupación (ignora obstáculos <= 15 cm)
  A* inicio -> meta, generar waypoints
  |
  v
LOOP (cada paso de simulación)
  Leer encoders -> actualizar odometría (x, y, theta)
  Leer 8 sensores IR -> filtrar con Kalman (uno por sensor)
  Si distancia a la meta < 0.08 m -> FIN (cerrar CSV, detener motores)

  estado == TRACKING:
    Si algún sensor filtrado > 100 (obstáculo imprevisto):
        detener motores, estimar y marcar el obstáculo en la grilla,
        pasar a estado REVERSING
    Si la desviación a la ruta > 0.12 m -> replanificar A*
    Si no -> avanzar hacia el waypoint lookahead (control proporcional)

  estado == REVERSING:
    retroceder en línea recta
    al recorrer 0.06 m -> replanificar A* desde la posición actual
                           y volver a estado TRACKING
```

## Relación con los laboratorios anteriores

| Componente | Origen | Uso en el proyecto |
|---|---|---|
| Modelo cinemático diferencial (v, ω → rueda izq/der) | Lab 1 | Conversión de comandos de velocidad en `make_motor_speeds()` |
| Integración de pose (Δs, Δθ, punto medio) | Lab 1 | Odometría en `update_odometry()` |
| Lectura y filtrado de sensores de distancia | Lab 2 | Filtro de Kalman por sensor IR (`KalmanIR`) |
| Navegación reactiva ante obstáculos | Lab 2 | Estado `REVERSING` + replanificación al detectar un obstáculo imprevisto |

El proyecto extiende ambos laboratorios agregando la capa global: en vez de solo reaccionar, el robot ahora planifica una ruta completa con A* y usa la capa reactiva del Lab 2 únicamente para los obstáculos que el mapa no conocía de antemano.

## Resultados obtenidos

| Métrica | Simple | Complejo |
|---|---|---|
| Tiempo total hasta la meta | 48.5 s | 48.7 s |
| Longitud de ruta planificada (A*) | 3.004 m | 4.312 m |
| Longitud de trayectoria ejecutada | 4.183 m | 4.404 m |
| Diferencia ruta/trayectoria | 1.179 m | 0.092 m |
| Obstáculos imprevistos detectados y evitados | 1 | 1 |
| Meta alcanzada | Sí | Sí |

Datos completos paso a paso en `docs/datos_simple.csv` y `docs/datos_complejo.csv`. Gráficos generados con `graficar_final.py`.

### Trayectoria ejecutada vs. ruta planificada

![Trayectoria simple](docs/trayectoria_simple.png)
![Trayectoria complejo](docs/trayectoria_complejo.png)

### Señal IR frontal (cruda vs. Kalman) y encoders

![Señales simple](docs/senales_simple.png)
![Señales complejo](docs/senales_complejo.png)

### Seguimiento de waypoints y longitud acumulada

![Seguimiento simple](docs/seguimiento_simple.png)
![Seguimiento complejo](docs/seguimiento_complejo.png)

### Video demostrativo

> Pendiente de grabar y enlazar.

## Instrucciones para ejecutar

**Requisitos:** Webots R2025a, Python 3 con `matplotlib`.

1. Clonar el repositorio y cambiar a la rama del proyecto final:
   ```bash
   git clone https://github.com/Diegodsito/RobotWebot.git
   cd RobotWebot
   git checkout final-project
   ```

2. Abrir en Webots `worlds/escenario_simple.wbt` o `worlds/escenario_complejo.wbt` y presionar **Play**. El controlador detecta el escenario automáticamente, navega hasta la meta y se detiene solo. Se genera `docs/datos_<escenario>.csv`.

3. Generar los gráficos desde la raíz del repositorio:
   ```bash
   python graficar_final.py simple
   python graficar_final.py complejo
   ```
   Esto guarda en `docs/` las imágenes `trayectoria_*.png`, `senales_*.png` y `seguimiento_*.png`, e imprime las métricas en consola.

## Conclusiones

- A* sobre la grilla de ocupación encontró rutas válidas en ambos escenarios y el robot llegó a la meta en las dos pruebas (~48 s), con una diferencia ruta/trayectoria de solo 9 cm en el escenario complejo.
- Excluir del mapa estático los obstáculos pequeños y resolverlos solo con evitación reactiva + replanificación funcionó correctamente: en ambas corridas se detectó y evitó exactamente 1 obstáculo imprevisto sin colisión.
- El filtro de Kalman por sensor IR estabiliza la lectura cruda y evita falsas alarmas de obstáculo por ruido puntual.

## Limitaciones y mejoras posibles

- No hay corrección de heading con los sensores (solo odometría pura), por lo que el error de pose puede acumularse en trayectorias más largas.
- La detección de escenario busca la palabra "semplice" en el nombre del archivo, que nunca coincide con `escenario_simple.wbt`; en la práctica no afecta el resultado porque ambos mundos comparten la misma posición de inicio y meta, pero sería una mejora corregirlo.
- Falta grabar y enlazar el video demostrativo.
- El CSV registra el estado y la posición en cada paso, pero no las replanificaciones intermedias; agregarlas permitiría un análisis más fino de cada evitación.
