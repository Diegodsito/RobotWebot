# Laboratorio 2: Navegación reactiva con filtrado y fusión de sensores en Webots

**Curso:** Robótica y Sistemas Autónomos 2026-01 — ICI 4150

**Integrantes:**
- Maria Paganetti
- Ignacia Brahim
- Diego Alvarado
- Sean Jamen
- Ariel Villar

---

## Objetivo

Implementar un sistema de navegación reactiva en Webots para un robot móvil diferencial, utilizando sensores de distancia y encoders de rueda, aplicando filtrado sobre las mediciones y empleando un filtro de Kalman para estimar la distancia frontal a obstáculos y mejorar la toma de decisiones.

---

## Robot y sensores utilizados

**Robot:** e-puck (diferencial de dos ruedas independientes)
- Radio de rueda: `r = 0.0205 m`
- Diámetro del robot: `~7.4 cm`

**Sensores de distancia (infrarrojo):**

| Sensor | Posición |
|--------|----------|
| ps0, ps1 | Frontales derechos |
| ps6, ps7 | Frontales izquierdos |
| ps2 | Lateral izquierdo |
| ps5 | Lateral derecho |

**Encoders:** `left wheel sensor` y `right wheel sensor` — entregan posición angular acumulada en radianes.

---

## Frecuencia de muestreo

El controlador se ejecuta cada paso de simulación definido por `basicTimeStep`:

$$T_s = 16 \text{ ms} \qquad f_s = \frac{1}{T_s} = 62.5 \text{ Hz}$$

Todas las señales (crudas, filtradas y estimadas) se registran a esta frecuencia.

---

## Análisis de señales registradas

Los sensores infrarrojos del e-puck entregan valores crudos en escala no lineal (0–4095). Para obtener distancias en metros se aplica la función de mapeo:

$$d = \frac{1.16}{\sqrt{v_{crudo}}} \quad \text{con } v_{crudo} \geq 65, \quad d \in [0.01,\ 0.15] \text{ m}$$

Si `v_crudo < 65` (sin obstáculo detectable) se retorna el valor máximo `0.15 m`. La señal cruda presenta ruido considerable, especialmente durante giros y cuando el robot enfrenta superficies en ángulo.

---

## Estimación del avance mediante encoders

Los encoders entregan desplazamiento angular $\theta$ en radianes. El avance lineal de cada rueda se calcula como:

$$s = r \cdot \theta$$

El avance promedio del robot entre dos instantes consecutivos:

$$\Delta d_k = \frac{s_{izq} + s_{der}}{2}$$

Este valor alimenta la etapa de predicción del filtro de Kalman.

---

## Filtro simple aplicado

Se aplica un filtro exponencial de paso bajo sobre la lectura frontal máxima convertida a metros:

$$z_k = \alpha \cdot z_{k,\text{crudo}} + (1 - \alpha) \cdot z_{k-1} \qquad \alpha = 0.25$$

Un $\alpha$ bajo suaviza más el ruido pero introduce mayor retardo ante cambios bruscos de distancia.

---

## Implementación del filtro de Kalman

El filtro de Kalman estima la distancia frontal $\hat{d}_k$ combinando la predicción por odometría con la medición directa del sensor.

### Etapa de predicción

$$\hat{d}_k^- = \hat{d}_{k-1} - \Delta d_k \qquad P_k^- = P_{k-1} + Q \qquad (Q = 0.005)$$

### Etapa de corrección

$$K_k = \frac{P_k^-}{P_k^- + R} \qquad (R = 0.07)$$

$$\hat{d}_k = \hat{d}_k^- + K_k\left(z_{k,\text{crudo}} - \hat{d}_k^-\right)$$

$$P_k = (1 - K_k)\cdot P_k^-$$

La corrección usa $z_{k,\text{crudo}}$ (medición directa del sensor, ruidosa), independiente del filtro simple. La ganancia $K_k$ pondera automáticamente cuánto confiar en la predicción versus la medición en cada instante: si $R$ es grande, el filtro confía más en la predicción; si $P_k^-$ es grande, confía más en el sensor.

---

## Lógica de navegación reactiva

El robot opera con tres estados basados en `d_est` (estimación Kalman):

| Estado | Condición de entrada | Acción |
|--------|---------------------|--------|
| `FORWARD` | `d_est > 0.133 m` | Avanza a 2.0 rad/s |
| `FRENANDO` | `d_est < 0.128 m` | Detiene motores por 240 ms (15 pasos × 16 ms) |
| `AVOID` | Fin del frenado | Gira en el lugar a 1.2 rad/s |

**Dirección del giro** (decidida con sensores laterales):
- `max(ps0, ps1) > max(ps6, ps7)` → obstáculo más próximo por la derecha → gira a la izquierda
- caso contrario → obstáculo más próximo por la izquierda → gira a la derecha

---

## Gráficos de señales

### Escenario simple
![Señales escenario simple](docs/grafico_simple.png)

### Escenario complejo
![Señales escenario complejo](docs/grafico_complejo.png)

---

## Escenarios de prueba

### Escenario 1 — Simple (`escenario_simple.wbt`)

Arena de 2.5×2.5 m con 3 obstáculos aislados. El robot parte desde (-0.9, 0) mirando hacia +x.

- Obstáculo central en (0, 0)
- Obstáculo superior en (0.7, 0.5)
- Obstáculo inferior en (0.5, -0.7)

**Estabilidad del movimiento:** el robot avanza de forma continua y estable durante la mayor parte de la simulación. Al haber pocos obstáculos y espacio amplio, los cambios de dirección son infrecuentes.

**Giros innecesarios:** se registra un único evento de esquive (~segundo 20). La estimación Kalman amortigua los picos de ruido, evitando activaciones falsas del estado `AVOID` que sí podrían ocurrir con la señal cruda.

**Capacidad para evitar colisiones:** el robot detecta el obstáculo con suficiente anticipación gracias a `d_est`, frena correctamente y gira sin colisionar.

**Diferencias entre señales:** en el gráfico se aprecia que la señal cruda (rojo) cae abruptamente y con mayor ruido que el filtro simple (naranja). El Kalman (azul) desciende de forma más gradual, reflejo de que combina la inercia de la predicción odométrica con la medición del sensor.

### Escenario 2 — Complejo (`escenario_complejo.wbt`)

Pasillo en L de 0.30 m de ancho formado por 4 paredes, más 3 obstáculos dispersos. El robot debe navegar por el tramo horizontal, detectar el fondo del pasillo y girar para recorrer el tramo vertical.

**Estabilidad del movimiento:** el movimiento es menos uniforme que en el escenario simple. Dentro del pasillo, los sensores laterales detectan las paredes continuamente, generando variaciones en las señales que podrían desestabilizar una navegación basada solo en lecturas crudas.

**Giros innecesarios:** se observan dos eventos de esquive (~segundos 30 y 42). El Kalman evita reacciones prematuras ante el ruido que genera la proximidad de las paredes del pasillo, reduciendo giros falsos respecto a lo que produciría la señal cruda.

**Capacidad para evitar colisiones:** el robot navega el pasillo completo y esquiva los obstáculos dispersos sin colisionar. La combinación de sensores laterales y `d_est` permite decidir la dirección de giro correctamente en cada evento.

**Diferencias entre señales:** la divergencia entre los encoders izquierdo y derecho es claramente visible en el gráfico inferior, evidenciando los giros realizados. En los valles de distancia, la señal cruda cae de forma más brusca e irregular que el Kalman, lo que confirma que la fusión sensorial entrega una estimación más confiable en entornos de alta densidad de obstáculos.

---

## Análisis y conclusiones

- El **filtro simple** reduce el ruido de alta frecuencia pero introduce retardo ante cambios bruscos, lo que puede provocar reacciones tardías frente a obstáculos próximos.
- El **filtro de Kalman** combina la predicción odométrica con la medición del sensor, produciendo una estimación más estable y con menor retardo cuando el robot avanza en línea recta.
- Usar `d_est` en lugar de lecturas crudas para las decisiones de navegación reduce los giros innecesarios causados por picos de ruido transitorio.
- En el escenario complejo, los encoders divergen visiblemente durante los giros dentro del pasillo, lo que aumenta la incertidumbre de la predicción y hace que la ganancia $K_k$ suba, confiando más en el sensor.

---

## Instrucciones para ejecutar

**Requisitos:** Webots R2025a, Python 3 con `matplotlib`

1. Abrir Webots: `File → Open World` y seleccionar el escenario deseado:
   - `worlds/escenario_simple.wbt`
   - `worlds/escenario_complejo.wbt`

2. Correr la simulación (▶). Se genera `docs/datos_simulacion.csv` automáticamente.

3. Detener la simulación y desde la raíz del repositorio ejecutar:
   ```bash
   python graficar.py simple
   # o
   python graficar.py complejo
   ```
   Genera el PNG correspondiente en `docs/`.
