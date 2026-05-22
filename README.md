# Laboratorio 2 de Robótica: Control de Movimientos en Webots

Implementar un sistema básico de navegación reactiva en Webots para un
robot móvil diferencial, utilizando sensores de distancia y encoders de rueda,
aplicando filtrado sobre las mediciones y empleando un filtro de Kalman para
estimar la distancia frontal a obstáculos y mejorar la toma de decisiones.

# Estado del Proyecto - Entrega Laboratorio 2

## 1. Lista de Control de Actividades / Lista di Controllo delle Attività (Checklist)

### 🇪🇸 Español
* [x] **Inicialización:** Configurar motores, sensores de distancia y encoders.
* [x] **Filtro simple:** Aplicar un filtro simple sobre los sensores frontales. 
* [x] **Estimación de avance:** Calcular el desplazamiento del robot usando encoders.
* [ ] **Almacenamiento de datos:** Registrar y guardar lecturas crudas y filtradas para los gráficos. 
* [x] **Filtro de Kalman:** Implementar las etapas de predicción y corrección. 
* [ ] **Navegación Reactiva:** Diseñar las reglas de decisión para avanzar o esquivar. 
* [ ] **Giro con sensores laterales:** Decidir la dirección del giro según la proximidad del obstáculo.
* [ ] **Escenarios de prueba:** Diseñar los dos entornos en Webots (simple y complejo). 

### 🇮🇹 Italiano (Stacce)
* [x] **Inizializzazione:** Configurare motori, sensori di distanza ed encoder.
* [x] **Filtro semplice:** Applicare un filtro semplice sui sensori frontali. 
* [x] **Stima dell'avanzamento:** Calcolare lo spostamento del robot usando gli encoder. 
* [ ] **Salvataggio dei dati:** Registrare e salvare le letture grezze e filtrate per i grafici. 
* [x] **Filtro di Kalman:** Implementare le fasi di predizione e correzione. 
* [ ] **Navigazione Reattiva:** Progettare le regole decisionali per avanzare o schivare. 
* [ ] **Svolta con sensori laterali:** Decidere la direzione della svolta in base alla vicinanza dell'ostacolo. 
* [ ] **Scenari di prova:** Progettare i due ambienti su Webots (semplice e complesso).

---

## 2. Notas de Integrantes / Note dei Membri (Actualizado: Lun 18 de Mayo)

### 🇪🇸 Español

* **María (Sensor y Logica Navegación) :** * *Aporte:* Dejó lista la base del robot. Configuró los motores libres, activó los encoders y seleccionó 4 sensores clave (`ps0`, `ps7` al frente; `ps2`, `ps5` a los lados). También programó la conversión matemática de las ruedas a metros y un filtro alpha para limpiar el ruido frontal.
  * *Qué le falta:* El robot se mueve por tiempos preprogramados (rutinas ciegas). Falta conectar sus avances con la lógica de sensores que usará el resto del equipo.
* **Ariel:** *(Pendiente)*
* **Ignacia (Filtro Kalman):** *Aporte:* Diseñó e integró el algoritmo del Filtro de Kalman de 1 dimensión en el controlador principal, definiendo las variables de incertidumbre (`Q`, `R`, `P` y `d_est`) y programando las etapas de Predicción y Corrección. Además, desarrolló e integró la función de mapeo `sensor_a_metros` para resolver la no-linealidad de los sensores infrarrojos, logrando que el filtro reciba lecturas físicamente coherentes en metros.
  * *Qué le falta:* Sintonizar los valores de ruido (`Q` y `R`) haciendo pruebas directas en la simulación, y utilizar la distancia final estimada (`d_est`) para construir la lógica de navegación reactiva. (Maria)
* **Shon:** *(Pendiente)*
* **Diego:** *(Pendiente)*

### 🇮🇹 Italiano

* **María (Sensori e Logica di Navigazione):** * *Contributo:* Ha preparato la base del robot. Ha configurato i motori liberi, attivato gli encoder e selezionato 4 sensori chiave (`ps0`, `ps7` frontali; `ps2`, `ps5` laterali). Ha inoltre programmato la conversione matematica delle ruote in metri e un filtro alpha per pulire il rumore frontale.
  * *Cosa manca:* Il robot si muove in base a tempi preprogrammati (routine cieche). Manca il collegamento dei suoi progressi con la logica dei sensori che userà il resto del team.
* **Ariel:** *(In attesa)*
* **Ignacia (Filtro Kalman):** *Contributo:* Ha progettato e integrato l'algoritmo del Filtro di Kalman a 1 dimensione nel controller principale, definendo le variabili di incertezza (`Q`, `R`, `P` e `d_est`) e programmando le fasi di Predizione e Correzione. Inoltre, ha sviluppato e integrato la funzione di mappatura `sensor_a_metros` per risolvere la non-linearità dei sensori a infrarossi, consentendo al filtro di ricevere letture fisicamente coerenti in metri.
* 
  * *Cosa manca:* Calibrare i valori del rumore (`Q` e `R`) effettuando test diretti nella simulazione, e utilizzare la distanza finale stimata (`d_est`) per costruire la logica di navigazione reattiva. (Maria)
* **Shon:** *(In attesa)*
* **Diego:** *(In attesa)*

---

## 3. Descripción del robot y sensores

**Robot:** e-puck (diferencial de dos ruedas)
- Radio de rueda: `r = 0.0205 m`

**Sensores de distancia utilizados:**

| Sensor | Posición |
|--------|----------|
| ps0, ps1 | Frontales derechos |
| ps6, ps7 | Frontales izquierdos |
| ps2 | Lateral izquierdo |
| ps5 | Lateral derecho |

**Encoders:** `left wheel sensor` y `right wheel sensor` — entregan posición angular en radianes.

---

## 4. Frecuencia de muestreo

El controlador se ejecuta cada `basicTimeStep = 16 ms`:

$$T_s = 0.016 \text{ s} \qquad f_s = 62.5 \text{ Hz}$$

---

## 5. Análisis de señales registradas

Los sensores infrarrojos entregan valores crudos en escala no lineal. Se aplica la conversión:

$$d = \frac{1.16}{\sqrt{v_{crudo}}} \quad (v_{crudo} \geq 65), \quad d \in [0.01,\ 0.15] \text{ m}$$

Si `v_crudo < 65` (sin obstáculo detectable) se retorna `0.15 m`. La señal cruda presenta ruido considerable, especialmente en giros y superficies anguladas.

---

## 6. Estimación del avance mediante encoders

Los encoders entregan desplazamiento angular $\theta$ (rad). El avance lineal es:

$$s = r \cdot \theta$$

El avance promedio del robot entre dos instantes consecutivos:

$$\Delta d_k = \frac{s_{izq} + s_{der}}{2}$$

Este valor alimenta la etapa de predicción del filtro de Kalman.

---

## 7. Filtro simple aplicado

Filtro exponencial de paso bajo sobre la lectura frontal máxima convertida a metros:

$$z_k = \alpha \cdot z_{k,\text{crudo}} + (1 - \alpha) \cdot z_{k-1} \qquad \alpha = 0.25$$

Suaviza el ruido de alta frecuencia con un retardo moderado.

---

## 8. Implementación del filtro de Kalman

### Etapa de predicción

$$\hat{d}_k^- = \hat{d}_{k-1} - \Delta d_k \qquad P_k^- = P_{k-1} + Q \qquad (Q = 0.005)$$

### Etapa de corrección

$$K_k = \frac{P_k^-}{P_k^- + R} \qquad (R = 0.07)$$

$$\hat{d}_k = \hat{d}_k^- + K_k\left(z_{k,\text{crudo}} - \hat{d}_k^-\right)$$

$$P_k = (1 - K_k)\cdot P_k^-$$

La corrección usa `z_k_crudo` (medición directa del sensor, ruidosa) — independiente del filtro simple. $K_k$ pondera automáticamente cuánto confiar en la predicción vs. la medición.

---

## 9. Lógica de navegación reactiva

El robot opera con tres estados basados en `d_est` (estimación Kalman):

| Estado | Condición de entrada | Acción |
|--------|---------------------|--------|
| `FORWARD` | `d_est > 0.133 m` | Avanza a 2.0 rad/s |
| `FRENANDO` | `d_est < 0.128 m` | Frena por 240 ms (15 pasos × 16 ms) |
| `AVOID` | Fin del frenado | Gira en el lugar a 1.2 rad/s |

**Dirección del giro** (sensores laterales):
- `max(ps0, ps1) > max(ps6, ps7)` → obstáculo por la derecha → gira izquierda
- caso contrario → obstáculo por la izquierda → gira derecha

---

## 10. Gráficos de señales

> Generados corriendo cada simulación y luego ejecutando `python graficar.py simple` o `python graficar.py complejo` desde la raíz del repo.

### Escenario simple
![Señales escenario simple](docs/grafico_simple.png)

### Escenario complejo
![Señales escenario complejo](docs/grafico_complejo.png)

---

## 11. Resultados en escenarios de prueba

### Escenario simple (`escenario_simple.wbt`)
Arena 2.5×2.5 m, 3 obstáculos aislados. Robot parte desde (-0.9, 0) mirando +x.

Permite observar el comportamiento base del filtro Kalman con pocos eventos de esquive.

### Escenario complejo (`escenario_complejo.wbt`)
Pasillo en L de 0.30 m de ancho + 3 obstáculos dispersos.

El robot recorre el tramo horizontal, detecta el fondo del pasillo y gira para recorrer el tramo vertical. Los sensores laterales detectan las paredes continuamente, lo que permite comparar claramente la señal cruda (ruidosa) contra la estimación Kalman (estable).

---

## 12. Análisis y conclusiones

- El **filtro simple** reduce ruido de alta frecuencia pero introduce retardo ante cambios bruscos.
- El **filtro de Kalman** combina odometría y sensor, produciendo una estimación más estable y con menor retardo cuando el robot avanza rectamente.
- Usar `d_est` en lugar de lecturas crudas reduce los **giros innecesarios** causados por picos de ruido.
- En pasillos estrechos, la ganancia $K_k$ sube porque la odometría acumula error en los giros, pasando a confiar más en el sensor.

---

## 13. Instrucciones para ejecutar

**Requisitos:** Webots R2025a, Python 3 con `matplotlib`

1. Abrir Webots: `File → Open World` y seleccionar el `.wbt` deseado.
2. Correr la simulación (▶). Se genera `docs/datos_simulacion.csv` automáticamente.
3. Detener la simulación y desde la raíz del repo ejecutar:
   ```bash
   python graficar.py
   ```
   Genera `docs/grafico_senales.png`.
