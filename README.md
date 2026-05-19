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
