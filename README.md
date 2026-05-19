# Laboratorio 2 de Robótica: Control de Movimientos en Webots

Implementar un sistema básico de navegación reactiva en Webots para un
robot móvil diferencial, utilizando sensores de distancia y encoders de rueda,
aplicando filtrado sobre las mediciones y empleando un filtro de Kalman para
estimar la distancia frontal a obstáculos y mejorar la toma de decisiones.

# Estado del Proyecto - Entrega Laboratorio 2

## 1. Lista de Control de Actividades / Lista di Controllo delle Attività (Checklist)

### 🇪🇸 Español
* [cite_start][x] **Inicialización:** Configurar motores, sensores de distancia y encoders.
* [cite_start][x] **Filtro simple:** Aplicar un filtro simple sobre los sensores frontales. 
* [cite_start][x] **Estimación de avance:** Calcular el desplazamiento del robot usando encoders.
* [cite_start][ ] **Almacenamiento de datos:** Registrar y guardar lecturas crudas y filtradas para los gráficos. 
* [cite_start][ ] **Filtro de Kalman:** Implementar las etapas de predicción y corrección. 
* [cite_start][ ] **Navegación Reactiva:** Diseñar las reglas de decisión para avanzar o esquivar. 
* [cite_start][ ] **Giro con sensores laterales:** Decidir la dirección del giro según la proximidad del obstáculo.
* [cite_start][ ] **Escenarios de prueba:** Diseñar los dos entornos en Webots (simple y complejo). 

### 🇮🇹 Italiano (Stacce)
* [cite_start][x] **Inizializzazione:** Configurare motori, sensori di distanza ed encoder.
* [cite_start][x] **Filtro semplice:** Applicare un filtro semplice sui sensori frontali. 
* [cite_start][x] **Stima dell'avanzamento:** Calcolare lo spostamento del robot usando gli encoder. 
* [cite_start][ ] **Salvataggio dei dati:** Registrare e salvare le letture grezze e filtrate per i grafici. 
* [cite_start][ ] **Filtro di Kalman:** Implementare le fasi di predizione e correzione. 
* [cite_start][ ] **Navigazione Reattiva:** Progettare le regole decisionali per avanzare o schivare. 
* [cite_start][ ] **Svolta con sensori laterali:** Decidere la direzione della svolta in base alla vicinanza dell'ostacolo. 
* [cite_start][ ] **Scenari di prova:** Progettare i due ambienti su Webots (semplice e complesso).

---

## 2. Notas de Integrantes / Note dei Membri (Actualizado: Lun 18 de Mayo)

### 🇪🇸 Español

* **María:** * *Aporte:* Dejó lista la base del robot. Configuró los motores libres, activó los encoders y seleccionó 4 sensores clave (`ps0`, `ps7` al frente; `ps2`, `ps5` a los lados). También programó la conversión matemática de las ruedas a metros y un filtro alpha para limpiar el ruido frontal.
  * *Qué le falta:* El robot se mueve por tiempos preprogramados (rutinas ciegas). Falta conectar sus avances con la lógica de sensores que usará el resto del equipo.
* **Ariel:** *(Pendiente)*
* **Ignacia (Filtro Kalman) :** *(Pendiente)*
* **Shon:** *(Pendiente)*
* **Diego:** *(Pendiente)*

### 🇮🇹 Italiano

* **María:** * *Contributo:* Ha preparato la base del robot. Ha configurato i motori liberi, attivato gli encoder e selezionato 4 sensori chiave (`ps0`, `ps7` frontali; `ps2`, `ps5` laterali). Ha inoltre programmato la conversione matematica delle ruote in metri e un filtro alpha per pulire il rumore frontale.
  * *Cosa manca:* Il robot si muove in base a tempi preprogrammati (routine cieche). Manca il collegamento dei suoi progressi con la logica dei sensori che userà il resto del team.
* **Ariel:** *(In attesa)*
* **Ignacia (Filtro Kalman) :** *(In attesa)*
* **Shon:** *(In attesa)*
* **Diego:** *(In attesa)*
