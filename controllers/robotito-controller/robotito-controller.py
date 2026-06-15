"""
robotito_controller_v2.py - Controller principale e-puck (Webots).

Architettura modulare:
  perception   -> sensori IR (ostacoli + correzione theta opzionale)
  localization -> odometria encoder (stima posa)
  planning     -> A* + smoothing (percorso globale)
  control      -> Pure Pursuit (inseguimento morbido)

Vincoli: Python puro, niente EKF, niente supervisor.
Richiede path_planning.py nella stessa cartella (l'A* originale).
"""

from controller import Robot
import math
import csv
import os

from perception import Perception
from localization import Localization
from planning import Planner
from control import PurePursuitController

# ----------------------------------------------------------------- parametri
ESCENARIO = "complejo"
CONFIG = {
    "simple":   {"start": (-0.9, 0.0),  "goal": (0.9, 0.0)},
    "complejo": {"start": (-1.0, -1.0), "goal": (1.0, 1.0)},
}

WHEEL_RADIUS = 0.0205     # [m] verifica col tuo .wbt / proto e-puck
AXLE_LENGTH = 0.052       # [m] idem: questi due valori scalano l'odometria
MAX_SPEED = 6.28          # [rad/s]

# IMPORTANTE: theta iniziale = orientamento REALE del robot nel .wbt.
# Deve coincidere con il campo 'rotation' del file mondo, altrimenti il robot
# parte con una stima sbagliata e sterza nella direzione errata.
INITIAL_THETA = 0.0       # <-- METTI QUI lo stesso heading del .wbt

# Correzione IR di theta: in simulazione pura di solito NON serve (encoder
# quasi perfetti). Attivala se accendi rumore/slittamento ruota.
USE_IR_HEADING_CORRECTION = False
IR_HEADING_GAIN = 0.05

# Replan solo se il robot e' bloccato (mappa statica: replan periodico inutile).
REPLAN_IF_STUCK = True
STUCK_TIME_S = 3.0
PROGRESS_EPS = 0.02       # avanzamento minimo [m] per non considerarsi bloccato

# ----------------------------------------------------------------- setup
robot = Robot()
timestep = int(robot.getBasicTimeStep())

left_motor = robot.getDevice("left wheel motor")
right_motor = robot.getDevice("right wheel motor")
for m in (left_motor, right_motor):
    m.setPosition(float('inf'))
    m.setVelocity(0.0)

start_w = CONFIG[ESCENARIO]["start"]
goal_w = CONFIG[ESCENARIO]["goal"]

perception = Perception(robot, timestep)
loc = Localization(robot, timestep, WHEEL_RADIUS, AXLE_LENGTH,
                   start_pose=(start_w[0], start_w[1], INITIAL_THETA))
planner = Planner(ESCENARIO)
ctrl = PurePursuitController(WHEEL_RADIUS, AXLE_LENGTH, max_wheel_speed=MAX_SPEED)

waypoints = planner.plan(start_w, goal_w)
if not waypoints:
    print("[ERROR] Nessun percorso trovato.")
else:
    print(f"[Init] Percorso: {len(waypoints)} waypoint (dopo smoothing + resample).")

# logging
os.makedirs('../../docs', exist_ok=True)
csv_path = f'../../docs/datos_{ESCENARIO}.csv'
csv_file = open(csv_path, 'w', newline='')
writer = csv.writer(csv_file)
writer.writerow(['t', 'x', 'y', 'theta_deg', 'wp_restanti'])

# stato per il rilevamento "bloccato"
last_progress_t = 0.0
last_dist_goal = None

# ----------------------------------------------------------------- loop
while robot.step(timestep) != -1:

    # 1) LOCALIZZAZIONE (odometria)
    x, y, theta = loc.update()

    # 2) PERCEZIONE
    raw = perception.read_raw()
    if USE_IR_HEADING_CORRECTION:
        loc.correct_heading(perception.heading_correction(raw), IR_HEADING_GAIN)
        x, y, theta = loc.pose
    reactive_bias = perception.reactive_turn(raw)
    front_blocked, _left, _right = perception.obstacle_flags(raw)

    # 3) REPLAN solo se bloccato
    if REPLAN_IF_STUCK and waypoints:
        d = math.hypot(goal_w[0] - x, goal_w[1] - y)
        t = robot.getTime()
        if last_dist_goal is None or d < last_dist_goal - PROGRESS_EPS:
            last_dist_goal = d
            last_progress_t = t
        elif t - last_progress_t > STUCK_TIME_S:
            new_wps = planner.plan((x, y), goal_w)
            if new_wps:
                waypoints = new_wps
                print(f"[Replan] bloccato: nuovo percorso ({len(waypoints)} wp) "
                      f"da ({x:.2f}, {y:.2f})")
            last_progress_t = t

    # 4) CONTROLLO (Pure Pursuit)
    left_v, right_v, done = ctrl.compute(waypoints, (x, y, theta),
                                         reactive_bias, front_blocked)
    if done:
        left_motor.setVelocity(0.0)
        right_motor.setVelocity(0.0)
        print("[INFO] Meta raggiunta!")
        break
    left_motor.setVelocity(left_v)
    right_motor.setVelocity(right_v)

    # 5) LOG
    writer.writerow([robot.getTime(), x, y, math.degrees(theta), len(waypoints)])

csv_file.close()
print(f"[INFO] CSV salvato in {csv_path}")
