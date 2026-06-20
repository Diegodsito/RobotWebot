"""
graficar_final.py — Gráficos del Proyecto Final: Path Planning + Navegación Autónoma
Proyecto Final ICI 4150 — Robótica y Sistemas Autónomos 2026-01

Genera 3 figuras a partir del CSV de la simulación (docs/datos_<escenario>.csv):
  1. Trayectoria estimada en el plano XY (con ruta planificada A* superpuesta
     y los obstáculos reales leídos del .wbt correspondiente)
  2. Señales del sensor IR frontal: crudo vs Kalman + encoders vs tiempo
  3. Distancia al waypoint activo vs tiempo + longitud ejecutada acumulada

Uso (desde la raíz del repositorio):
    python graficar_final.py simple
    python graficar_final.py complejo
"""

import csv
import os
import re
import sys
import math
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.transforms import Affine2D

REPO_ROOT = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(REPO_ROOT, 'controllers', 'robotito-controller'))
import path_planning

# =============================================================================
# ARGUMENTOS
# =============================================================================

escenario = sys.argv[1] if len(sys.argv) > 1 else "complejo"
csv_path  = os.path.join(REPO_ROOT, 'docs', f'datos_{escenario}.csv')
wbt_path  = os.path.join(REPO_ROOT, 'worlds', f'escenario_{escenario}.wbt')

# Único goal alcanzable con el controlador actual: la detección de escenario en
# robotito-controller.py busca la palabra "semplice" en el nombre del .wbt, que
# nunca aparece en "escenario_simple.wbt" -> SCENARIOS["complejo"] se usa SIEMPRE,
# en ambos mundos. Por eso el robot parte de (-1,-1) y la meta real es (1,1)
# sin importar qué escenario se cargue.
GOAL_CFG = (1.0, 1.0)
WAYPOINT_REACHED = 0.07
IR_DANGER_THRESHOLD = 100

# =============================================================================
# LEER CSV
# =============================================================================

tiempos, xs, ys = [], [], []
thetas, wp_rest, dist_wps = [], [], []
estados_lista = []
crudos, kalmans = [], []
enc_izq, enc_der = [], []
long_ej = []

with open(csv_path, 'r') as f:
    for row in csv.DictReader(f):
        tiempos.append(float(row['tiempo_s']))
        xs.append(float(row['x_est']))
        ys.append(float(row['y_est']))
        thetas.append(float(row['theta_deg']))
        wp_rest.append(int(row['wp_restantes']))
        dist_wps.append(float(row['dist_wp']))
        estados_lista.append(row['estado'])
        crudos.append(float(row['crudo_m']))
        kalmans.append(float(row['kalman_m']))
        enc_izq.append(float(row['enc_izq_rad']))
        enc_der.append(float(row['enc_der_rad']))
        long_ej.append(float(row['long_ejecutada_m']))

START_CFG = (xs[0], ys[0])

# =============================================================================
# RECONSTRUIR RUTA PLANIFICADA (para superponerla en el XY)
# =============================================================================

grid_map  = path_planning.create_map_from_wbt(wbt_path)
grid_path = path_planning.a_star(
    grid_map,
    path_planning.world_to_grid(*START_CFG),
    path_planning.world_to_grid(*GOAL_CFG)
)
ruta_x, ruta_y = [], []
if grid_path:
    for r, c in grid_path:
        wx, wy = path_planning.grid_to_world(r, c)
        ruta_x.append(wx)
        ruta_y.append(wy)

# =============================================================================
# MÉTRICAS RESUMEN
# =============================================================================

longitud_plan = sum(
    math.hypot(ruta_x[i] - ruta_x[i - 1], ruta_y[i] - ruta_y[i - 1])
    for i in range(1, len(ruta_x))
)

longitud_ejec = long_ej[-1] if long_ej else 0.0
tiempo_total  = tiempos[-1] if tiempos else 0.0
n_reversing   = sum(1 for i in range(1, len(estados_lista))
                    if estados_lista[i] == "REVERSING" and estados_lista[i - 1] != "REVERSING")
dist_final_goal = math.hypot(xs[-1] - GOAL_CFG[0], ys[-1] - GOAL_CFG[1]) if xs else float('inf')
llego = dist_final_goal < 0.10

print(f"\n{'='*50}")
print(f"  MÉTRICAS — Escenario: {escenario}")
print(f"{'='*50}")
print(f"  Tiempo total              : {tiempo_total:.1f} s")
print(f"  Longitud planificada      : {longitud_plan:.3f} m")
print(f"  Longitud ejecutada        : {longitud_ejec:.3f} m")
print(f"  Diferencia ruta/trayect.  : {abs(longitud_ejec - longitud_plan):.3f} m")
print(f"  Obstáculos imprevistos    : {n_reversing}")
print(f"  Meta alcanzada            : {'Sí' if llego else 'No'}")
print(f"{'='*50}\n")

# =============================================================================
# HELPERS PARA DIBUJAR OBSTÁCULOS (leídos directamente del .wbt actual)
# =============================================================================

def parse_solidboxes(path):
    with open(path, 'r', encoding='utf-8') as f:
        content = f.read()
    boxes = re.findall(r'SolidBox\s*\{(.*?)\}', content, re.DOTALL)
    out = []
    for box in boxes:
        n_match = re.search(r'name\s+"([^"]*)"', box)
        name = n_match.group(1) if n_match else ""
        if name.lower().startswith("marca"):
            continue
        t_match = re.search(r'translation\s+([-\d\.eE]+)\s+([-\d\.eE]+)', box)
        s_match = re.search(r'size\s+([-\d\.eE]+)\s+([-\d\.eE]+)', box)
        if not t_match or not s_match:
            continue
        cx, cy = float(t_match.group(1)), float(t_match.group(2))
        w, h = float(s_match.group(1)), float(s_match.group(2))
        theta = 0.0
        r_match = re.search(r'rotation\s+([-\d\.eE]+)\s+([-\d\.eE]+)\s+([-\d\.eE]+)\s+([-\d\.eE]+)', box)
        if r_match:
            z_axis, angle = float(r_match.group(3)), float(r_match.group(4))
            theta = angle if z_axis >= 0 else -angle
        out.append((cx, cy, w, h, theta))
    return out

def draw_obstacles(ax):
    for cx, cy, w, h, theta in parse_solidboxes(wbt_path):
        # obstaculos chicos (<=15cm) son "imprevistos": el planificador los ignora
        # y el robot solo los esquiva de forma reactiva con el sensor IR
        es_imprevisto = w <= 0.15 and h <= 0.15
        color = 'firebrick' if es_imprevisto else 'steelblue'
        rect = patches.Rectangle((-w / 2, -h / 2), w, h,
                                  color=color, alpha=0.6, zorder=3)
        rect.set_transform(Affine2D().rotate(theta).translate(cx, cy) + ax.transData)
        ax.add_patch(rect)

# =============================================================================
# FIGURA 1 — Trayectoria XY
# =============================================================================

fig1, ax1 = plt.subplots(figsize=(7, 7))
ax1.set_title(f'Trayectoria del robot — Escenario {escenario}', fontsize=13)

ax1.add_patch(patches.Rectangle((-1.25, -1.25), 2.5, 2.5,
    linewidth=2, edgecolor='black', facecolor='#f5f0e8', zorder=1))

draw_obstacles(ax1)

if ruta_x:
    ax1.plot(ruta_x, ruta_y, '--', color='orange', linewidth=1.2,
              alpha=0.8, label='Ruta planificada (A*)', zorder=4)

ax1.plot(xs, ys, color='royalblue', linewidth=1.5,
          label='Trayectoria ejecutada (odometría)', zorder=5)

ax1.plot(xs[0], ys[0], 'go', markersize=10, label='Inicio', zorder=6)
ax1.plot(GOAL_CFG[0], GOAL_CFG[1], 'r*', markersize=14, label='Meta', zorder=6)

reversing_xs = [xs[i] for i in range(len(estados_lista)) if estados_lista[i] == "REVERSING"]
reversing_ys = [ys[i] for i in range(len(estados_lista)) if estados_lista[i] == "REVERSING"]
if reversing_xs:
    ax1.scatter(reversing_xs, reversing_ys, c='red', s=4, alpha=0.5,
                label='Evitación de obstáculo imprevisto', zorder=5)

ax1.set_xlim(-1.35, 1.35)
ax1.set_ylim(-1.35, 1.35)
ax1.set_xlabel('x (m)')
ax1.set_ylabel('y (m)')
ax1.set_aspect('equal')
ax1.legend(loc='upper left', fontsize=8)
ax1.grid(True, linestyle='--', alpha=0.3)

info = (f"Tiempo: {tiempo_total:.0f} s\n"
        f"L planificada: {longitud_plan:.2f} m\n"
        f"L ejecutada: {longitud_ejec:.2f} m\n"
        f"Evitaciones: {n_reversing}x")
ax1.text(0.98, 0.02, info, transform=ax1.transAxes,
          fontsize=8, va='bottom', ha='right',
          bbox=dict(boxstyle='round', facecolor='white', alpha=0.8))

plt.tight_layout()
fig1.savefig(os.path.join(REPO_ROOT, 'docs', f'trayectoria_{escenario}.png'), dpi=150)
print(f'Guardado: docs/trayectoria_{escenario}.png')

# =============================================================================
# FIGURA 2 — Señal IR frontal (crudo vs Kalman) + encoders
# =============================================================================

fig2, (ax2a, ax2b) = plt.subplots(2, 1, figsize=(12, 8))
fig2.suptitle(f'Señal IR frontal y encoders — Escenario {escenario}', fontsize=13)

ax2a.plot(tiempos, crudos, label='Crudo (sensor IR)', alpha=0.5, color='red')
ax2a.plot(tiempos, kalmans, label='Filtro de Kalman', color='blue', linewidth=1.5)
ax2a.axhline(y=IR_DANGER_THRESHOLD, color='gray', linestyle='--', linewidth=0.8,
             label=f'Umbral evitación ({IR_DANGER_THRESHOLD})')

en_reversing, t_ini = False, None
for i, est in enumerate(estados_lista):
    if est == "REVERSING" and not en_reversing:
        en_reversing, t_ini = True, tiempos[i]
    elif est != "REVERSING" and en_reversing:
        en_reversing = False
        ax2a.axvspan(t_ini, tiempos[i], alpha=0.12, color='red', label='_nolegend_')
if en_reversing:
    ax2a.axvspan(t_ini, tiempos[-1], alpha=0.12, color='red')

ax2a.set_ylabel('Valor IR (unidades crudas del sensor)')
ax2a.set_xlabel('Tiempo (s)')
ax2a.set_title('Sensor IR frontal (ps0/ps7): crudo vs Kalman')
ax2a.legend(fontsize=9)
ax2a.grid(True)

ax2b.plot(tiempos, enc_izq, label='Encoder izquierdo (rad)', color='green')
ax2b.plot(tiempos, enc_der, label='Encoder derecho (rad)', color='purple')
ax2b.set_ylabel('Posición angular (rad)')
ax2b.set_xlabel('Tiempo (s)')
ax2b.set_title('Lecturas de encoders de rueda')
ax2b.legend(fontsize=9)
ax2b.grid(True)

plt.tight_layout()
fig2.savefig(os.path.join(REPO_ROOT, 'docs', f'senales_{escenario}.png'), dpi=150)
print(f'Guardado: docs/senales_{escenario}.png')

# =============================================================================
# FIGURA 3 — Distancia al waypoint + longitud ejecutada
# =============================================================================

fig3, (ax3a, ax3b) = plt.subplots(2, 1, figsize=(12, 6))
fig3.suptitle(f'Seguimiento de waypoints — Escenario {escenario}', fontsize=13)

ax3a.plot(tiempos, dist_wps, color='darkorange', linewidth=1.0)
ax3a.axhline(y=WAYPOINT_REACHED, color='gray', linestyle='--', linewidth=0.8,
             label=f'Tolerancia llegada a waypoint ({WAYPOINT_REACHED} m)')
ax3a.set_ylabel('Distancia al waypoint (m)')
ax3a.set_xlabel('Tiempo (s)')
ax3a.set_title('Distancia al waypoint activo en cada instante')
ax3a.legend(fontsize=9)
ax3a.grid(True)

ax3b.plot(tiempos, long_ej, color='teal', linewidth=1.5)
ax3b.axhline(y=longitud_plan, color='orange', linestyle='--', linewidth=1.0,
             label=f'Longitud planificada ({longitud_plan:.2f} m)')
ax3b.set_ylabel('Longitud acumulada (m)')
ax3b.set_xlabel('Tiempo (s)')
ax3b.set_title('Longitud de trayectoria ejecutada acumulada')
ax3b.legend(fontsize=9)
ax3b.grid(True)

plt.tight_layout()
fig3.savefig(os.path.join(REPO_ROOT, 'docs', f'seguimiento_{escenario}.png'), dpi=150)
print(f'Guardado: docs/seguimiento_{escenario}.png')

plt.show()
print('\nGraficación completada.')
