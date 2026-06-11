"""
graficar_final.py — Gráficos del Proyecto Final: Path Planning + Navegación Autónoma
Proyecto Final ICI 4150 — Robótica y Sistemas Autónomos 2026-01

Genera 4 figuras a partir del CSV de la simulación:
  1. Trayectoria estimada en el plano XY (con ruta planificada superpuesta)
  2. Señales de distancia: crudo vs Kalman (igual estructura que Lab 2)
  3. Encoders izquierdo y derecho vs tiempo
  4. Distancia al waypoint activo vs tiempo

Uso:
    python graficar_final.py simple
    python graficar_final.py complejo
"""

import csv
import sys
import math
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import path_planning

# =============================================================================
# ARGUMENTOS
# =============================================================================

escenario = sys.argv[1] if len(sys.argv) > 1 else "complejo"
csv_path  = f'docs/datos_{escenario}.csv'

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

# =============================================================================
# RECONSTRUIR RUTA PLANIFICADA (para superponerla en el XY)
# =============================================================================

if escenario == "simple":
    base_map   = path_planning.create_map_simple()
    start_cfg  = (-0.9,  0.0)
    goal_cfg   = ( 0.9,  0.0)
else:
    base_map   = path_planning.create_map_complejo()
    start_cfg  = (-1.1, -1.1)
    goal_cfg   = ( 1.1,  1.1)

safe_map   = path_planning.inflate_map(base_map)
grid_path  = path_planning.a_star(
    safe_map,
    path_planning.world_to_grid(*start_cfg),
    path_planning.world_to_grid(*goal_cfg)
)
ruta_x, ruta_y = [], []
if grid_path:
    for p in grid_path:
        wx, wy = path_planning.grid_to_world(p[0], p[1])
        ruta_x.append(wx)
        ruta_y.append(wy)

# =============================================================================
# MÉTRICAS RESUMEN
# =============================================================================

longitud_plan = 0.0
for i in range(1, len(ruta_x)):
    longitud_plan += math.hypot(ruta_x[i]-ruta_x[i-1], ruta_y[i]-ruta_y[i-1])

longitud_ejec   = long_ej[-1] if long_ej else 0.0
tiempo_total    = tiempos[-1]  if tiempos else 0.0
n_avoid         = sum(1 for i in range(1, len(estados_lista))
                      if estados_lista[i] == "AVOID" and estados_lista[i-1] != "AVOID")
llego           = estados_lista[-1] == "DONE"

print(f"\n{'='*50}")
print(f"  MÉTRICAS — Escenario: {escenario}")
print(f"{'='*50}")
print(f"  Tiempo total              : {tiempo_total:.1f} s")
print(f"  Longitud planificada      : {longitud_plan:.3f} m")
print(f"  Longitud ejecutada        : {longitud_ejec:.3f} m")
print(f"  Diferencia ruta/trayect.  : {abs(longitud_ejec - longitud_plan):.3f} m")
print(f"  Activaciones AVOID        : {n_avoid}")
print(f"  Meta alcanzada            : {'Sí' if llego else 'No'}")
print(f"{'='*50}\n")

# =============================================================================
# HELPERS PARA DIBUJAR OBSTÁCULOS
# =============================================================================

def draw_obstacles(ax):
    if escenario == "simple":
        for cx, cy in [(0.00, 0.00), (0.70, 0.50), (0.50, -0.70)]:
            ax.add_patch(patches.Rectangle(
                (cx-0.075, cy-0.075), 0.15, 0.15,
                color='saddlebrown', alpha=0.75, zorder=3))
    else:
        # Spines y trampas (versión simplificada eje-alineada, igual que el mapa)
        muros = [
            (-0.79-0.05,  -0.52-0.30,  0.10, 0.60),
            (-0.425-0.30, -0.425-0.30, 0.60, 0.60),
            ( 0.175-0.30, -0.389-0.30, 0.60, 0.60),
            ( 0.90-0.35,   0.17-0.05,  0.70, 0.10),
            ( 0.45-0.35,   0.66-0.05,  0.70, 0.10),
            (-0.79-0.05,   0.18-0.35,  0.10, 0.70),
            (-0.48-0.25,  -0.77-0.05,  0.50, 0.10),
            (-0.25-0.05,   1.05-0.20,  0.10, 0.40),
            (-0.08-0.05,   0.09-0.20,  0.10, 0.40),
            ( 0.75-0.05,  -0.50-0.20,  0.10, 0.40),
            ( 0.58-0.20,  -0.79-0.05,  0.40, 0.10),
        ]
        for x0, y0, w, h in muros:
            ax.add_patch(patches.Rectangle((x0, y0), w, h,
                         color='steelblue', alpha=0.5, zorder=3))
        for cx, cy in [(-0.8, 0.68), (0.44, -0.34), (-0.58, -0.03)]:
            ax.add_patch(patches.Rectangle(
                (cx-0.05, cy-0.05), 0.10, 0.10,
                color='saddlebrown', alpha=0.75, zorder=3))

# =============================================================================
# FIGURA 1 — Trayectoria XY
# =============================================================================

fig1, ax1 = plt.subplots(figsize=(7, 7))
ax1.set_title(f'Trayectoria del robot — Escenario {escenario}', fontsize=13)

# Arena
ax1.add_patch(patches.Rectangle((-1.25, -1.25), 2.5, 2.5,
    linewidth=2, edgecolor='black', facecolor='#f5f0e8', zorder=1))

draw_obstacles(ax1)

# Ruta planificada por A*
if ruta_x:
    ax1.plot(ruta_x, ruta_y, '--', color='orange', linewidth=1.2,
             alpha=0.8, label='Ruta planificada (A*)', zorder=4)

# Trayectoria ejecutada
ax1.plot(xs, ys, color='royalblue', linewidth=1.5,
         label='Trayectoria ejecutada (odometría)', zorder=5)

# Inicio y meta
ax1.plot(xs[0],  ys[0],  'go', markersize=10, label='Inicio', zorder=6)
ax1.plot(goal_cfg[0], goal_cfg[1], 'r*', markersize=14, label='Meta', zorder=6)

# Zonas AVOID destacadas
avoid_xs = [xs[i] for i in range(len(estados_lista)) if estados_lista[i] == "AVOID"]
avoid_ys = [ys[i] for i in range(len(estados_lista)) if estados_lista[i] == "AVOID"]
if avoid_xs:
    ax1.scatter(avoid_xs, avoid_ys, c='red', s=4, alpha=0.4,
                label='Zona AVOID activo', zorder=5)

ax1.set_xlim(-1.35, 1.35)
ax1.set_ylim(-1.35, 1.35)
ax1.set_xlabel('x (m)')
ax1.set_ylabel('y (m)')
ax1.set_aspect('equal')
ax1.legend(loc='upper left', fontsize=8)
ax1.grid(True, linestyle='--', alpha=0.3)

# Anotaciones de métricas
info = (f"Tiempo: {tiempo_total:.0f} s\n"
        f"L planificada: {longitud_plan:.2f} m\n"
        f"L ejecutada: {longitud_ejec:.2f} m\n"
        f"AVOID: {n_avoid}x")
ax1.text(0.98, 0.02, info, transform=ax1.transAxes,
         fontsize=8, va='bottom', ha='right',
         bbox=dict(boxstyle='round', facecolor='white', alpha=0.8))

plt.tight_layout()
fig1.savefig(f'docs/trayectoria_{escenario}.png', dpi=150)
print(f'Guardado: docs/trayectoria_{escenario}.png')

# =============================================================================
# FIGURA 2 — Señales de distancia (estructura igual que Lab 2)
# =============================================================================

fig2, (ax2a, ax2b) = plt.subplots(2, 1, figsize=(12, 8))
fig2.suptitle(f'Señales de distancia y encoders — Escenario {escenario}', fontsize=13)

ax2a.plot(tiempos, crudos,  label='Crudo (sensor IR)',     alpha=0.5, color='red')
ax2a.plot(tiempos, kalmans, label='Kalman (d_est)',         color='blue', linewidth=1.5)
ax2a.axhline(y=0.10, color='gray', linestyle='--', linewidth=0.8,
             label='Umbral AVOID (0.10 m)')

# Marcar zonas AVOID con fondo coloreado
en_avoid = False
t_ini_avoid = None
for i, est in enumerate(estados_lista):
    if est == "AVOID" and not en_avoid:
        en_avoid = True
        t_ini_avoid = tiempos[i]
    elif est != "AVOID" and en_avoid:
        en_avoid = False
        ax2a.axvspan(t_ini_avoid, tiempos[i], alpha=0.12, color='red', label='_nolegend_')
if en_avoid:
    ax2a.axvspan(t_ini_avoid, tiempos[-1], alpha=0.12, color='red')

ax2a.set_ylabel('Distancia frontal (m)')
ax2a.set_xlabel('Tiempo (s)')
ax2a.set_title('Señales de distancia frontal al obstáculo')
ax2a.legend(fontsize=9)
ax2a.grid(True)

ax2b.plot(tiempos, enc_izq, label='Encoder izquierdo (rad)', color='green')
ax2b.plot(tiempos, enc_der, label='Encoder derecho (rad)',   color='purple')
ax2b.set_ylabel('Posición angular (rad)')
ax2b.set_xlabel('Tiempo (s)')
ax2b.set_title('Lecturas de encoders de rueda')
ax2b.legend(fontsize=9)
ax2b.grid(True)

plt.tight_layout()
fig2.savefig(f'docs/senales_{escenario}.png', dpi=150)
print(f'Guardado: docs/senales_{escenario}.png')

# =============================================================================
# FIGURA 3 — Distancia al waypoint + longitud ejecutada
# =============================================================================

fig3, (ax3a, ax3b) = plt.subplots(2, 1, figsize=(12, 6))
fig3.suptitle(f'Seguimiento de waypoints — Escenario {escenario}', fontsize=13)

ax3a.plot(tiempos, dist_wps, color='darkorange', linewidth=1.0)
ax3a.axhline(y=0.05, color='gray', linestyle='--', linewidth=0.8,
             label='Tolerancia llegada (0.05 m)')
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
fig3.savefig(f'docs/seguimiento_{escenario}.png', dpi=150)
print(f'Guardado: docs/seguimiento_{escenario}.png')

plt.show()
print('\nGraficación completada.')
