import csv
import sys
import matplotlib.pyplot as plt

escenario = sys.argv[1] if len(sys.argv) > 1 else "simulacion"

tiempos, crudos, filtrados, kalmans, enc_izq, enc_der = [], [], [], [], [], []

with open('docs/datos_simulacion.csv', 'r') as f:
    for row in csv.DictReader(f):
        tiempos.append(float(row['tiempo_s']))
        crudos.append(float(row['crudo_metros']))
        filtrados.append(float(row['filtrado_metros']))
        kalmans.append(float(row['kalman_metros']))
        enc_izq.append(float(row['encoder_izq_rad']))
        enc_der.append(float(row['encoder_der_rad']))

fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 8))
fig.suptitle(f'Escenario: {escenario}', fontsize=13)

ax1.plot(tiempos, crudos,    label='Crudo',                alpha=0.5, color='red')
ax1.plot(tiempos, filtrados, label='Filtro simple (α=0.25)', color='orange', linewidth=1.5)
ax1.plot(tiempos, kalmans,   label='Kalman (d_est)',        color='blue',   linewidth=1.5)
ax1.axhline(y=0.128, color='gray', linestyle='--', linewidth=0.8, label='Umbral AVOID (0.128 m)')
ax1.set_ylabel('Distancia frontal (m)')
ax1.set_xlabel('Tiempo (s)')
ax1.set_title('Señales de distancia frontal al obstáculo')
ax1.legend()
ax1.grid(True)

ax2.plot(tiempos, enc_izq, label='Encoder izquierdo (rad)', color='green')
ax2.plot(tiempos, enc_der, label='Encoder derecho (rad)',   color='purple')
ax2.set_ylabel('Posición angular (rad)')
ax2.set_xlabel('Tiempo (s)')
ax2.set_title('Lecturas de encoders de rueda')
ax2.legend()
ax2.grid(True)

plt.tight_layout()
output = f'docs/grafico_{escenario}.png'
plt.savefig(output, dpi=150)
plt.show()
print(f'Grafico guardado en {output}')
