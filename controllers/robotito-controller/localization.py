"""
localization.py - Stima della posa (x, y, theta) da odometria encoder.

Modello cinematico differenziale (Lab 1 / eq. 5-7 del progetto):
  ds  = (ds_r + ds_l) / 2
  dth = (ds_r - ds_l) / L
  x  += ds * cos(theta + dth/2)     <- integrazione al punto medio (2 ordine)
  y  += ds * sin(theta + dth/2)
  theta += dth

In simulazione Webots gli encoder sono quasi esatti: l'odometria e' la base
PIU' affidabile. La correzione IR (opzionale) limita solo il drift quando
c'e' rumore o slittamento ruota.
"""

import math


class Localization:
    def __init__(self, robot, timestep, wheel_radius, axle_length,
                 start_pose=(0.0, 0.0, 0.0)):
        self.r = wheel_radius
        self.L = axle_length
        self.x, self.y, self.theta = start_pose

        self.left_enc = robot.getDevice('left wheel sensor')
        self.right_enc = robot.getDevice('right wheel sensor')
        self.left_enc.enable(timestep)
        self.right_enc.enable(timestep)

        # Un primo step per avere letture encoder valide come riferimento.
        robot.step(timestep)
        self.prev_l = self.left_enc.getValue()
        self.prev_r = self.right_enc.getValue()

    def update(self):
        """Aggiorna la posa con un passo di odometria. Ritorna (x, y, theta)."""
        cl = self.left_enc.getValue()
        cr = self.right_enc.getValue()
        dl = (cl - self.prev_l) * self.r          # arco percorso ruota sinistra
        dr = (cr - self.prev_r) * self.r          # arco percorso ruota destra
        self.prev_l, self.prev_r = cl, cr

        ds = (dr + dl) / 2.0
        dth = (dr - dl) / self.L
        self.x += ds * math.cos(self.theta + dth / 2.0)
        self.y += ds * math.sin(self.theta + dth / 2.0)
        self.theta = self._wrap(self.theta + dth)
        return self.x, self.y, self.theta

    def correct_heading(self, delta_norm, gain=0.05):
        """
        Complementary filter leggero su theta.
        delta_norm: differenza normalizzata da Perception.heading_correction() in [-1,1].
        gain: quanto fidarsi del sensore (piccolo = odometria domina).
        """
        if delta_norm != 0.0:
            self.theta = self._wrap(self.theta + gain * delta_norm)

    @staticmethod
    def _wrap(a):
        """Normalizza un angolo in [-pi, pi]."""
        return math.atan2(math.sin(a), math.cos(a))

    @property
    def pose(self):
        return (self.x, self.y, self.theta)
