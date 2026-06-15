"""
perception.py - Lettura e interpretazione dei sensori IR dell'e-puck.

Responsabilita':
  - mappare i sensori IR ai loro angoli REALI (montaggio fisico e-puck)
  - rilevare ostacoli vicini (collision avoidance reattiva)
  - fornire un riferimento angolare OPZIONALE per correggere theta

Fatti hardware (e-puck, Webots), verificati:
  - 8 sensori ps0..ps7 numerati IN SENSO ORARIO a partire dal FRONTE
  - ps0,ps1,ps2 guardano a DESTRA ; ps5,ps6,ps7 guardano a SINISTRA
  - i due sensori FRONTALI sono ps0 (~ -17 gradi) e ps7 (~ +17 gradi)
  - portata utile ~4 cm: oltre questa distanza il sensore legge ~0
  - valore grezzo: 0 (niente) ... ~4000 (ostacolo a contatto)
"""

import math

# Angoli REALI dei sensori nel frame del robot.
# Convenzione: fronte = 0 rad, SINISTRA = + , DESTRA = -  (anti-orario positivo).
SENSOR_ANGLES = {
    'ps0': math.radians(-17),   # fronte-destra
    'ps1': math.radians(-45),   # destra
    'ps2': math.radians(-90),   # lato destro
    'ps3': math.radians(-150),  # retro-destra
    'ps4': math.radians(150),   # retro-sinistra
    'ps5': math.radians(90),    # lato sinistro
    'ps6': math.radians(45),    # sinistra
    'ps7': math.radians(17),    # fronte-sinistra
}

OBSTACLE_THRESHOLD = 80.0   # valore grezzo: sopra questo = ostacolo vicino
MAX_RAW = 4000.0            # saturazione lettura grezza


class Perception:
    def __init__(self, robot, timestep,
                 sensor_names=('ps0', 'ps1', 'ps2', 'ps5', 'ps6', 'ps7')):
        self.names = list(sensor_names)
        self.sensors = {}
        for name in self.names:
            dev = robot.getDevice(name)
            dev.enable(timestep)
            self.sensors[name] = dev
        self.angles = {n: SENSOR_ANGLES[n] for n in self.names}

    def read_raw(self):
        """Valori grezzi {nome: valore}. nan -> 0.0 (= nessun ostacolo)."""
        out = {}
        for n, dev in self.sensors.items():
            v = dev.getValue()
            out[n] = v if math.isfinite(v) else 0.0
        return out

    def obstacle_flags(self, raw=None):
        """(front, left, right) booleani: ostacolo vicino in quella zona."""
        if raw is None:
            raw = self.read_raw()
        g = lambda n: raw.get(n, 0.0)
        front = g('ps0') > OBSTACLE_THRESHOLD or g('ps7') > OBSTACLE_THRESHOLD
        right = any(g(n) > OBSTACLE_THRESHOLD for n in ('ps0', 'ps1', 'ps2'))
        left = any(g(n) > OBSTACLE_THRESHOLD for n in ('ps5', 'ps6', 'ps7'))
        return front, left, right

    def reactive_turn(self, raw=None):
        """
        Bias di sterzata reattivo (adimensionale, ~[-2, 2]):
        positivo = gira a SINISTRA, negativo = gira a DESTRA.
        Somma pesata stile Braitenberg: piu' un sensore di destra "vede",
        piu' spingiamo a sinistra, e viceversa.
        """
        if raw is None:
            raw = self.read_raw()
        bias = 0.0
        push_left = {'ps0': 1.0, 'ps1': 0.6, 'ps2': 0.3}   # destra -> spinge a sx (+)
        push_right = {'ps7': 1.0, 'ps6': 0.6, 'ps5': 0.3}  # sinistra -> spinge a dx (-)
        for n, wgt in push_left.items():
            bias += wgt * min(raw.get(n, 0.0), MAX_RAW) / MAX_RAW
        for n, wgt in push_right.items():
            bias -= wgt * min(raw.get(n, 0.0), MAX_RAW) / MAX_RAW
        return bias

    def heading_correction(self, raw=None):
        """
        Correzione OPZIONALE di orientamento, dimensionalmente corretta.

        Confronta coppie di sensori SIMMETRICHE (sx vs dx). Se entrambe vedono
        lo stesso muro frontale e le letture differiscono, il robot e' inclinato
        rispetto al muro. Ritorna una DIFFERENZA NORMALIZZATA (adimensionale)
        in [-1, 1]; il chiamante la moltiplica per un guadagno [rad].

        Differenza chiave rispetto alla vecchia fusione:
          - NON si sommano metri a radianti: si usa una differenza normalizzata
            come segnale per un controllo proporzionale sull'angolo;
          - si usa l'ASIMMETRIA sx/dx (che codifica l'orientamento), non la
            media dei residui (che codifica la posizione);
          - agisce solo quando un muro e' davvero rilevato (entrambe > soglia).
        """
        if raw is None:
            raw = self.read_raw()
        pairs = [('ps7', 'ps0'), ('ps6', 'ps1')]  # (sinistra, destra) simmetriche
        contributions = []
        for left_name, right_name in pairs:
            l = raw.get(left_name, 0.0)
            r = raw.get(right_name, 0.0)
            if l > OBSTACLE_THRESHOLD and r > OBSTACLE_THRESHOLD:
                # destra piu' vicina (r>l) -> muro piu' vicino a destra
                # -> bisogna ruotare a SINISTRA (theta +) -> contributo positivo
                contributions.append((r - l) / MAX_RAW)
        if not contributions:
            return 0.0
        return sum(contributions) / len(contributions)
