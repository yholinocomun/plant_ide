"""
Planta del robot balancin (pendulo invertido sobre 2 ruedas).
Modelo en espacio de estados CORREGIDO + calibracion real del actuador.
Estado X = [x, x_dot, theta, theta_dot]   (theta en rad)
Entrada u = PWM (-255..255)
Salida y = [x, theta]
"""
import numpy as np

# --- Parametros identificados (Fases 0-4, validados) ---
A = np.array([[0, 1,        0,   0],
              [0, 0,  -7.4697,   0],
              [0, 0,        0,   1],
              [0, 0, 104.6806,   0]])

# B en PWM con K_tau NOMINAL (placa)
B_nom = np.array([[0.0], [0.1979], [0.0], [-1.4869]])

# Calibracion real del actuador (hallada en hardware: Kang_p=59.5 funciona)
K_TAU_NOM  = 0.00338      # N*m/PWM  (placa, sin carga)
K_TAU_REAL = 0.00054      # N*m/PWM  (real, bajo carga)
ESCALA_ACT = K_TAU_REAL / K_TAU_NOM      # ~0.16  (motor real ~6x mas debil)

B = B_nom * ESCALA_ACT    # B REAL en PWM  -> usar este para disenar

C = np.array([[1, 0, 0, 0],
              [0, 0, 1, 0]])
D = np.zeros((2, 1))

def plant():
    return A, B, C, D
