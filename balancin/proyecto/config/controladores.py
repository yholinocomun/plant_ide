"""Metadatos UNIFORMES de cada controlador (nombre, ganancias, color).
Un solo lugar -> alimenta extraccion, graficado y la tabla comparativa."""
CTRL = {
 "lqr":   {"nombre":"LQR (pre-gain)", "color":"#1f77b4",
           "ganancias":{"Kang_p":59.50,"Kang_d":1.70,"Kpos_p":30.36,"Kpos_d":61.09},
           "diseno":"lqr(A,B,Q,R) + pre-gain; R2D; validado en HW"},
 "lqg":   {"nombre":"LQG (Kalman)", "color":"#ff7f0e",
           "ganancias":{"K":[-70.71,-196.97,-1985.22,-284.52]},
           "diseno":"lqr + lqe (observador de Kalman), u=-K x_est"},
 "fopid": {"nombre":"PID Fraccionario", "color":"#2ca02c",
           "ganancias":{"Kp":45.0,"Ki":12.0,"Kd":2.5,"lambda":0.95,"mu":0.15},
           "diseno":"pidtune -> FOPID (Oustaloup), s^-0.95 y s^0.15"},
 "mpc":   {"nombre":"LQR predictivo (MPC)", "color":"#d62728",
           "ganancias":{"Kmpc":[-1.395,-8.131,-1414.19,-197.57]},
           "diseno":"Riccati horizonte finito N=60 (predictivo)"},
 "imc":   {"nombre":"IMC (filtro Q)", "color":"#9467bd",
           "ganancias":{"K_ANG":43.5,"K_GYRO":3.10,"LAMBDA":0.010,"GAIN":0.75},
           "diseno":"PD + filtro Q de 1er orden, beta=dt/(lambda+dt)"},
 "hinf":  {"nombre":"H-infinito", "color":"#8c564b",
           "ganancias":{"orden":6,"HGAIN":0.10,"C_ctrl":"x (salida 1)"},
           "diseno":"augw(W1,W2,W3)+hinfsyn -> controlador dinamico orden 6"},
    "cascada":{"nombre":"Cascada (colocacion polos)", "color":"#17becf",
           "ganancias":{"K2":-20.33,"KC":1.19,"alpha":10.2314,"CASGAIN":4.0,"T22":0.05,"T11":0.50},
           "diseno":"cascada corregida: PI externo (cancela polo estable) + P interno estabiliza P2"},
}
COLS = ["t_ms","theta_deg","theta_dot_dps","x_m","x_dot_ms","u_pwm","setpoint_deg","modo"]
