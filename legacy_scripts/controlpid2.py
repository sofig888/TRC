# ==========================================
# PARÁMETROS DE AUTO-AJUSTE (SINTONIZACIÓN)
# ==========================================
KP_MIN = 0.2          # Ganancia para correcciones suaves en rectas
KP_MAX = 2.0           # Ganancia máxima para curvas cerradas
FACTOR_CURVA = 0.010   # Qué tan rápido sube el Kp según el error

VEL_RECTA = 1565       # Velocidad en rectas
VEL_CURVA_MIN = 1545   # Velocidad mínima al detectar curva fuerte (para no salirse)

# ==========================================
# LÓGICA DE CONTROL DINÁMICO
# ==========================================
def calcular_control_inteligente(error):
    abs_error = abs(error)
    
    # 1. AJUSTE DINÁMICO DE KP (Auto-Calibración en tiempo real)
    # Entre más grande el error, más sube el KP automáticamente
    kp_dinamico = KP_MIN + (abs_error * FACTOR_CURVA)
    kp_dinamico = min(kp_dinamico, KP_MAX) # No dejar que suba al infinito
    
    # 2. CÁLCULO DE DIRECCIÓN
    ajuste = error * kp_dinamico * SENTIDO
    pwm_dir = NEUTRO + ajuste
    
    # 3. CONTROL DE VELOCIDAD DINÁMICO (Frenado en curva)
    # Si el error es mayor a 50px, empezamos a bajar la velocidad
    if abs_error > 50:
        # Mapeo: A más error, nos acercamos a VEL_CURVA_MIN
        reduccion = abs_error * 0.1
        vel_final = max(VEL_CURVA_MIN, VEL_RECTA - reduccion)
    else:
        vel_final = VEL_RECTA
        
    return int(pwm_dir), int(vel_final)
