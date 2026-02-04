import pyrealsense2 as rs
import numpy as np
import cv2
import pigpio
import time
import threading

# ==========================================
# 1. CONFIGURACIÓN DE HARDWARE (GPIO)
# ==========================================
ESC_PIN = 18; SERVO_PIN = 13
LED_BLANCO = 26; LED_ROJO = 27
LED_GIRO_IZQ = 22; LED_GIRO_DER = 23

NEUTRO = 1500
VEL_CRUCERO = 1558 # Velocidad base autónoma
LIMIT_IZQ = 1200
LIMIT_DER = 1800

# ==========================================
# 2. PARÁMETROS DE NAVEGACIÓN (AJUSTABLES)
# ==========================================
SENTIDO = -1          # Cambia a 1 si el giro está invertido
K_BASE = 0.2          # Multiplicador mínimo (para rectas)
K_FACTOR = 0.002      # Aumento de agresividad por cada píxel de error
ZONA_MUERTA = 8       # Ignorar errores menores a 8px

# Alturas de escaneo (Scanlines)
Y_INF = 400           # Posición actual (cerca del parachoques)
Y_SUP = 300           # Intención/Anticipación (mirando adelante)
ALTO_SCAN = 5         # Grosor de la línea de escaneo

# Colores calibrados
AZUL_BAJO = np.array([82, 62, 110])
AZUL_ALTO = np.array([135, 255, 255])

# ==========================================
# VARIABLES GLOBALES
# ==========================================
pi = pigpio.pi()
ejecutando = True
error_actual = 0      # Error en Y_INF
error_anticipacion = 0 # Diferencia entre Y_SUP e Y_INF
lineas_vistas = 0
intermitentes_emergencia = False

# ==========================================
# HILO DE VISIÓN (SCANLINE)
# ==========================================
def procesar_franja(hsv_img, y):
    roi = hsv_img[y : y + ALTO_SCAN, :]
    mask = cv2.inRange(roi, AZUL_BAJO, AZUL_ALTO)
    cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    puntos = []
    for c in cnts:
        if cv2.contourArea(c) > 50:
            M = cv2.moments(c)
            if M["m00"] != 0:
                puntos.append(int(M["m10"] / M["m00"]))
    
    puntos.sort()
    if len(puntos) >= 2:
        izq, der = puntos[0], puntos[-1]
        return (izq + der) // 2, der - izq, mask
    return None, None, mask

def hilo_vision():
    global ejecutando, error_actual, error_anticipacion, lineas_vistas, intermitentes_emergencia
    
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    pipeline.start(config)
    
    try:
        while ejecutando:
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            if not color_frame: continue

            img = np.asanyarray(color_frame.get_data())
            hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
            centro_cam = img.shape[1] // 2

            c_inf, w_inf, _ = procesar_franja(hsv, Y_INF)
            c_sup, w_sup, _ = procesar_franja(hsv, Y_SUP)

            if c_inf is not None and c_sup is not None:
                lineas_vistas = 2
                error_actual = c_inf - centro_cam
                error_anticipacion = c_sup - c_inf # Qué tanto se "tuerce" la pista
                intermitentes_emergencia = False
            else:
                lineas_vistas = 0
                intermitentes_emergencia = True

            # Feedback visual minimalista
            if c_inf: cv2.circle(img, (c_inf, Y_INF), 5, (0, 255, 0), -1)
            if c_sup: cv2.circle(img, (c_sup, Y_SUP), 5, (0, 255, 255), -1)
            cv2.imshow("Cerebro Scanline", img)
            if cv2.waitKey(1) & 0xFF == ord('q'): break
    finally:
        pipeline.stop()

# ==========================================
# HILO DE LUCES E INTERMITENTES
# ==========================================
def hilo_luces():
    global ejecutando, intermitentes_emergencia
    for p in [LED_BLANCO, LED_ROJO, LED_GIRO_IZQ, LED_GIRO_DER]:
        pi.set_mode(p, pigpio.OUTPUT)

    while ejecutando:
        if intermitentes_emergencia:
            estado = not pi.read(LED_GIRO_IZQ)
            pi.write(LED_GIRO_IZQ, estado)
            pi.write(LED_GIRO_DER, estado)
            pi.write(LED_ROJO, 1)
        else:
            pi.write(LED_GIRO_IZQ, 0); pi.write(LED_GIRO_DER, 0); pi.write(LED_ROJO, 0)
        time.sleep(0.4)

# ==========================================
# CONTROL MAESTRO (AUTÓNOMO)
# ==========================================
def bucle_autonomo():
    global ejecutando, error_actual, error_anticipacion, lineas_vistas
    
    # Armado
    pi.set_servo_pulsewidth(ESC_PIN, NEUTRO)
    pi.set_servo_pulsewidth(SERVO_PIN, NEUTRO)
    time.sleep(2)
    
    threading.Thread(target=hilo_vision, daemon=True).start()
    threading.Thread(target=hilo_luces, daemon=True).start()

    print("SISTEMA AUTÓNOMO DINÁMICO ONLINE")

    try:
        while ejecutando:
            if lineas_vistas >= 2:
                # 1. CÁLCULO DE KP DINÁMICO
                # A mayor error, mayor el multiplicador (parabólico)
                kp_dinamico = K_BASE + (abs(error_actual) * K_FACTOR)
                
                # 2. CÁLCULO DE GIRO (Combinamos posición + anticipación)
                # Sumamos el error de posición y una parte del error de anticipación
                error_total = error_actual + (error_anticipacion * 0.5)
                
                if abs(error_total) < ZONA_MUERTA:
                    pwm_dir = NEUTRO
                else:
                    ajuste = error_total * kp_dinamico * SENTIDO
                    pwm_dir = NEUTRO + ajuste
                
                # 3. EJECUCIÓN
                pwm_dir = max(LIMIT_IZQ, min(LIMIT_DER, pwm_dir))
                pi.set_servo_pulsewidth(SERVO_PIN, pwm_dir)
                
                # Velocidad adaptativa: frena un poco si el giro es muy brusco
                if abs(error_total) > 60:
                    pi.set_servo_pulsewidth(ESC_PIN, VEL_CRUCERO - 10)
                else:
                    pi.set_servo_pulsewidth(ESC_PIN, VEL_CRUCERO)
            else:
                # STOP DE SEGURIDAD
                pi.set_servo_pulsewidth(ESC_PIN, NEUTRO)
                pi.set_servo_pulsewidth(SERVO_PIN, NEUTRO)
            
            time.sleep(0.02)

    except KeyboardInterrupt:
        ejecutando = False
    finally:
        pi.set_servo_pulsewidth(ESC_PIN, NEUTRO); pi.set_servo_pulsewidth(SERVO_PIN, NEUTRO)
        pi.stop()

if __name__ == "__main__":
    bucle_autonomo()
