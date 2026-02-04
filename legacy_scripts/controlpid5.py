import pyrealsense2 as rs
import numpy as np
import cv2
import pigpio
import time
import threading

# ==========================================
# 1. PARÁMETROS DE CALIBRACIÓN (CONTROL)
# ==========================================
SENTIDO = 1          # Cambia a 1 si el giro está invertido físicamente
K_BASE = 1.0         # Sensibilidad en rectas (Prueba entre 0.15 y 0.3)
K_AGRESIVO = 0.0025   # Cuánto aumenta el Kp por cada píxel de error
ZONA_MUERTA = 10      # Píxeles de tolerancia en el centro
VEL_CRUCERO = 1560    # Velocidad base (1500=Neutro, >1500=Avance)

# Configuración de las "Rebanadas" (Scanlines)
Y_INF = 400           # Posición actual (Cerca del coche)
Y_SUP = 300           # Anticipación (Mirando hacia adelante)
ALTO_SCAN = 40         # Grosor de la franja

# Colores calibrados (Azul Cinta)
AZUL_BAJO = np.array([82, 62, 110])
AZUL_ALTO = np.array([135, 255, 255])

# ==========================================
# 2. CONFIGURACIÓN DE HARDWARE (GPIO)
# ==========================================
ESC_PIN = 18; SERVO_PIN = 13
LED_BLANCO = 26; LED_ROJO = 27
LED_GIRO_IZQ = 22; LED_GIRO_DER = 23
NEUTRO = 1500
LIMIT_IZQ = 1200
LIMIT_DER = 1800

# ==========================================
# 3. VARIABLES DE ESTADO GLOBALES
# ==========================================
pi = pigpio.pi()
ejecutando = True
error_posicion = 0
error_anticipacion = 0
lineas_vistas = 0
intermitentes_emergencia = False

# ==========================================
# 4. HILO DE VISIÓN (SCANLINE ROI)
# ==========================================
def procesar_scanline(hsv_img, y):
    # Extraer solo la franja necesaria para ahorrar CPU
    roi = hsv_img[y : y + ALTO_SCAN, :]
    mask = cv2.inRange(roi, AZUL_BAJO, AZUL_ALTO)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((3,3), np.uint8))
    
    cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    puntos = []
    for c in cnts:
        if cv2.contourArea(c) > 40:
            M = cv2.moments(c)
            if M["m00"] != 0:
                puntos.append(int(M["m10"] / M["m00"]))
    
    puntos.sort()
    if len(puntos) >= 2:
        # Detectamos carril (Línea Izquierda y Derecha)
        centro = (puntos[0] + puntos[-1]) // 2
        return centro, mask
    return None, mask

def hilo_vision():
    global ejecutando, error_posicion, error_anticipacion, lineas_vistas, intermitentes_emergencia
    
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

            # Analizar ambas franjas
            c_inf, _ = procesar_scanline(hsv, Y_INF)
            c_sup, _ = procesar_scanline(hsv, Y_SUP)

            if c_inf is not None and c_sup is not None:
                lineas_vistas = 2
                error_posicion = c_inf - centro_cam
                error_anticipacion = c_sup - c_inf # Vector de dirección de la pista
                intermitentes_emergencia = False
            else:
                lineas_vistas = 0
                intermitentes_emergencia = True # Activar si pierde la pista

            # Visualización rápida para debug
            if c_inf: cv2.circle(img, (c_inf, Y_INF), 7, (0, 255, 0), -1)
            if c_sup: cv2.circle(img, (c_sup, Y_SUP), 7, (0, 255, 255), -1)
            cv2.imshow("Scanner de Carril", img)
            if cv2.waitKey(1) & 0xFF == ord('q'): break
    finally:
        pipeline.stop()

# ==========================================
# 5. HILO DE LUCES Y SEGURIDAD
# ==========================================
def hilo_luces():
    global ejecutando, intermitentes_emergencia
    for p in [LED_BLANCO, LED_ROJO, LED_GIRO_IZQ, LED_GIRO_DER]:
        pi.set_mode(p, pigpio.OUTPUT)

    while ejecutando:
        if intermitentes_emergencia:
            # Parpadeo de emergencia y freno
            estado = not pi.read(LED_GIRO_IZQ)
            pi.write(LED_GIRO_IZQ, estado); pi.write(LED_GIRO_DER, estado)
            pi.write(LED_ROJO, 1)
        else:
            pi.write(LED_GIRO_IZQ, 0); pi.write(LED_GIRO_DER, 0); pi.write(LED_ROJO, 0)
        time.sleep(0.4)

# ==========================================
# 6. BUCLE MAESTRO DE NAVEGACIÓN
# ==========================================
def bucle_autonomo():
    global ejecutando, error_posicion, error_anticipacion, lineas_vistas
    
    # Armado del sistema
    pi.set_servo_pulsewidth(ESC_PIN, NEUTRO)
    pi.set_servo_pulsewidth(SERVO_PIN, NEUTRO)
    print("Iniciando Sistema... Calibrando 2 seg.")
    time.sleep(2)
    
    threading.Thread(target=hilo_vision, daemon=True).start()
    threading.Thread(target=hilo_luces, daemon=True).start()

    try:
        while ejecutando:
            if lineas_vistas >= 2:
                # --- A. LÓGICA DE GIRO DINÁMICO ---
                # Error total = Posición actual + 60% de lo que viene adelante
                error_total = error_posicion + (error_anticipacion * 0.6)
                
                # Ajustar agresividad (Kp Dinámico)
                kp_actual = K_BASE + (abs(error_total) * K_AGRESIVO)
                
                if abs(error_total) < ZONA_MUERTA:
                    pwm_dir = NEUTRO
                else:
                    ajuste = error_total * kp_actual * SENTIDO
                    pwm_dir = NEUTRO + ajuste
                
                # --- B. LÓGICA DE VELOCIDAD ---
                # Si el giro es muy pronunciado, reduce la velocidad automáticamente
                if abs(error_total) > 80:
                    vel_final = VEL_CRUCERO - 8
                else:
                    vel_final = VEL_CRUCERO
                
                # --- C. EJECUCIÓN ---
                pwm_dir = max(LIMIT_IZQ, min(LIMIT_DER, pwm_dir))
                pi.set_servo_pulsewidth(SERVO_PIN, pwm_dir)
                pi.set_servo_pulsewidth(ESC_PIN, vel_final)
                
            else:
                # SISTEMA DE SEGURIDAD: Freno seco si se pierde la pista
                pi.set_servo_pulsewidth(ESC_PIN, NEUTRO)
                pi.set_servo_pulsewidth(SERVO_PIN, NEUTRO)
            
            time.sleep(0.02) # Control a 50Hz

    except KeyboardInterrupt:
        ejecutando = False
    finally:
        pi.set_servo_pulsewidth(ESC_PIN, NEUTRO)
        pi.set_servo_pulsewidth(SERVO_PIN, NEUTRO)
        for p in [LED_BLANCO, LED_ROJO, LED_GIRO_IZQ, LED_GIRO_DER]: pi.write(p, 0)
        pi.stop()

if __name__ == "__main__":
    bucle_autonomo()
