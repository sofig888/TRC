import pyrealsense2 as rs
import numpy as np
import cv2
import pigpio
import time
import threading

# ==========================================
# 1. PARÁMETROS DE CALIBRACIÓN (MODIFICA AQUÍ)
# ==========================================
KP = 3.0             # Sensibilidad fija (ajusta según tus pruebas)
SENTIDO = 1          # Cambia a 1 o -1 para corregir dirección invertida
ZONA_MUERTA = 5      # Píxeles de tolerancia en el centro
VEL_AUTONOMA = 1570   # Velocidad base para avanzar

# Alturas de escaneo (Scanlines)
Y_INF = 400           # Línea de posición (cerca)
Y_SUP = 319           # Línea de anticipación (lejos)
ALTO_SCAN = 80         # Grosor de las franjas visuales

# Filtro de color (Tus valores calibrados)
AZUL_BAJO = np.array([82, 62, 110])
AZUL_ALTO = np.array([135, 255, 255])

# Pines GPIO
ESC_PIN = 18; SERVO_PIN = 13
LED_BLANCO = 26; LED_ROJO = 27
LED_GIRO_IZQ = 22; LED_GIRO_DER = 23
NEUTRO = 1500

# ==========================================
# 2. VARIABLES GLOBALES DE ESTADO
# ==========================================
pi = pigpio.pi()
ejecutando = True
error_final = 0
pista_detectada = False
intermitentes_emergencia = False

# Inicializar Pines
for p in [LED_BLANCO, LED_ROJO, LED_GIRO_IZQ, LED_GIRO_DER]:
    pi.set_mode(p, pigpio.OUTPUT)
    pi.write(p, 0)

# ==========================================
# 3. LÓGICA DE VISIÓN (SCANLINE B/N)
# ==========================================
def obtener_centro_franja(hsv_img, y, ancho_cam):
    roi = hsv_img[y : y + ALTO_SCAN, :]
    mask = cv2.inRange(roi, AZUL_BAJO, AZUL_ALTO)
    mask = cv2.dilate(mask, None, iterations=1)
    
    cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    puntos = []
    for c in cnts:
        if cv2.contourArea(c) > 50:
            M = cv2.moments(c)
            if M["m00"] != 0:
                puntos.append(int(M["m10"] / M["m00"]))
    
    puntos.sort() # El primero es izquierda, el último es derecha
    
    if len(puntos) >= 2:
        centro = (puntos[0] + puntos[-1]) // 2
        return centro, mask
    return None, mask

def hilo_vision():
    global ejecutando, error_final, pista_detectada, intermitentes_emergencia
    
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
            h, w = img.shape[:2]
            centro_cam = w // 2
            hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

            # Procesar las dos franjas
            c_inf, mask_inf = obtener_centro_franja(hsv, Y_INF, w)
            c_sup, mask_sup = obtener_centro_franja(hsv, Y_SUP, w)

            # --- CONSTRUCCIÓN DE LA IMAGEN VISUAL ---
            # Imagen normal de fondo
            img_show = img.copy()

            # Insertar el filtro B/N solo en las rebanadas
            for y, mask in [(Y_INF, mask_inf), (Y_SUP, mask_sup)]:
                mask_bgr = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
                img_show[y : y + ALTO_SCAN, :] = mask_bgr

            if c_inf is not None and c_sup is not None:
                pista_detectada = True
                intermitentes_emergencia = False
                # El error es una mezcla: posición actual + inclinación hacia adelante
                error_pos = c_inf - centro_cam
                error_inc = c_sup - c_inf
                error_final = error_pos + (error_inc * 0.5)
                
                # Dibujar guías visuales
                cv2.line(img_show, (c_inf, Y_INF), (c_sup, Y_SUP), (0, 255, 0), 2)
                cv2.circle(img_show, (c_inf, Y_INF), 5, (0, 255, 0), -1)
            else:
                pista_detectada = False
                intermitentes_emergencia = True

            cv2.imshow("Scanline B/N - Autonomo", img_show)
            if cv2.waitKey(1) & 0xFF == ord('q'): break
    finally:
        pipeline.stop()

# ==========================================
# 4. HILO DE LUCES
# ==========================================
def hilo_luces():
    global ejecutando, intermitentes_emergencia
    while ejecutando:
        if intermitentes_emergencia:
            estado = not pi.read(LED_GIRO_IZQ)
            pi.write(LED_GIRO_IZQ, estado); pi.write(LED_GIRO_DER, estado)
            pi.write(LED_ROJO, 1)
        else:
            pi.write(LED_GIRO_IZQ, 0); pi.write(LED_GIRO_DER, 0); pi.write(LED_ROJO, 0)
        time.sleep(0.4)

# ==========================================
# 5. CONTROL DE MOTORES (BUCLE PRINCIPAL)
# ==========================================
def control_maestro():
    global ejecutando, error_final, pista_detectada
    
    # Armado del ESC
    pi.set_servo_pulsewidth(ESC_PIN, NEUTRO)
    pi.set_servo_pulsewidth(SERVO_PIN, NEUTRO)
    print("Armando ESC... 2 segundos")
    time.sleep(2)

    # Iniciar hilos
    threading.Thread(target=hilo_vision, daemon=True).start()
    threading.Thread(target=hilo_luces, daemon=True).start()

    try:
        while ejecutando:
            if pista_detectada:
                # Aplicar dirección
                if abs(error_final) > ZONA_MUERTA:
                    ajuste = error_final * KP * SENTIDO
                    pwm_dir = NEUTRO + ajuste
                else:
                    pwm_dir = NEUTRO
                
                # Límites de seguridad
                pwm_dir = max(1200, min(1800, pwm_dir))
                pi.set_servo_pulsewidth(SERVO_PIN, pwm_dir)
                
                # Avanzar
                pi.set_servo_pulsewidth(ESC_PIN, VEL_AUTONOMA)
            else:
                # Si pierde una línea, stop y centrar dirección
                pi.set_servo_pulsewidth(ESC_PIN, NEUTRO)
                pi.set_servo_pulsewidth(SERVO_PIN, NEUTRO)
            
            time.sleep(0.02) # Frecuencia de control 50Hz

    except KeyboardInterrupt:
        ejecutando = False
    finally:
        pi.set_servo_pulsewidth(ESC_PIN, NEUTRO)
        pi.set_servo_pulsewidth(SERVO_PIN, NEUTRO)
        pi.stop()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    control_maestro()
