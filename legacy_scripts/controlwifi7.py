import pyrealsense2 as rs
import numpy as np
import cv2
import pigpio
import time
import threading
import socket
from flask import Flask, Response

# ==========================================
# 1. CONFIGURACIÓN DEL SERVIDOR WEB
# ==========================================
app = Flask(__name__)
frame_streaming = None

def gen_frames():
    global frame_streaming
    while True:
        if frame_streaming is not None:
            ret, buffer = cv2.imencode('.jpg', frame_streaming)
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')
        else:
            time.sleep(0.1)

@app.route('/video_feed')
def video_feed():
    return Response(gen_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

# ==========================================
# 2. PARÁMETROS AJUSTABLES
# ==========================================
# --- MODOS DE OPERACIÓN ---
# Cambia esta variable manualmente o mediante una lógica externa para activar el giro
MODO_RECTO = 0
MODO_GIRO_IZQ = 1
MODO_GIRO_DER = 2
MODO_ACTUAL = MODO_GIRO_DER  # <--- Cambia esto para probar los giros

# --- Parámetros de Giro ---
PWM_GIRO_IZQ = 1200   # Ángulo del servo para vuelta izquierda
PWM_GIRO_DER = 1800   # Ángulo del servo para vuelta derecha
DELAY_CURVA = 0.001     # Tiempo (segundos) desde que ve la curva hasta que gira

# --- Control Recto ---
KP = 4.0
SENTIDO = 1
VEL_AUTONOMA = 1610

# --- Scanlines ---
Y_INF = 360; ALTO_SCAN_INF = 120
Y_SUP = 289; ALTO_SCAN_SUP = 70
DILATACION_ITER = 2
AREA_MIN_LINEA = 20

# --- Colores ---
AZUL_BAJO = np.array([82, 62, 110])
AZUL_ALTO = np.array([135, 255, 255])

# --- Hardware ---
ESC_PIN = 18; SERVO_PIN = 13
NEUTRO = 1500
pi = pigpio.pi()

# ==========================================
# 3. LÓGICA DE PROCESAMIENTO
# ==========================================
ejecutando = True
error_final = 0
pista_detectada = False

def obtener_datos_franja(hsv_img, y, alto):
    roi = hsv_img[y : y + alto, :]
    mask = cv2.inRange(roi, AZUL_BAJO, AZUL_ALTO)
    mask = cv2.dilate(mask, None, iterations=DILATACION_ITER)
    cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    puntos = []
    for c in cnts:
        if cv2.contourArea(c) > AREA_MIN_LINEA:
            M = cv2.moments(c)
            if M["m00"] != 0:
                puntos.append(int(M["m10"] / M["m00"]))
    puntos.sort()
    if len(puntos) >= 2:
        return (puntos[0] + puntos[-1]) // 2, mask
    return None, mask

def hilo_vision():
    global ejecutando, error_final, pista_detectada, frame_streaming
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

            c_inf, m_inf = obtener_datos_franja(hsv, Y_INF, ALTO_SCAN_INF)
            c_sup, m_sup = obtener_datos_franja(hsv, Y_SUP, ALTO_SCAN_SUP)

            img_viz = img.copy()
            img_viz[Y_INF : Y_INF + ALTO_SCAN_INF, :] = cv2.cvtColor(m_inf, cv2.COLOR_GRAY2BGR)
            img_viz[Y_SUP : Y_SUP + ALTO_SCAN_SUP, :] = cv2.cvtColor(m_sup, cv2.COLOR_GRAY2BGR)

            if c_inf is not None and c_sup is not None:
                pista_detectada = True
                error_pos = c_inf - centro_cam
                error_inc = c_sup - c_inf
                error_final = error_pos + (error_inc * 0.5)
                cv2.line(img_viz, (c_inf, Y_INF), (c_sup, Y_SUP), (0, 255, 0), 2)
            else:
                pista_detectada = False

            # Mostrar estado en pantalla
            cv2.putText(img_viz, f"MODO: {MODO_ACTUAL}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
            frame_streaming = img_viz
    finally:
        pipeline.stop()

# ==========================================
# 4. CONTROL DE ESTADOS
# ==========================================
def control_maestro():
    global ejecutando, error_final, pista_detectada, MODO_ACTUAL
    
    pi.set_servo_pulsewidth(ESC_PIN, NEUTRO)
    pi.set_servo_pulsewidth(SERVO_PIN, NEUTRO)
    time.sleep(2)

    threading.Thread(target=hilo_vision, daemon=True).start()
    threading.Thread(target=lambda: app.run(host='0.0.0.0', port=5000, debug=False, use_reloader=False), daemon=True).start()

    print(f"\nSISTEMA INICIADO - MODO ACTUAL: {MODO_ACTUAL}")

    try:
        while ejecutando:
            # --- MODO 1: RECTO (Seguidor de línea normal) ---
            if MODO_ACTUAL == MODO_RECTO:
                if pista_detectada:
                    pwm_dir = NEUTRO + (error_final * KP * SENTIDO)
                    pwm_dir = max(1200, min(1800, pwm_dir))
                    pi.set_servo_pulsewidth(SERVO_PIN, pwm_dir)
                    pi.set_servo_pulsewidth(ESC_PIN, VEL_AUTONOMA)
                else:
                    pi.set_servo_pulsewidth(ESC_PIN, NEUTRO)

            # --- MODOS 2 y 3: GIROS ---
            elif MODO_ACTUAL in [MODO_GIRO_IZQ, MODO_GIRO_DER]:
                print(f"Iniciando secuencia de giro. Esperando delay: {DELAY_CURVA}s")
                time.sleep(DELAY_CURVA)
                
                # Definir ángulo según el modo
                angulo_giro = PWM_GIRO_IZQ if MODO_ACTUAL == MODO_GIRO_IZQ else PWM_GIRO_DER
                
                # Ejecutar giro "ciego" hasta recuperar pista
                while ejecutando:
                    pi.set_servo_pulsewidth(SERVO_PIN, angulo_giro)
                    pi.set_servo_pulsewidth(ESC_PIN, VEL_AUTONOMA)
                    
                    # Condición de salida: Volver a ver 2 líneas paralelas (pista_detectada)
                    if pista_detectada:
                        print("Pista recuperada. Volviendo a MODO_RECTO")
                        MODO_ACTUAL = MODO_RECTO
                        break
                    time.sleep(0.02)

            time.sleep(0.02)

    except KeyboardInterrupt:
        ejecutando = False
    finally:
        pi.set_servo_pulsewidth(ESC_PIN, NEUTRO)
        pi.stop()

if __name__ == "__main__":
    control_maestro()
