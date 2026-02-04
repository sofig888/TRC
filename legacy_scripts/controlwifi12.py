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
KP = 3.0            # Sensibilidad en línea recta/curvas suaves
SENTIDO = 1
VEL_AUTONOMA = 1565 # Un poco más lento ayuda a no perder la línea en curvas cerradas
GIRO_BUSQUEDA = 400 # Cuánto girar el servo (offset del neutRO) cuando se pierde la línea

# --- Scanlines ---
Y_INF = 430           
ALTO_SCAN_INF = 50    
Y_SUP = 359           
ALTO_SCAN_SUP = 70    
DILATACION_ITER = 2   
AREA_MIN_LINEA = 30   

# --- Colores ---
AZUL_BAJO = np.array([82, 62, 110])
AZUL_ALTO = np.array([135, 255, 255])

# --- Hardware ---
ESC_PIN = 18; SERVO_PIN = 13
NEUTRO = 1500
pi = pigpio.pi()

# ==========================================
# 3. LÓGICA DE PROCESAMIENTO Y ESTADOS
# ==========================================
ejecutando = True
error_final = 0
pista_detectada = False
ultimo_lado = "centro" # "izq", "der", "centro"

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
    global ejecutando, error_final, pista_detectada, frame_streaming, ultimo_lado
    
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
            # Dibujar línea divisoria central para debug visual
            cv2.line(img_viz, (centro_cam, 0), (centro_cam, 480), (255, 255, 0), 1)

            if c_inf is not None:
                pista_detectada = True
                # Determinar de qué lado está la línea antes de procesar
                if c_inf < (centro_cam - 30): ultimo_lado = "izq"
                elif c_inf > (centro_cam + 30): ultimo_lado = "der"
                
                error_pos = c_inf - centro_cam
                error_inc = (c_sup - c_inf) if c_sup is not None else 0
                error_final = error_pos + (error_inc * 0.6)
                
                cv2.circle(img_viz, (c_inf, Y_INF + 25), 10, (0, 255, 0), -1)
            else:
                pista_detectada = False

            # Visualización de máscaras
            img_viz[Y_INF : Y_INF + ALTO_SCAN_INF, :] = cv2.cvtColor(m_inf, cv2.COLOR_GRAY2BGR)
            frame_streaming = img_viz
    finally:
        pipeline.stop()

# ==========================================
# 4. CONTROL MAESTRO (ESTADOS)
# ==========================================
def control_maestro():
    global ejecutando, error_final, pista_detectada, ultimo_lado
    
    pi.set_servo_pulsewidth(ESC_PIN, NEUTRO)
    pi.set_servo_pulsewidth(SERVO_PIN, NEUTRO)
    time.sleep(2)

    threading.Thread(target=hilo_vision, daemon=True).start()
    threading.Thread(target=lambda: app.run(host='0.0.0.0', port=5000, debug=False, use_reloader=False), daemon=True).start()

    print("SISTEMA INICIADO - BUSCANDO LÍNEA")

    try:
        while ejecutando:
            if pista_detectada:
                # --- MODO NORMAL (SEGUIMIENTO) ---
                pwm_dir = NEUTRO + (error_final * KP * SENTIDO)
                # Limitar para no forzar el servo físicamente
                pwm_dir = max(1100, min(1900, pwm_dir))
                
                pi.set_servo_pulsewidth(SERVO_PIN, pwm_dir)
                pi.set_servo_pulsewidth(ESC_PIN, VEL_AUTONOMA)
            else:
                # --- MODO BÚSQUEDA (GIRO BRUSCO) ---
                # No detenemos el motor, pero podemos bajar un poco la velocidad si prefieres
                pi.set_servo_pulsewidth(ESC_PIN, VEL_AUTONOMA - 20) 
                
                if ultimo_lado == "izq":
                    # Giro brusco a la izquierda
                    pi.set_servo_pulsewidth(SERVO_PIN, NEUTRO - GIRO_BUSQUEDA)
                elif ultimo_lado == "der":
                    # Giro brusco a la derecha
                    pi.set_servo_pulsewidth(SERVO_PIN, NEUTRO + GIRO_BUSQUEDA)
                else:
                    # Si nunca vio nada, se queda recto buscando
                    pi.set_servo_pulsewidth(SERVO_PIN, NEUTRO)
            
            time.sleep(0.01) # Ciclo de control más rápido (100Hz)
            
    except KeyboardInterrupt:
        ejecutando = False
    finally:
        pi.set_servo_pulsewidth(ESC_PIN, NEUTRO)
        pi.set_servo_pulsewidth(SERVO_PIN, NEUTRO)
        pi.stop()

if __name__ == "__main__":
    control_maestro()
