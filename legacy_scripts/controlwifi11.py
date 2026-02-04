import pyrealsense2 as rs
import numpy as np
import cv2
import pigpio
import time
import threading
import socket
from flask import Flask, Response

# ==========================================
# 1. CONFIGURACIÓN Y PARÁMETROS
# ==========================================
app = Flask(__name__)
frame_streaming = None

# --- Control ---
KP_RECTA = 0.35
KP_CURVA = 1.5  # Más agresivo cuando falta una línea
SENTIDO = -1
VEL_AUTONOMA = 1550
NEUTRO = 1500

# --- Lógica de Carril ---
ANCHO_CARRIL_ESTIMADO = 320 # Pixeles aprox entre líneas
UMBRAL_CENTRO = 320         # Punto medio de la imagen (640/2)

# --- Scanlines ---
Y_INF = 380           
ALTO_SCAN_INF = 100    
Y_SUP = 300           
ALTO_SCAN_SUP = 79    

DILATACION_ITER = 2   
AREA_MIN_LINEA = 50   

# --- Colores (Azul) ---
AZUL_BAJO = np.array([82, 62, 110])
AZUL_ALTO = np.array([135, 255, 255])

# --- Hardware ---
ESC_PIN = 18
SERVO_PIN = 13
pi = pigpio.pi()

# ==========================================
# 2. PROCESAMIENTO DE VISIÓN MEJORADO
# ==========================================
ejecutando = True
error_final = 0
pista_detectada = False
modo_curva = False

def obtener_centro_inteligente(hsv_img, y, alto):
    roi = hsv_img[y : y + alto, :]
    mask = cv2.inRange(roi, AZUL_BAJO, AZUL_ALTO)
    mask = cv2.dilate(mask, None, iterations=DILATACION_ITER)
    
    cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    izq_puntos = []
    der_puntos = []

    for c in cnts:
        if cv2.contourArea(c) > AREA_MIN_LINEA:
            M = cv2.moments(c)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                if cx < UMBRAL_CENTRO:
                    izq_puntos.append(cx)
                else:
                    der_puntos.append(cx)
    
    # Lógica de decisión
    if izq_puntos and der_puntos:
        # Vemos ambas: modo normal
        centro = (max(izq_puntos) + min(der_puntos)) // 2
        return centro, mask, False 
    elif izq_puntos:
        # Solo vemos la izquierda: estimamos la derecha
        centro = max(izq_puntos) + (ANCHO_CARRIL_ESTIMADO // 2)
        return centro, mask, True
    elif der_puntos:
        # Solo vemos la derecha: estimamos la izquierda
        centro = min(der_puntos) - (ANCHO_CARRIL_ESTIMADO // 2)
        return centro, mask, True
    
    return None, mask, False

def hilo_vision():
    global ejecutando, error_final, pista_detectada, frame_streaming, modo_curva
    
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

            c_inf, m_inf, curva_inf = obtener_centro_inteligente(hsv, Y_INF, ALTO_SCAN_INF)
            c_sup, m_sup, curva_sup = obtener_centro_inteligente(hsv, Y_SUP, ALTO_SCAN_SUP)

            modo_curva = curva_inf or curva_sup
            img_viz = img.copy()

            if c_inf is not None and c_sup is not None:
                pista_detectada = True
                error_pos = c_inf - centro_cam
                error_inc = c_sup - c_inf
                error_final = error_pos + (error_inc * 0.6) # Mayor peso a la anticipación (superior)
                
                # Visualización
                color_linea = (0, 0, 255) if modo_curva else (0, 255, 0)
                cv2.line(img_viz, (c_inf, Y_INF), (c_sup, Y_SUP), color_linea, 3)
                if modo_curva:
                    cv2.putText(img_viz, "MODO CURVA - MEMORIA", (50, 50), 
                                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
            else:
                pista_detectada = False

            # Pintar máscaras en la vista previa
            img_viz[Y_INF : Y_INF + ALTO_SCAN_INF, :] = cv2.cvtColor(m_inf, cv2.COLOR_GRAY2BGR)
            img_viz[Y_SUP : Y_SUP + ALTO_SCAN_SUP, :] = cv2.cvtColor(m_sup, cv2.COLOR_GRAY2BGR)
            frame_streaming = img_viz

    finally:
        pipeline.stop()

# ==========================================
# 3. CONTROL Y SERVIDOR
# ==========================================
def gen_frames():
    global frame_streaming
    while True:
        if frame_streaming is not None:
            ret, buffer = cv2.imencode('.jpg', frame_streaming)
            yield (b'--frame\r\n' b'Content-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')
        else:
            time.sleep(0.1)

@app.route('/video_feed')
def video_feed():
    return Response(gen_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

def control_maestro():
    global ejecutando, error_final, pista_detectada, modo_curva
    
    pi.set_servo_pulsewidth(ESC_PIN, NEUTRO)
    pi.set_servo_pulsewidth(SERVO_PIN, NEUTRO)
    time.sleep(2)

    threading.Thread(target=hilo_vision, daemon=True).start()
    threading.Thread(target=lambda: app.run(host='0.0.0.0', port=5000, debug=False, use_reloader=False), daemon=True).start()

    print("SISTEMA INICIADO...")

    try:
        while ejecutando:
            if pista_detectada:
                # Usar un KP más fuerte si estamos estimando por pérdida de línea
                kp_actual = KP_CURVA if modo_curva else KP_RECTA
                
                pwm_dir = NEUTRO + (error_final * kp_actual * SENTIDO)
                pwm_dir = max(1100, min(1900, pwm_dir)) # Rango extendido para curvas cerradas
                
                pi.set_servo_pulsewidth(SERVO_PIN, pwm_dir)
                
                # Opcional: bajar velocidad un poco en modo curva
                vel_actual = VEL_AUTONOMA - 15 if modo_curva else VEL_AUTONOMA
                pi.set_servo_pulsewidth(ESC_PIN, vel_actual)
            else:
                # Parada total solo si no hay rastro de ninguna línea
                pi.set_servo_pulsewidth(ESC_PIN, NEUTRO)
            
            time.sleep(0.02)
    except KeyboardInterrupt:
        ejecutando = False
    finally:
        pi.set_servo_pulsewidth(ESC_PIN, NEUTRO)
        pi.stop()

if __name__ == "__main__":
    control_maestro()
