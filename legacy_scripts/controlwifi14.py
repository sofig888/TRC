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
KP = 1.9
KD = 0.4
SENTIDO = 1
VEL_AUTONOMA = 1565 
GIRO_BUSQUEDA = 300   
TIEMPO_MAX_PERDIDO = 0.5 

# --- AJUSTE DE CARRIL ---
# Cuántos píxeles debe alejarse de la línea para estar en el "centro"
# Si tu carril es ancho, aumenta este valor (ej. 100 o 120)
OFFSET_CARRIL = 80

# --- Scanlines ---
Y_INF = 380           
ALTO_SCAN_INF = 100    
AREA_MIN_LINEA = 60   

# --- Colores ---
AZUL_BAJO = np.array([90, 60, 50]) 
AZUL_ALTO = np.array([135, 255, 255])

# --- Hardware ---
ESC_PIN = 18; SERVO_PIN = 13
NEUTRO = 1500
pi = pigpio.pi()

# ==========================================
# 3. LÓGICA DE VISIÓN MEJORADA
# ==========================================
ejecutando = True
error_final = 0
pista_detectada = False
ultimo_lado = "centro"
last_error = 0

def hilo_vision():
    global ejecutando, error_final, pista_detectada, frame_streaming, ultimo_lado, last_error
    
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    pipeline.start(config)

    centro_pantalla = 320 # Centro físico de la cámara

    try:
        while ejecutando:
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            if not color_frame: continue

            img = np.asanyarray(color_frame.get_data())
            hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

            # Máscara de la franja inferior
            roi_inf = hsv[Y_INF : Y_INF + ALTO_SCAN_INF, :]
            mask_inf = cv2.inRange(roi_inf, AZUL_BAJO, AZUL_ALTO)
            mask_inf = cv2.dilate(mask_inf, None, iterations=1)
            
            cnts, _ = cv2.findContours(mask_inf, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            puntos_detectados = []
            for c in cnts:
                if cv2.contourArea(c) > AREA_MIN_LINEA:
                    M = cv2.moments(c)
                    if M["m00"] != 0:
                        puntos_detectados.append(int(M["m10"] / M["m00"]))
            
            puntos_detectados.sort()

            img_viz = img.copy()
            target_x = centro_pantalla # Por defecto, ir al centro

            if len(puntos_detectados) > 0:
                pista_detectada = True
                
                # CASO A: Vemos dos líneas (El ideal)
                if len(puntos_detectados) >= 2:
                    # El objetivo es el promedio exacto entre la línea más a la izq y la más a la der
                    linea_izq = puntos_detectados[0]
                    linea_der = puntos_detectados[-1]
                    target_x = (linea_izq + linea_der) // 2
                    ultimo_lado = "centro"
                    cv2.putText(img_viz, "MODO: DOBLE LINEA", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                
                # CASO B: Solo vemos una línea
                else:
                    linea_unica = puntos_detectados[0]
                    # ¿Es la línea izquierda o la derecha?
                    if linea_unica < centro_pantalla:
                        # Es la izquierda -> El objetivo está a la derecha de la línea
                        target_x = linea_unica + OFFSET_CARRIL
                        ultimo_lado = "izq"
                        cv2.putText(img_viz, "MODO: SIGUIENDO IZQ", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
                    else:
                        # Es la derecha -> El objetivo está a la izquierda de la línea
                        target_x = linea_unica - OFFSET_CARRIL
                        ultimo_lado = "der"
                        cv2.putText(img_viz, "MODO: SIGUIENDO DER", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)

                # Calcular error respecto al objetivo calculado (no necesariamente el centro de la cámara)
                error_actual = target_x - centro_pantalla
                
                # Filtro de suavizado
                if abs(error_actual - last_error) < 180:
                    error_final = error_actual
                last_error = error_final

                # Visualización del objetivo
                cv2.circle(img_viz, (target_x, Y_INF + 20), 15, (0, 0, 255), 2) # Círculo rojo = donde queremos ir
                for p in puntos_detectados:
                    cv2.circle(img_viz, (p, Y_INF + 20), 8, (255, 0, 0), -1) # Puntos azules = líneas vistas
            else:
                pista_detectada = False

            img_viz[Y_INF : Y_INF + ALTO_SCAN_INF, :] = cv2.cvtColor(mask_inf, cv2.COLOR_GRAY2BGR)
            frame_streaming = img_viz

    finally:
        pipeline.stop()

# ==========================================
# 4. CONTROL Y SEGURIDAD
# ==========================================
def control_maestro():
    global ejecutando, error_final, pista_detectada, ultimo_lado
    
    pi.set_servo_pulsewidth(ESC_PIN, NEUTRO)
    pi.set_servo_pulsewidth(SERVO_PIN, NEUTRO)
    time.sleep(2)

    threading.Thread(target=hilo_vision, daemon=True).start()
    threading.Thread(target=lambda: app.run(host='0.0.0.0', port=5000), daemon=True).start()

    ts_inicio_perdida = None
    last_p_error = 0

    try:
        while ejecutando:
            if pista_detectada:
                ts_inicio_perdida = None
                
                # PID de dirección
                d_error = error_final - last_p_error
                pwm_dir = NEUTRO + (error_final * KP * SENTIDO) + (d_error * KD)
                last_p_error = error_final
                
                pwm_dir = max(1150, min(1850, pwm_dir))
                pi.set_servo_pulsewidth(SERVO_PIN, pwm_dir)
                pi.set_servo_pulsewidth(ESC_PIN, VEL_AUTONOMA)
            
            else:
                # Búsqueda y parada de seguridad
                if ts_inicio_perdida is None: ts_inicio_perdida = time.time()
                
                if (time.time() - ts_inicio_perdida) > TIEMPO_MAX_PERDIDO:
                    pi.set_servo_pulsewidth(ESC_PIN, NEUTRO)
                    pi.set_servo_pulsewidth(SERVO_PIN, NEUTRO)
                else:
                    pi.set_servo_pulsewidth(ESC_PIN, VEL_AUTONOMA - 10)
                    if ultimo_lado == "izq":
                        pi.set_servo_pulsewidth(SERVO_PIN, NEUTRO - GIRO_BUSQUEDA)
                    else:
                        pi.set_servo_pulsewidth(SERVO_PIN, NEUTRO + GIRO_BUSQUEDA)
            
            time.sleep(0.02)
            
    except KeyboardInterrupt:
        ejecutando = False
    finally:
        pi.set_servo_pulsewidth(ESC_PIN, NEUTRO); pi.stop()

if __name__ == "__main__":
    control_maestro()
