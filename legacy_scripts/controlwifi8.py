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
KP = 8.0
SENTIDO = 1
ZONA_MUERTA_VISUAL = 1  # Pixeles de tolerancia para decir que está "Recto"
VEL_AUTONOMA = 1550

Y_INF = 360           
ALTO_SCAN_INF = 10    
Y_SUP = 279           
ALTO_SCAN_SUP = 10    

DILATACION_ITER = 2   
AREA_MIN_LINEA = 20   

AZUL_BAJO = np.array([82, 62, 110])
AZUL_ALTO = np.array([135, 255, 255])

ESC_PIN = 18; SERVO_PIN = 13
NEUTRO = 1500
pi = pigpio.pi()

# ==========================================
# 3. LÓGICA DE PROCESAMIENTO
# ==========================================
ejecutando = True
error_final = 0
pista_detectada = False
estado_conduccion = "BUSCANDO..." # Nueva variable para el texto

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
    global ejecutando, error_final, pista_detectada, frame_streaming, estado_conduccion
    
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
                error_final = error_pos + (error_inc * 0.30)
                
                # --- Lógica de Detección de Modo ---
                # Usamos error_final para determinar hacia dónde apunta el cálculo
                if abs(error_final) < ZONA_MUERTA_VISUAL:
                    estado_conduccion = "MODO: RECTA"
                    color_texto = (0, 255, 0) # Verde
                elif error_final * SENTIDO > 0:
                    estado_conduccion = "MODO: GIRO DER"
                    color_texto = (0, 255, 255) # Amarillo
                else:
                    estado_conduccion = "MODO: GIRO IZQ"
                    color_texto = (0, 165, 255) # Naranja
                
                cv2.line(img_viz, (c_inf, Y_INF), (c_sup, Y_SUP), color_texto, 3)
            else:
                pista_detectada = False
                estado_conduccion = "PISTA PERDIDA"
                color_texto = (0, 0, 255) # Rojo

            # --- Dibujar en Pantalla ---
            cv2.putText(img_viz, estado_conduccion, (20, 50), 
                        cv2.FONT_HERSHEY_SIMPLEX, 1, color_texto, 2, cv2.LINE_AA)
            
            frame_streaming = img_viz
    finally:
        pipeline.stop()

# ==========================================
# 4. CONTROL Y SERVIDOR
# ==========================================
def control_maestro():
    global ejecutando, error_final, pista_detectada
    
    pi.set_servo_pulsewidth(ESC_PIN, NEUTRO)
    pi.set_servo_pulsewidth(SERVO_PIN, NEUTRO)
    time.sleep(2)

    threading.Thread(target=hilo_vision, daemon=True).start()
    threading.Thread(target=lambda: app.run(host='0.0.0.0', port=5000, debug=False, use_reloader=False), daemon=True).start()

    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.connect(("8.8.8.8", 80))
    ip_local = s.getsockname()[0]
    s.close()

    print(f"\nSISTEMA ACTIVO EN: http://{ip_local}:5000/video_feed")

    try:
        while ejecutando:
            if pista_detectada:
                pwm_dir = NEUTRO + (error_final * KP * SENTIDO)
                pwm_dir = max(1200, min(1800, pwm_dir))
                pi.set_servo_pulsewidth(SERVO_PIN, pwm_dir)
                pi.set_servo_pulsewidth(ESC_PIN, VEL_AUTONOMA)
            else:
                pi.set_servo_pulsewidth(ESC_PIN, NEUTRO)
            time.sleep(0.02)
    except KeyboardInterrupt:
        ejecutando = False
    finally:
        pi.set_servo_pulsewidth(ESC_PIN, NEUTRO)
        pi.stop()

if __name__ == "__main__":
    control_maestro()
