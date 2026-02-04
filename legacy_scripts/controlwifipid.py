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
frame_streaming = None  # Variable global para compartir el frame procesado

def gen_frames():
    global frame_streaming
    while True:
        if frame_streaming is not None:
            # Codificar el frame en JPEG para el streaming
            ret, buffer = cv2.imencode('.jpg', frame_streaming)
            frame_bytes = buffer.tobytes()
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')
        else:
            time.sleep(0.1)

@app.route('/video_feed')
def video_feed():
    return Response(gen_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

# ==========================================
# 2. PARÁMETROS DE CALIBRACIÓN Y HARDWARE
# ==========================================
KP = 3.0
SENTIDO = 1
ZONA_MUERTA = 10
VEL_AUTONOMA = 1580
AZUL_BAJO = np.array([82, 62, 110])
AZUL_ALTO = np.array([135, 255, 255])

ESC_PIN = 18; SERVO_PIN = 13
NEUTRO = 1500

pi = pigpio.pi()
ejecutando = True
error_final = 0
pista_detectada = False

# ==========================================
# 3. LÓGICA DE VISIÓN (SCANLINE)
# ==========================================
def obtener_centro_franja(hsv_img, y):
    roi = hsv_img[y : y + 8, :]
    mask = cv2.inRange(roi, AZUL_BAJO, AZUL_ALTO)
    cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    puntos = [int(cv2.moments(c)["m10"]/cv2.moments(c)["m00"]) for c in cnts if cv2.contourArea(c) > 50 and cv2.moments(c)["m00"] != 0]
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

            c_inf, m_inf = obtener_centro_franja(hsv, 400)
            c_sup, m_sup = obtener_centro_franja(hsv, 300)

            # Dibujar en la imagen para el streaming
            img_viz = img.copy()
            img_viz[400:428, :] = cv2.cvtColor(m_inf, cv2.COLOR_GRAY2BGR)
            img_viz[300:328, :] = cv2.cvtColor(m_sup, cv2.COLOR_GRAY2BGR)

            if c_inf is not None and c_sup is not None:
                pista_detectada = True
                error_final = (c_inf - centro_cam) + (c_sup - c_inf) * 0.5
                cv2.line(img_viz, (c_inf, 400), (c_sup, 300), (0, 255, 0), 2)
            else:
                pista_detectada = False

            # ACTUALIZAR EL FRAME PARA EL STREAMING
            frame_streaming = img_viz

    finally:
        pipeline.stop()

# ==========================================
# 4. CONTROL Y ARRANQUE
# ==========================================
def control_maestro():
    global ejecutando, error_final, pista_detectada
    
    pi.set_servo_pulsewidth(ESC_PIN, NEUTRO)
    pi.set_servo_pulsewidth(SERVO_PIN, NEUTRO)
    time.sleep(2)

    threading.Thread(target=hilo_vision, daemon=True).start()
    
    # Iniciar servidor Flask en un hilo separado
    threading.Thread(target=lambda: app.run(host='0.0.0.0', port=5000, debug=False, use_reloader=False), daemon=True).start()

    # Obtener IP local para mostrarla
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.connect(("8.8.8.8", 80))
    ip_local = s.getsockname()[0]
    s.close()

    print(f"\n" + "="*40)
    print(f"SISTEMA ONLINE - IP: {ip_local}")
    print(f"Ver streaming en: http://{ip_local}:5000/video_feed")
    print("="*40 + "\n")

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
