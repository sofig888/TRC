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
KP = 1.8              # Bajamos un poco para suavidad
KD = 0.5              # Derivativo para evitar oscilaciones
SENTIDO = 1
VEL_AUTONOMA = 1565 
GIRO_BUSQUEDA = 400   # Giro fuerte pero no al máximo para mantener tracción
TIEMPO_MAX_PERDIDO = 1.0 # Segundos antes de detenerse si no encuentra nada

# --- Scanlines ---
Y_INF = 400           
ALTO_SCAN_INF = 80    
Y_SUP = 320           
ALTO_SCAN_SUP = 80    
DILATACION_ITER = 1   
AREA_MIN_LINEA = 50   # Aumentado para ignorar ruido lejano

# --- Colores ---
AZUL_BAJO = np.array([90, 50, 50]) # Ajustado para ser más selectivo
AZUL_ALTO = np.array([130, 255, 255])

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
ultimo_lado = "centro"
tiempo_perdido = 0
last_error = 0

def obtener_punto_mejor_candidato(mask, centro_anterior):
    """Selecciona el contorno más lógico para evitar saltos a otros carriles."""
    cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    mejor_x = None
    min_distancia = 999
    
    for c in cnts:
        if cv2.contourArea(c) > AREA_MIN_LINEA:
            M = cv2.moments(c)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                # Priorizar el que esté más cerca del centro previo para evitar saltar a otra línea
                dist = abs(cx - centro_anterior)
                if dist < min_distancia:
                    min_distancia = dist
                    mejor_x = cx
    return mejor_x

def hilo_vision():
    global ejecutando, error_final, pista_detectada, frame_streaming, ultimo_lado, last_error
    
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    pipeline.start(config)

    centro_pantalla = 320

    try:
        while ejecutando:
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            if not color_frame: continue

            img = np.asanyarray(color_frame.get_data())
            hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

            # Procesar franjas
            roi_inf = hsv[Y_INF : Y_INF + ALTO_SCAN_INF, :]
            mask_inf = cv2.inRange(roi_inf, AZUL_BAJO, AZUL_ALTO)
            
            # Usamos el centro de la pantalla como referencia inicial
            c_inf = obtener_punto_mejor_candidato(mask_inf, centro_pantalla)

            img_viz = img.copy()

            if c_inf is not None:
                pista_detectada = True
                # Actualizar memoria de lado con un pequeño margen
                if c_inf < (centro_pantalla - 20): ultimo_lado = "izq"
                elif c_inf > (centro_pantalla + 20): ultimo_lado = "der"
                
                # Cálculo de error simple pero robusto
                error_actual = c_inf - centro_pantalla
                
                # Filtro para evitar volantazos: Si el cambio de error es absurdo, lo ignoramos
                if abs(error_actual - last_error) < 150: 
                    error_final = error_actual
                
                last_error = error_final
                cv2.circle(img_viz, (c_inf, Y_INF + 20), 10, (0, 255, 0), -1)
            else:
                pista_detectada = False

            # Visualización
            img_viz[Y_INF : Y_INF + ALTO_SCAN_INF, :] = cv2.cvtColor(mask_inf, cv2.COLOR_GRAY2BGR)
            frame_streaming = img_viz
    finally:
        pipeline.stop()

# ==========================================
# 4. CONTROL DE ESTADOS Y SEGURIDAD
# ==========================================
def control_maestro():
    global ejecutando, error_final, pista_detectada, ultimo_lado, tiempo_perdido
    
    pi.set_servo_pulsewidth(ESC_PIN, NEUTRO)
    pi.set_servo_pulsewidth(SERVO_PIN, NEUTRO)
    time.sleep(2)

    threading.Thread(target=hilo_vision, daemon=True).start()
    threading.Thread(target=lambda: app.run(host='0.0.0.0', port=5000), daemon=True).start()

    ts_inicio_perdida = None

    try:
        while ejecutando:
            if pista_detectada:
                ts_inicio_perdida = None # Reset temporizador
                
                # PID básico (P + D)
                dif_error = error_final - last_error
                pwm_dir = NEUTRO + (error_final * KP * SENTIDO) + (dif_error * KD)
                pwm_dir = max(1150, min(1850, pwm_dir))
                
                pi.set_servo_pulsewidth(SERVO_PIN, pwm_dir)
                pi.set_servo_pulsewidth(ESC_PIN, VEL_AUTONOMA)
            
            else:
                # --- LÓGICA DE BÚSQUEDA Y SEGURIDAD ---
                if ts_inicio_perdida is None:
                    ts_inicio_perdida = time.time()
                
                duracion_perdida = time.time() - ts_inicio_perdida

                if duracion_perdida > TIEMPO_MAX_PERDIDO:
                    # SEGURIDAD: Si se perdió mucho tiempo, detenerse
                    pi.set_servo_pulsewidth(ESC_PIN, NEUTRO)
                    pi.set_servo_pulsewidth(SERVO_PIN, NEUTRO)
                else:
                    # Todavía buscando: giro brusco en la última dirección vista
                    pi.set_servo_pulsewidth(ESC_PIN, VEL_AUTONOMA - 15) # Un poco más lento para buscar
                    if ultimo_lado == "izq":
                        pi.set_servo_pulsewidth(SERVO_PIN, NEUTRO - GIRO_BUSQUEDA)
                    else:
                        pi.set_servo_pulsewidth(SERVO_PIN, NEUTRO + GIRO_BUSQUEDA)
            
            time.sleep(0.02)
            
    except KeyboardInterrupt:
        ejecutando = False
    finally:
        pi.set_servo_pulsewidth(ESC_PIN, NEUTRO)
        pi.set_servo_pulsewidth(SERVO_PIN, NEUTRO)
        pi.stop()

if __name__ == "__main__":
    control_maestro()
