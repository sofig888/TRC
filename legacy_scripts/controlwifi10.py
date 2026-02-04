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
# 2. PARÁMETROS AJUSTABLES (MODIFICA AQUÍ)
# ==========================================
# --- Control ---
KP = 3.0
SENTIDO = 1
ZONA_MUERTA = 10

# Velocidad
VEL_AUTONOMA = 1560
VEL_GIRO = 1540          # velocidad un poco menor en curva fuerte (ajusta)
GIRO_UMBRAL = 120        # si |error| > esto, reduce velocidad (ajusta)

# --- Hug / Sesgo dentro del carril ---
# 0.50 = centro del carril
# 0.60 = más pegado a la derecha
# 0.65 = todavía más a la derecha
OBJETIVO_RECTO = 0.52
OBJETIVO_DERECHA = 0.62   # <- sube/baja esto para esos "3 cm" (0.60-0.68)
OBJETIVO_IZQUIERDA = 0.48

# Detección de “curva” con inclinación entre franjas
CURVA_UMBRAL = 25         # pixels: mientras más bajo, más fácil “decide curva”
PESO_INCLINACION = 0.55   # como ya traías (error_inc * 0.5)

# --- Suavizado de dirección (para que no sea brusco) ---
ALPHA_DIR = 0.7          # filtro 0-1 (más alto = responde más rápido)
MAX_DELTA_PWM = 30        # máximo cambio de PWM por ciclo (suaviza mucho)

# --- Scanlines (Las "Rebanadas") ---
Y_INF = 380
ALTO_SCAN_INF = 120

Y_SUP = 299
ALTO_SCAN_SUP = 80

DILATACION_ITER = 2
AREA_MIN_LINEA = 40

# --- Colores ---
AZUL_BAJO = np.array([82, 62, 110])
AZUL_ALTO = np.array([135, 255, 255])

# --- Hardware ---
ESC_PIN = 18; SERVO_PIN = 13
NEUTRO = 1500
PWM_MIN = 1200
PWM_MAX = 1800
pi = pigpio.pi()

# ==========================================
# 3. LÓGICA DE PROCESAMIENTO
# ==========================================
ejecutando = True
error_final = 0
pista_detectada = False

# guardamos cosas para debug/overlay
debug_info = {
    "c_inf": None, "c_sup": None,
    "l_inf": None, "r_inf": None,
    "l_sup": None, "r_sup": None,
    "target_inf": None, "target_sup": None,
    "curva": "?"
}

def obtener_datos_franja(hsv_img, y, alto):
    """
    Regresa:
      left_x, right_x, center_x, mask
    donde left_x es el punto más a la izquierda detectado, right_x el más a la derecha,
    y center_x = (left_x + right_x)/2
    """
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
        left_x = puntos[0]
        right_x = puntos[-1]
        center_x = (left_x + right_x) // 2
        return left_x, right_x, center_x, mask

    return None, None, None, mask

def clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v

def limitar_delta(actual, deseado, max_delta):
    d = deseado - actual
    if d > max_delta:
        return actual + max_delta
    if d < -max_delta:
        return actual - max_delta
    return deseado

def hilo_vision():
    global ejecutando, error_final, pista_detectada, frame_streaming, debug_info

    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    pipeline.start(config)

    try:
        while ejecutando:
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            if not color_frame:
                continue

            img = np.asanyarray(color_frame.get_data())
            hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
            centro_cam = img.shape[1] // 2

            l_inf, r_inf, c_inf, m_inf = obtener_datos_franja(hsv, Y_INF, ALTO_SCAN_INF)
            l_sup, r_sup, c_sup, m_sup = obtener_datos_franja(hsv, Y_SUP, ALTO_SCAN_SUP)

            img_viz = img.copy()
            img_viz[Y_INF : Y_INF + ALTO_SCAN_INF, :] = cv2.cvtColor(m_inf, cv2.COLOR_GRAY2BGR)
            img_viz[Y_SUP : Y_SUP + ALTO_SCAN_SUP, :] = cv2.cvtColor(m_sup, cv2.COLOR_GRAY2BGR)

            if c_inf is not None and c_sup is not None and l_inf is not None and r_inf is not None and l_sup is not None and r_sup is not None:
                pista_detectada = True

                # Detectar inclinación (curva) entre franjas
                error_pos = c_inf - centro_cam
                error_inc = c_sup - c_inf  # si cambia hacia la derecha/izquierda arriba vs abajo

                # --- decidir tipo de curva (para sesgar objetivo) ---
                if error_inc > CURVA_UMBRAL:
                    curva = "DERECHA"
                    frac_inf = OBJETIVO_DERECHA
                    frac_sup = OBJETIVO_DERECHA
                elif error_inc < -CURVA_UMBRAL:
                    curva = "IZQUIERDA"
                    frac_inf = OBJETIVO_IZQUIERDA
                    frac_sup = OBJETIVO_IZQUIERDA
                else:
                    curva = "RECTO"
                    frac_inf = OBJETIVO_RECTO
                    frac_sup = OBJETIVO_RECTO

                # --- target dentro del carril (hug) ---
                ancho_inf = max(1, (r_inf - l_inf))
                ancho_sup = max(1, (r_sup - l_sup))
                target_inf = int(l_inf + frac_inf * ancho_inf)
                target_sup = int(l_sup + frac_sup * ancho_sup)

                # Ahora el error es contra ese target (no contra el centro del carril)
                err_inf = target_inf - centro_cam
                err_inc = target_sup - target_inf

                # Combinas posición + “inclinación”
                error_final = err_inf + (err_inc * PESO_INCLINACION)

                # Debug overlay
                debug_info.update({
                    "c_inf": c_inf, "c_sup": c_sup,
                    "l_inf": l_inf, "r_inf": r_inf,
                    "l_sup": l_sup, "r_sup": r_sup,
                    "target_inf": target_inf, "target_sup": target_sup,
                    "curva": curva
                })

                # Dibujos: bordes y targets
                cv2.circle(img_viz, (l_inf, Y_INF + ALTO_SCAN_INF//2), 5, (0, 0, 255), -1)
                cv2.circle(img_viz, (r_inf, Y_INF + ALTO_SCAN_INF//2), 5, (0, 0, 255), -1)
                cv2.circle(img_viz, (l_sup, Y_SUP + ALTO_SCAN_SUP//2), 5, (0, 0, 255), -1)
                cv2.circle(img_viz, (r_sup, Y_SUP + ALTO_SCAN_SUP//2), 5, (0, 0, 255), -1)

                cv2.circle(img_viz, (target_inf, Y_INF + ALTO_SCAN_INF//2), 6, (0, 255, 255), -1)
                cv2.circle(img_viz, (target_sup, Y_SUP + ALTO_SCAN_SUP//2), 6, (0, 255, 255), -1)
                cv2.line(img_viz, (target_inf, Y_INF), (target_sup, Y_SUP), (0, 255, 0), 2)

                cv2.putText(img_viz, f"Curva: {curva}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,255,0), 2)
                cv2.putText(img_viz, f"err_final: {int(error_final)}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2)
            else:
                pista_detectada = False
                debug_info["curva"] = "SIN PISTA"

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

    # Suavizado de dirección
    pwm_dir_filtrado = NEUTRO

    try:
        while ejecutando:
            if pista_detectada:
                # zona muerta
                e = error_final
                if abs(e) < ZONA_MUERTA:
                    e = 0

                # Dirección “deseada”
                pwm_dir_deseado = NEUTRO + (e * KP * SENTIDO)
                pwm_dir_deseado = clamp(pwm_dir_deseado, PWM_MIN, PWM_MAX)

                # 1) limitar cambios bruscos (rate limit)
                pwm_dir_limitado = limitar_delta(pwm_dir_filtrado, pwm_dir_deseado, MAX_DELTA_PWM)

                # 2) filtro suave (low-pass)
                pwm_dir_filtrado = (1 - ALPHA_DIR) * pwm_dir_filtrado + ALPHA_DIR * pwm_dir_limitado

                pi.set_servo_pulsewidth(SERVO_PIN, int(pwm_dir_filtrado))

                # Velocidad adaptativa (opcional)
                if abs(e) > GIRO_UMBRAL:
                    pi.set_servo_pulsewidth(ESC_PIN, VEL_GIRO)
                else:
                    pi.set_servo_pulsewidth(ESC_PIN, VEL_AUTONOMA)
            else:
                pi.set_servo_pulsewidth(ESC_PIN, NEUTRO)
                pi.set_servo_pulsewidth(SERVO_PIN, NEUTRO)

            time.sleep(0.02)

    except KeyboardInterrupt:
        ejecutando = False
    finally:
        pi.set_servo_pulsewidth(ESC_PIN, NEUTRO)
        pi.set_servo_pulsewidth(SERVO_PIN, NEUTRO)
        pi.stop()

if __name__ == "__main__":
    control_maestro()
