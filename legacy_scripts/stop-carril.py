import pyrealsense2 as rs
import numpy as np
import cv2
import pigpio
import sys
import tty
import termios
import time
import threading
import select

# ==========================================
# CONFIGURACIÓN DE HARDWARE (GPIO)
# ==========================================
ESC_PIN = 18
SERVO_PIN = 13
LED_BLANCO = 26
LED_ROJO = 27
LED_GIRO_IZQ = 22
LED_GIRO_DER = 23

# Valores PWM
NEUTRO = 1500
MAX_AVANCE = 2000
MAX_REVERSA = 1300
PASO_VELOCIDAD = 20
IZQ = 1200
DER = 1800
PASO_DIRECCION = 80

# ==========================================
# CONFIGURACIÓN VISIÓN Y PERSISTENCIA
# ==========================================
PERSISTENCIA_LIMITE = 15
DISTANCIA_SEGURA = 1.8
DISTANCIA_PARADA = 0.3
DISTANCIA_INTERMITENTES = 0.6  # 60 cm

memoria_stop = {
    "contador": PERSISTENCIA_LIMITE,
    "distancia": 0.0,
    "box": None,
    "puntos": None
}

# ==========================================
# MÁSCARA PARA CARRIL AZUL (AJUSTABLE)
# ==========================================
# Azul típico (cinta aislante azul). Ajusta si hace falta:
AZUL_LO = np.array([90, 80, 40])    # H,S,V
AZUL_HI = np.array([135, 255, 255])

KERNEL = np.ones((5, 5), np.uint8)

# ==========================================
# VARIABLES DE ESTADO GLOBAL
# ==========================================
pi = pigpio.pi()
velocidad_actual = NEUTRO
direccion_actual = NEUTRO
luces_blancas_on = False
intermitentes_emergencia = False
ejecutando = True

# Inicializar Pines
for p in [LED_BLANCO, LED_ROJO, LED_GIRO_IZQ, LED_GIRO_DER]:
    pi.set_mode(p, pigpio.OUTPUT)
    pi.write(p, 0)

# ==========================================
# FUNCIONES DE APOYO
# ==========================================
def obtener_distancia_robusta(depth_frame, cx, cy):
    dist = depth_frame.get_distance(cx, cy)
    if dist > 0:
        return dist
    for dx in range(-3, 4, 3):
        for dy in range(-3, 4, 3):
            val = depth_frame.get_distance(cx + dx, cy + dy)
            if val > 0:
                return val
    return 0

def hilo_intermitentes():
    global ejecutando, direccion_actual, intermitentes_emergencia
    while ejecutando:
        if intermitentes_emergencia:
            estado = not pi.read(LED_GIRO_IZQ)
            pi.write(LED_GIRO_IZQ, estado)
            pi.write(LED_GIRO_DER, estado)
        elif direccion_actual < NEUTRO:
            pi.write(LED_GIRO_IZQ, not pi.read(LED_GIRO_IZQ))
            pi.write(LED_GIRO_DER, 0)
        elif direccion_actual > NEUTRO:
            pi.write(LED_GIRO_DER, not pi.read(LED_GIRO_DER))
            pi.write(LED_GIRO_IZQ, 0)
        else:
            pi.write(LED_GIRO_IZQ, 0)
            pi.write(LED_GIRO_DER, 0)
        time.sleep(0.4)

def is_data():
    return select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], [])

# ==========================================
# HILO DE CONTROL DE USUARIO (TECLADO)
# ==========================================
def hilo_control_teclado():
    global velocidad_actual, direccion_actual, ejecutando, luces_blancas_on, intermitentes_emergencia

    old_settings = termios.tcgetattr(sys.stdin)
    last_s_press = 0

    try:
        tty.setcbreak(sys.stdin.fileno())
        while ejecutando:
            if time.time() - last_s_press > 0.7:
                pi.write(LED_ROJO, 0)

            if is_data():
                char = sys.stdin.read(1).lower()
                if char == 's':
                    pi.write(LED_ROJO, 1)
                    last_s_press = time.time()
                    if velocidad_actual > MAX_REVERSA:
                        velocidad_actual -= PASO_VELOCIDAD
                elif char == 'w':
                    if velocidad_actual < MAX_AVANCE:
                        velocidad_actual += PASO_VELOCIDAD
                elif char == 'a':
                    direccion_actual = max(IZQ, direccion_actual - PASO_DIRECCION)
                elif char == 'd':
                    direccion_actual = min(DER, direccion_actual + PASO_DIRECCION)
                elif char == 'l':
                    luces_blancas_on = not luces_blancas_on
                    pi.write(LED_BLANCO, 1 if luces_blancas_on else 0)
                elif char == 'i':
                    intermitentes_emergencia = not intermitentes_emergencia
                elif char == ' ':
                    velocidad_actual = NEUTRO
                    direccion_actual = NEUTRO
                elif char == 'q':
                    ejecutando = False

            # --- LÓGICA DE SEGURIDAD (VELOCIDAD POR DISTANCIA) ---
            if memoria_stop["contador"] < PERSISTENCIA_LIMITE:
                d = memoria_stop["distancia"]
                if 0 < d <= DISTANCIA_PARADA:
                    velocidad_actual = NEUTRO
                elif DISTANCIA_PARADA < d < DISTANCIA_SEGURA:
                    max_permitida = 1500 + (d - DISTANCIA_PARADA) * ((2000 - 1500) / (DISTANCIA_SEGURA - DISTANCIA_PARADA))
                    if velocidad_actual > max_permitida:
                        velocidad_actual = int(max_permitida)

            pi.set_servo_pulsewidth(ESC_PIN, velocidad_actual)
            pi.set_servo_pulsewidth(SERVO_PIN, direccion_actual)
            time.sleep(0.01)

    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

# ==========================================
# BUCLE PRINCIPAL (VISIÓN)
# ==========================================
def main():
    global ejecutando, intermitentes_emergencia

    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

    profile = pipeline.start(config)

    # Ajustes manuales cámara (como tenías)
    color_sensor = profile.get_device().query_sensors()[1]
    color_sensor.set_option(rs.option.enable_auto_exposure, 0)
    color_sensor.set_option(rs.option.exposure, 150)
    color_sensor.set_option(rs.option.gain, 64)

    align = rs.align(rs.stream.color)

    threading.Thread(target=hilo_intermitentes, daemon=True).start()
    threading.Thread(target=hilo_control_teclado, daemon=True).start()

    try:
        while ejecutando:
            frames = pipeline.wait_for_frames()
            aligned = align.process(frames)
            color_f = aligned.get_color_frame()
            depth_f = aligned.get_depth_frame()
            if not color_f or not depth_f:
                continue

            img = np.asanyarray(color_f.get_data())
            hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

            # ==========================
            # 1) STOP (ROJO) - igual que tenías
            # ==========================
            m1 = cv2.inRange(hsv, np.array([0, 130, 50]), np.array([10, 255, 255]))
            m2 = cv2.inRange(hsv, np.array([160, 130, 50]), np.array([180, 255, 255]))
            mask_stop = cv2.dilate(cv2.addWeighted(m1, 1.0, m2, 1.0, 0), KERNEL)

            contours, _ = cv2.findContours(mask_stop, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            hallado = False

            for cnt in contours:
                if cv2.contourArea(cnt) < 600:
                    continue
                approx = cv2.approxPolyDP(cnt, 0.02 * cv2.arcLength(cnt, True), True)

                if len(approx) == 8 and cv2.isContourConvex(approx):
                    x, y, w, h = cv2.boundingRect(approx)
                    if 0.7 <= float(w) / h <= 1.3:
                        dist = obtener_distancia_robusta(depth_f, x + w // 2, y + h // 2)
                        memoria_stop.update({"contador": 0, "distancia": dist, "box": (x, y, w, h), "puntos": approx})
                        hallado = True
                        break

            if not hallado:
                memoria_stop["contador"] += 1

            # Intermitentes automáticas por distancia STOP
            if memoria_stop["contador"] < PERSISTENCIA_LIMITE:
                if 0 < memoria_stop["distancia"] < DISTANCIA_INTERMITENTES:
                    intermitentes_emergencia = True
                else:
                    intermitentes_emergencia = False
            else:
                intermitentes_emergencia = False

            # Dibujo (STOP)
            img_draw = img.copy()
            if memoria_stop["contador"] < PERSISTENCIA_LIMITE:
                x, y, w, h = memoria_stop["box"]
                color = (0, 255, 0) if hallado else (0, 255, 255)
                cv2.drawContours(img_draw, [memoria_stop["puntos"]], 0, color, 3)
                cv2.putText(img_draw, f"STOP: {memoria_stop['distancia']:.2f}m",
                            (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

            # ==========================
            # 2) CARRIL (AZUL) -> máscara blanco/negro
            # ==========================
            mask_azul = cv2.inRange(hsv, AZUL_LO, AZUL_HI)

            # Limpieza básica para que no salga “ruido”
            mask_azul = cv2.medianBlur(mask_azul, 5)
            mask_azul = cv2.morphologyEx(mask_azul, cv2.MORPH_OPEN, KERNEL, iterations=1)
            mask_azul = cv2.morphologyEx(mask_azul, cv2.MORPH_CLOSE, KERNEL, iterations=2)

            # ==========================
            # 3) PARTIR VENTANA:
            # Arriba: mitad superior de la imagen normal
            # Abajo: mitad inferior de la máscara azul (blanco/negro)
            # ==========================
            H, W = img_draw.shape[:2]
            mitad = H // 2

            top = img_draw[:mitad, :, :]  # imagen
            bottom_mask = mask_azul[mitad:, :]  # máscara azul (mitad inferior)

            # convertir máscara a BGR para poder apilar con imagen
            bottom_mask_bgr = cv2.cvtColor(bottom_mask, cv2.COLOR_GRAY2BGR)

            vista = np.vstack([top, bottom_mask_bgr])

            cv2.imshow("Control Inteligente Pi4B", vista)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                ejecutando = False
                break

    finally:
        ejecutando = False
        pipeline.stop()
        for p in [LED_BLANCO, LED_ROJO, LED_GIRO_IZQ, LED_GIRO_DER]:
            pi.write(p, 0)
        pi.set_servo_pulsewidth(ESC_PIN, NEUTRO)
        pi.set_servo_pulsewidth(SERVO_PIN, NEUTRO)
        pi.stop()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
import pyrealsense2 as rs
import numpy as np
import cv2
import pigpio
import sys
import tty
import termios
import time
import threading
import select

# ==========================================
# CONFIGURACIÓN DE HARDWARE (GPIO)
# ==========================================
ESC_PIN = 18
SERVO_PIN = 13
LED_BLANCO = 26
LED_ROJO = 27
LED_GIRO_IZQ = 22
LED_GIRO_DER = 23

# Valores PWM
NEUTRO = 1500
MAX_AVANCE = 2000
MAX_REVERSA = 1300
PASO_VELOCIDAD = 20
IZQ = 1200
DER = 1800
PASO_DIRECCION = 80

# ==========================================
# CONFIGURACIÓN VISIÓN Y PERSISTENCIA
# ==========================================
PERSISTENCIA_LIMITE = 15
DISTANCIA_SEGURA = 1.8
DISTANCIA_PARADA = 0.3
DISTANCIA_INTERMITENTES = 0.6  # 60 cm

memoria_stop = {
    "contador": PERSISTENCIA_LIMITE,
    "distancia": 0.0,
    "box": None,
    "puntos": None
}

# ==========================================
# VARIABLES DE ESTADO GLOBAL
# ==========================================
pi = pigpio.pi()
velocidad_actual = NEUTRO
direccion_actual = NEUTRO
luces_blancas_on = False
intermitentes_emergencia = False
ejecutando = True

# Inicializar Pines
for p in [LED_BLANCO, LED_ROJO, LED_GIRO_IZQ, LED_GIRO_DER]:
    pi.set_mode(p, pigpio.OUTPUT)
    pi.write(p, 0)

# ==========================================
# FUNCIONES DE APOYO
# ==========================================
def obtener_distancia_robusta(depth_frame, cx, cy):
    dist = depth_frame.get_distance(cx, cy)
    if dist > 0:
        return dist
    for dx in range(-3, 4, 3):
        for dy in range(-3, 4, 3):
            val = depth_frame.get_distance(cx + dx, cy + dy)
            if val > 0:
                return val
    return 0

def hilo_intermitentes():
    global ejecutando, direccion_actual, intermitentes_emergencia
    while ejecutando:
        # Prioridad 1: Emergencia
        if intermitentes_emergencia:
            estado = not pi.read(LED_GIRO_IZQ)
            pi.write(LED_GIRO_IZQ, estado)
            pi.write(LED_GIRO_DER, estado)
        # Prioridad 2: Giro
        elif direccion_actual < NEUTRO:
            pi.write(LED_GIRO_IZQ, not pi.read(LED_GIRO_IZQ))
            pi.write(LED_GIRO_DER, 0)
        elif direccion_actual > NEUTRO:
            pi.write(LED_GIRO_DER, not pi.read(LED_GIRO_DER))
            pi.write(LED_GIRO_IZQ, 0)
        else:
            pi.write(LED_GIRO_IZQ, 0)
            pi.write(LED_GIRO_DER, 0)
        time.sleep(0.4)

def is_data():
    return select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], [])

# ==========================================
# HILO DE CONTROL DE USUARIO (TECLADO)
# ==========================================
def hilo_control_teclado():
    global velocidad_actual, direccion_actual, ejecutando, luces_blancas_on, intermitentes_emergencia

    old_settings = termios.tcgetattr(sys.stdin)
    last_s_press = 0

    try:
        tty.setcbreak(sys.stdin.fileno())
        while ejecutando:
            if time.time() - last_s_press > 0.7:
                pi.write(LED_ROJO, 0)

            if is_data():
                char = sys.stdin.read(1).lower()
                if char == 's':
                    pi.write(LED_ROJO, 1)
                    last_s_press = time.time()
                    if velocidad_actual > MAX_REVERSA:
                        velocidad_actual -= PASO_VELOCIDAD
                elif char == 'w':
                    if velocidad_actual < MAX_AVANCE:
                        velocidad_actual += PASO_VELOCIDAD
                elif char == 'a':
                    direccion_actual = max(IZQ, direccion_actual - PASO_DIRECCION)
                elif char == 'd':
                    direccion_actual = min(DER, direccion_actual + PASO_DIRECCION)
                elif char == 'l':
                    luces_blancas_on = not luces_blancas_on
                    pi.write(LED_BLANCO, 1 if luces_blancas_on else 0)
                elif char == 'i':
                    intermitentes_emergencia = not intermitentes_emergencia
                elif char == ' ':
                    velocidad_actual = NEUTRO
                    direccion_actual = NEUTRO
                elif char == 'q':
                    ejecutando = False

            # --- LÓGICA DE SEGURIDAD (VELOCIDAD POR DISTANCIA) ---
            if memoria_stop["contador"] < PERSISTENCIA_LIMITE:
                d = memoria_stop["distancia"]
                if 0 < d <= DISTANCIA_PARADA:
                    velocidad_actual = NEUTRO
                elif DISTANCIA_PARADA < d < DISTANCIA_SEGURA:
                    max_permitida = 1500 + (d - DISTANCIA_PARADA) * ((2000 - 1500) / (DISTANCIA_SEGURA - DISTANCIA_PARADA))
                    if velocidad_actual > max_permitida:
                        velocidad_actual = int(max_permitida)

            pi.set_servo_pulsewidth(ESC_PIN, velocidad_actual)
            pi.set_servo_pulsewidth(SERVO_PIN, direccion_actual)
            time.sleep(0.01)

    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

# ==========================================
# BUCLE PRINCIPAL (VISIÓN)
# ==========================================
def main():
    global ejecutando, intermitentes_emergencia

    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

    profile = pipeline.start(config)
    color_sensor = profile.get_device().query_sensors()[1]
    color_sensor.set_option(rs.option.enable_auto_exposure, 0)
    color_sensor.set_option(rs.option.exposure, 150)
    color_sensor.set_option(rs.option.gain, 64)

    align = rs.align(rs.stream.color)

    threading.Thread(target=hilo_intermitentes, daemon=True).start()
    threading.Thread(target=hilo_control_teclado, daemon=True).start()

    try:
        while ejecutando:
            frames = pipeline.wait_for_frames()
            aligned = align.process(frames)
            color_f = aligned.get_color_frame()
            depth_f = aligned.get_depth_frame()
            if not color_f or not depth_f:
                continue

            img = np.asanyarray(color_f.get_data())
            hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

            # Máscara rojo (STOP)
            m1 = cv2.inRange(hsv, np.array([0, 130, 50]), np.array([10, 255, 255]))
            m2 = cv2.inRange(hsv, np.array([160, 130, 50]), np.array([180, 255, 255]))
            mask = cv2.dilate(cv2.addWeighted(m1, 1.0, m2, 1.0, 0), np.ones((5, 5), np.uint8))

            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            hallado = False

            for cnt in contours:
                if cv2.contourArea(cnt) < 600:
                    continue
                approx = cv2.approxPolyDP(cnt, 0.02 * cv2.arcLength(cnt, True), True)

                if len(approx) == 8 and cv2.isContourConvex(approx):
                    x, y, w, h = cv2.boundingRect(approx)
                    if 0.7 <= float(w) / h <= 1.3:
                        dist = obtener_distancia_robusta(depth_f, x + w // 2, y + h // 2)
                        memoria_stop.update({"contador": 0, "distancia": dist, "box": (x, y, w, h), "puntos": approx})
                        hallado = True
                        break

            if not hallado:
                memoria_stop["contador"] += 1

            # --- LÓGICA DE INTERMITENTES AUTOMÁTICAS ---
            if memoria_stop["contador"] < PERSISTENCIA_LIMITE:
                if 0 < memoria_stop["distancia"] < DISTANCIA_INTERMITENTES:
                    intermitentes_emergencia = True
                else:
                    intermitentes_emergencia = False
            else:
                intermitentes_emergencia = False

            # Dibujo sobre imagen superior
            img_draw = img.copy()
            if memoria_stop["contador"] < PERSISTENCIA_LIMITE:
                x, y, w, h = memoria_stop["box"]
                color = (0, 255, 0) if hallado else (0, 255, 255)
                cv2.drawContours(img_draw, [memoria_stop["puntos"]], 0, color, 3)
                cv2.putText(img_draw, f"STOP: {memoria_stop['distancia']:.2f}m", (x, y - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

            # ==========================
            # VISUAL: PARTIR PANTALLA
            # Arriba: imagen
            # Abajo: máscara en AZUL
            # ==========================
            h, w = img_draw.shape[:2]
            mitad = h // 2

            # arriba: mitad superior de la imagen dibujada
            top = img_draw[:mitad, :, :]

            # abajo: mitad inferior, mostrar máscara en azul (BGR: azul en canal B)
            mask_bottom = mask[mitad:, :]  # misma mitad inferior
            blue = np.zeros((h - mitad, w, 3), dtype=np.uint8)
            blue[:, :, 0] = mask_bottom    # canal azul

            # Unir verticalmente
            vista = np.vstack([top, blue])

            cv2.imshow("Control Inteligente Pi4B", vista)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                ejecutando = False
                break

    finally:
        ejecutando = False
        pipeline.stop()
        for p in [LED_BLANCO, LED_ROJO, LED_GIRO_IZQ, LED_GIRO_DER]:
            pi.write(p, 0)
        pi.set_servo_pulsewidth(ESC_PIN, NEUTRO)
        pi.set_servo_pulsewidth(SERVO_PIN, NEUTRO)
        pi.stop()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()

import pyrealsense2 as rs
import numpy as np
import cv2
import pigpio
import time

# ==========================================
# GPIO / PWM
# ==========================================
ESC_PIN = 18
SERVO_PIN = 13

LED_BLANCO = 26
LED_ROJO = 27
LED_GIRO_IZQ = 22
LED_GIRO_DER = 23

NEUTRO = 1500
IZQ = 1200
DER = 1800

# Velocidad (ajusta a tu gusto)
VEL_CRUCERO = 1650   # avanza suave
VEL_FRENO = NEUTRO

# ==========================================
# STOP (igual que tu lógica)
# ==========================================
PERSISTENCIA_LIMITE = 15
DISTANCIA_SEGURA = 1.8
DISTANCIA_PARADA = 0.30
DISTANCIA_INTERMITENTES = 0.60

memoria_stop = {
    "contador": PERSISTENCIA_LIMITE,
    "distancia": 0.0,
    "box": None,
    "puntos": None
}

def obtener_distancia_robusta(depth_frame, cx, cy):
    dist = depth_frame.get_distance(cx, cy)
    if dist > 0:
        return dist
    for dx in range(-3, 4, 3):
        for dy in range(-3, 4, 3):
            val = depth_frame.get_distance(cx + dx, cy + dy)
            if val > 0:
                return val
    return 0

# ==========================================
# CARRIL AZUL (HSV)
# (tuneable para tu iluminación)
# ==========================================
AZUL_LO = np.array([90, 70, 40])     # H,S,V
AZUL_HI = np.array([140, 255, 255])

KERNEL = np.ones((5, 5), np.uint8)

# Para decidir si “sí hay carril”
AREA_MIN_AZUL = 2500   # sube/baja según qué tanto azul veas

# Control proporcional dirección: microsegundos por pixel
# (si gira muy agresivo, baja este valor; si gira poco, súbelo)
KP_STEER = 1.2

def clamp(x, lo, hi):
    return max(lo, min(hi, x))

# ==========================================
# STOP TIMER (2 segundos)
# ==========================================
STOP_HOLD_SECONDS = 2.0
stop_hold_until = 0.0
stop_cooldown_until = 0.0  # para evitar re-trigger continuo

# ==========================================
# INIT pigpio
# ==========================================
pi = pigpio.pi()

for p in [LED_BLANCO, LED_ROJO, LED_GIRO_IZQ, LED_GIRO_DER]:
    pi.set_mode(p, pigpio.OUTPUT)
    pi.write(p, 0)

def set_intermitentes_emergencia(on: bool):
    # sencillo: ambos prendidos fijos si on (puedes cambiar a parpadeo si quieres)
    pi.write(LED_GIRO_IZQ, 1 if on else 0)
    pi.write(LED_GIRO_DER, 1 if on else 0)

def esc_arm_sequence():
    # armar ESC: neutro un rato
    pi.set_servo_pulsewidth(ESC_PIN, NEUTRO)
    pi.set_servo_pulsewidth(SERVO_PIN, NEUTRO)
    time.sleep(2.0)

def main():
    global stop_hold_until, stop_cooldown_until

    esc_arm_sequence()

    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

    profile = pipeline.start(config)

    # Ajustes manuales (los tuyos)
    color_sensor = profile.get_device().query_sensors()[1]
    color_sensor.set_option(rs.option.enable_auto_exposure, 0)
    color_sensor.set_option(rs.option.exposure, 150)
    color_sensor.set_option(rs.option.gain, 64)

    align = rs.align(rs.stream.color)

    try:
        while True:
            frames = pipeline.wait_for_frames()
            aligned = align.process(frames)
            color_f = aligned.get_color_frame()
            depth_f = aligned.get_depth_frame()
            if not color_f or not depth_f:
                continue

            img = np.asanyarray(color_f.get_data())
            hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

            H, W = img.shape[:2]
            mitad = H // 2

            # ==========================
            # 1) STOP detect (ROJO) igual que tu código
            # ==========================
            m1 = cv2.inRange(hsv, np.array([0, 130, 50]), np.array([10, 255, 255]))
            m2 = cv2.inRange(hsv, np.array([160, 130, 50]), np.array([180, 255, 255]))
            mask_stop = cv2.dilate(cv2.addWeighted(m1, 1.0, m2, 1.0, 0), KERNEL)

            contours, _ = cv2.findContours(mask_stop, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            hallado_stop = False

            for cnt in contours:
                if cv2.contourArea(cnt) < 600:
                    continue
                approx = cv2.approxPolyDP(cnt, 0.02 * cv2.arcLength(cnt, True), True)
                if len(approx) == 8 and cv2.isContourConvex(approx):
                    x, y, w, h = cv2.boundingRect(approx)
                    if 0.7 <= float(w) / h <= 1.3:
                        dist = obtener_distancia_robusta(depth_f, x + w // 2, y + h // 2)
                        memoria_stop.update({"contador": 0, "distancia": dist, "box": (x, y, w, h), "puntos": approx})
                        hallado_stop = True
                        break

            if not hallado_stop:
                memoria_stop["contador"] += 1

            # ==========================
            # 2) Máscara azul SOLO en ROI (mitad inferior)
            # ==========================
            roi = hsv[mitad:, :]
            mask_azul = cv2.inRange(roi, AZUL_LO, AZUL_HI)

            # limpieza
            mask_azul = cv2.medianBlur(mask_azul, 5)
            mask_azul = cv2.morphologyEx(mask_azul, cv2.MORPH_OPEN, KERNEL, iterations=1)
            mask_azul = cv2.morphologyEx(mask_azul, cv2.MORPH_CLOSE, KERNEL, iterations=2)

            area_azul = int(cv2.countNonZero(mask_azul))
            carril_ok = area_azul > AREA_MIN_AZUL

            # centroide de azul (si hay)
            steer_pwm = NEUTRO
            if carril_ok:
                M = cv2.moments(mask_azul)
                if M["m00"] > 0:
                    cx = int(M["m10"] / M["m00"])  # 0..W-1 dentro del ROI
                    error = cx - (W // 2)
                    steer_pwm = int(NEUTRO + KP_STEER * error)
                    steer_pwm = clamp(steer_pwm, IZQ, DER)
                else:
                    carril_ok = False

            # ==========================
            # 3) Lógica STOP con timer 2s
            # ==========================
            now = time.time()

            stop_visible = (memoria_stop["contador"] < PERSISTENCIA_LIMITE)
            stop_dist = memoria_stop["distancia"] if stop_visible else 0.0

            # Intermitentes emergencia por distancia del STOP
            if stop_visible and (0 < stop_dist < DISTANCIA_INTERMITENTES):
                set_intermitentes_emergencia(True)
            else:
                set_intermitentes_emergencia(False)

            # Trigger de alto de 2 segundos si está MUY cerca
            can_trigger = (now > stop_cooldown_until) and stop_visible and (0 < stop_dist <= DISTANCIA_PARADA)
            if can_trigger:
                stop_hold_until = now + STOP_HOLD_SECONDS
                stop_cooldown_until = now + 4.0  # evita re-trigger mientras sigues viendo el stop

            # ==========================
            # 4) Salida de control (motor + servo)
            # ==========================
            if now < stop_hold_until:
                # Alto 2 segundos
                vel_pwm = VEL_FRENO
                # opcional: recto mientras espera
                steer_out = NEUTRO
                pi.write(LED_ROJO, 1)
            else:
                pi.write(LED_ROJO, 0)

                if carril_ok:
                    vel_pwm = VEL_CRUCERO
                    steer_out = steer_pwm
                else:
                    # Si no ve carril: se frena por seguridad
                    vel_pwm = VEL_FRENO
                    steer_out = NEUTRO

            pi.set_servo_pulsewidth(ESC_PIN, int(vel_pwm))
            pi.set_servo_pulsewidth(SERVO_PIN, int(steer_out))

            # ==========================
            # 5) Visual Debug: arriba imagen, abajo máscara azul (B/N)
            # ==========================
            img_draw = img.copy()

            # Dibujo STOP
            if stop_visible and memoria_stop["box"] is not None:
                x, y, w, h = memoria_stop["box"]
                c = (0, 255, 0) if hallado_stop else (0, 255, 255)
                cv2.drawContours(img_draw, [memoria_stop["puntos"]], 0, c, 3)
                cv2.putText(img_draw, f"STOP: {stop_dist:.2f}m", (x, y - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, c, 2)

            # Dibujo carril (línea centro)
            cv2.line(img_draw, (W // 2, mitad), (W // 2, H), (255, 255, 0), 2)
            cv2.putText(img_draw, f"carril_ok={carril_ok} area={area_azul} steer={steer_out}",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

            top = img_draw[:mitad, :, :]
            bottom_mask_bgr = cv2.cvtColor(mask_azul, cv2.COLOR_GRAY2BGR)
            vista = np.vstack([top, bottom_mask_bgr])

            cv2.imshow("Autonomo: Carril Azul + STOP", vista)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    finally:
        pipeline.stop()
        for p in [LED_BLANCO, LED_ROJO, LED_GIRO_IZQ, LED_GIRO_DER]:
            pi.write(p, 0)
        pi.set_servo_pulsewidth(ESC_PIN, NEUTRO)
        pi.set_servo_pulsewidth(SERVO_PIN, NEUTRO)
        pi.stop()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()

import pyrealsense2 as rs
import numpy as np
import cv2
import pigpio
import time

# ==========================================
# GPIO / PWM
# ==========================================
ESC_PIN = 18
SERVO_PIN = 13

LED_BLANCO = 26
LED_ROJO = 27
LED_GIRO_IZQ = 22
LED_GIRO_DER = 23

NEUTRO = 1500
IZQ = 1200
DER = 1800

# Velocidad (ajusta a tu gusto)
VEL_CRUCERO = 1650   # avanza suave
VEL_FRENO = NEUTRO

# ==========================================
# STOP (igual que tu lógica)
# ==========================================
PERSISTENCIA_LIMITE = 15
DISTANCIA_SEGURA = 1.8
DISTANCIA_PARADA = 0.30
DISTANCIA_INTERMITENTES = 0.60

memoria_stop = {
    "contador": PERSISTENCIA_LIMITE,
    "distancia": 0.0,
    "box": None,
    "puntos": None
}

def obtener_distancia_robusta(depth_frame, cx, cy):
    dist = depth_frame.get_distance(cx, cy)
    if dist > 0:
        return dist
    for dx in range(-3, 4, 3):
        for dy in range(-3, 4, 3):
            val = depth_frame.get_distance(cx + dx, cy + dy)
            if val > 0:
                return val
    return 0

# ==========================================
# CARRIL AZUL (HSV)
# (tuneable para tu iluminación)
# ==========================================
AZUL_LO = np.array([90, 70, 40])     # H,S,V
AZUL_HI = np.array([140, 255, 255])

KERNEL = np.ones((5, 5), np.uint8)

# Para decidir si “sí hay carril”
AREA_MIN_AZUL = 2500   # sube/baja según qué tanto azul veas

# Control proporcional dirección: microsegundos por pixel
# (si gira muy agresivo, baja este valor; si gira poco, súbelo)
KP_STEER = 1.2

def clamp(x, lo, hi):
    return max(lo, min(hi, x))

# ==========================================
# STOP TIMER (2 segundos)
# ==========================================
STOP_HOLD_SECONDS = 2.0
stop_hold_until = 0.0
stop_cooldown_until = 0.0  # para evitar re-trigger continuo

# ==========================================
# INIT pigpio
# ==========================================
pi = pigpio.pi()

for p in [LED_BLANCO, LED_ROJO, LED_GIRO_IZQ, LED_GIRO_DER]:
    pi.set_mode(p, pigpio.OUTPUT)
    pi.write(p, 0)

def set_intermitentes_emergencia(on: bool):
    # sencillo: ambos prendidos fijos si on (puedes cambiar a parpadeo si quieres)
    pi.write(LED_GIRO_IZQ, 1 if on else 0)
    pi.write(LED_GIRO_DER, 1 if on else 0)

def esc_arm_sequence():
    # armar ESC: neutro un rato
    pi.set_servo_pulsewidth(ESC_PIN, NEUTRO)
    pi.set_servo_pulsewidth(SERVO_PIN, NEUTRO)
    time.sleep(2.0)

def main():
    global stop_hold_until, stop_cooldown_until

    esc_arm_sequence()

    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

    profile = pipeline.start(config)

    # Ajustes manuales (los tuyos)
    color_sensor = profile.get_device().query_sensors()[1]
    color_sensor.set_option(rs.option.enable_auto_exposure, 0)
    color_sensor.set_option(rs.option.exposure, 150)
    color_sensor.set_option(rs.option.gain, 64)

    align = rs.align(rs.stream.color)

    try:
        while True:
            frames = pipeline.wait_for_frames()
            aligned = align.process(frames)
            color_f = aligned.get_color_frame()
            depth_f = aligned.get_depth_frame()
            if not color_f or not depth_f:
                continue

            img = np.asanyarray(color_f.get_data())
            hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

            H, W = img.shape[:2]
            mitad = H // 2

            # ==========================
            # 1) STOP detect (ROJO) igual que tu código
            # ==========================
            m1 = cv2.inRange(hsv, np.array([0, 130, 50]), np.array([10, 255, 255]))
            m2 = cv2.inRange(hsv, np.array([160, 130, 50]), np.array([180, 255, 255]))
            mask_stop = cv2.dilate(cv2.addWeighted(m1, 1.0, m2, 1.0, 0), KERNEL)

            contours, _ = cv2.findContours(mask_stop, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            hallado_stop = False

            for cnt in contours:
                if cv2.contourArea(cnt) < 600:
                    continue
                approx = cv2.approxPolyDP(cnt, 0.02 * cv2.arcLength(cnt, True), True)
                if len(approx) == 8 and cv2.isContourConvex(approx):
                    x, y, w, h = cv2.boundingRect(approx)
                    if 0.7 <= float(w) / h <= 1.3:
                        dist = obtener_distancia_robusta(depth_f, x + w // 2, y + h // 2)
                        memoria_stop.update({"contador": 0, "distancia": dist, "box": (x, y, w, h), "puntos": approx})
                        hallado_stop = True
                        break

            if not hallado_stop:
                memoria_stop["contador"] += 1

            # ==========================
            # 2) Máscara azul SOLO en ROI (mitad inferior)
            # ==========================
            roi = hsv[mitad:, :]
            mask_azul = cv2.inRange(roi, AZUL_LO, AZUL_HI)

            # limpieza
            mask_azul = cv2.medianBlur(mask_azul, 5)
            mask_azul = cv2.morphologyEx(mask_azul, cv2.MORPH_OPEN, KERNEL, iterations=1)
            mask_azul = cv2.morphologyEx(mask_azul, cv2.MORPH_CLOSE, KERNEL, iterations=2)

            area_azul = int(cv2.countNonZero(mask_azul))
            carril_ok = area_azul > AREA_MIN_AZUL

            # centroide de azul (si hay)
            steer_pwm = NEUTRO
            if carril_ok:
                M = cv2.moments(mask_azul)
                if M["m00"] > 0:
                    cx = int(M["m10"] / M["m00"])  # 0..W-1 dentro del ROI
                    error = cx - (W // 2)
                    steer_pwm = int(NEUTRO + KP_STEER * error)
                    steer_pwm = clamp(steer_pwm, IZQ, DER)
                else:
                    carril_ok = False

            # ==========================
            # 3) Lógica STOP con timer 2s
            # ==========================
            now = time.time()

            stop_visible = (memoria_stop["contador"] < PERSISTENCIA_LIMITE)
            stop_dist = memoria_stop["distancia"] if stop_visible else 0.0

            # Intermitentes emergencia por distancia del STOP
            if stop_visible and (0 < stop_dist < DISTANCIA_INTERMITENTES):
                set_intermitentes_emergencia(True)
            else:
                set_intermitentes_emergencia(False)

            # Trigger de alto de 2 segundos si está MUY cerca
            can_trigger = (now > stop_cooldown_until) and stop_visible and (0 < stop_dist <= DISTANCIA_PARADA)
            if can_trigger:
                stop_hold_until = now + STOP_HOLD_SECONDS
                stop_cooldown_until = now + 4.0  # evita re-trigger mientras sigues viendo el stop

            # ==========================
            # 4) Salida de control (motor + servo)
            # ==========================
            if now < stop_hold_until:
                # Alto 2 segundos
                vel_pwm = VEL_FRENO
                # opcional: recto mientras espera
                steer_out = NEUTRO
                pi.write(LED_ROJO, 1)
            else:
                pi.write(LED_ROJO, 0)

                if carril_ok:
                    vel_pwm = VEL_CRUCERO
                    steer_out = steer_pwm
                else:
                    # Si no ve carril: se frena por seguridad
                    vel_pwm = VEL_FRENO
                    steer_out = NEUTRO

            pi.set_servo_pulsewidth(ESC_PIN, int(vel_pwm))
            pi.set_servo_pulsewidth(SERVO_PIN, int(steer_out))

            # ==========================
            # 5) Visual Debug: arriba imagen, abajo máscara azul (B/N)
            # ==========================
            img_draw = img.copy()

            # Dibujo STOP
            if stop_visible and memoria_stop["box"] is not None:
                x, y, w, h = memoria_stop["box"]
                c = (0, 255, 0) if hallado_stop else (0, 255, 255)
                cv2.drawContours(img_draw, [memoria_stop["puntos"]], 0, c, 3)
                cv2.putText(img_draw, f"STOP: {stop_dist:.2f}m", (x, y - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, c, 2)

            # Dibujo carril (línea centro)
            cv2.line(img_draw, (W // 2, mitad), (W // 2, H), (255, 255, 0), 2)
            cv2.putText(img_draw, f"carril_ok={carril_ok} area={area_azul} steer={steer_out}",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

            top = img_draw[:mitad, :, :]
            bottom_mask_bgr = cv2.cvtColor(mask_azul, cv2.COLOR_GRAY2BGR)
            vista = np.vstack([top, bottom_mask_bgr])

            cv2.imshow("Autonomo: Carril Azul + STOP", vista)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    finally:
        pipeline.stop()
        for p in [LED_BLANCO, LED_ROJO, LED_GIRO_IZQ, LED_GIRO_DER]:
            pi.write(p, 0)
        pi.set_servo_pulsewidth(ESC_PIN, NEUTRO)
        pi.set_servo_pulsewidth(SERVO_PIN, NEUTRO)
        pi.stop()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()

import pyrealsense2 as rs
import numpy as np
import cv2
import pigpio
import sys
import tty
import termios
import time
import threading
import select

# ==========================================
# CONFIGURACIÓN DE HARDWARE (GPIO)
# ==========================================
ESC_PIN = 18
SERVO_PIN = 13
LED_BLANCO = 26
LED_ROJO = 27
LED_GIRO_IZQ = 22
LED_GIRO_DER = 23

# Valores PWM
NEUTRO = 1500
MAX_AVANCE = 2000
MAX_REVERSA = 1300
PASO_VELOCIDAD = 20
IZQ = 1200
DER = 1800
PASO_DIRECCION = 80

# ==========================================
# CONFIGURACIÓN VISIÓN Y PERSISTENCIA (STOP)
# ==========================================
PERSISTENCIA_LIMITE = 15
DISTANCIA_SEGURA = 1.8
DISTANCIA_PARADA = 0.3
DISTANCIA_INTERMITENTES = 0.6  # 60 cm

memoria_stop = {
    "contador": PERSISTENCIA_LIMITE,
    "distancia": 0.0,
    "box": None,
    "puntos": None
}

# ==========================================
# CARRIL AZUL (HSV) - AJUSTABLE
# ==========================================
# Para tu cinta azul en piso gris (puede requerir ajuste con la luz)
AZUL_LO = np.array([90, 70, 40])     # H,S,V
AZUL_HI = np.array([140, 255, 255])

KERNEL = np.ones((5, 5), np.uint8)

AREA_MIN_AZUL = 2500     # si se frena aunque ve azul, baja este valor
KP_STEER = 1.2           # fuerza de giro (si zigzaguea, baja a 0.7-1.0)

# Velocidad autónoma base (ajusta)
VEL_CRUCERO = 1650       # avanza suave
VEL_SIN_CARRIL = NEUTRO  # seguridad

# ==========================================
# TEMPORIZADOR STOP (2s) + COOLDOWN
# ==========================================
STOP_HOLD_SECONDS = 2.0
STOP_COOLDOWN_SECONDS = 4.0
stop_hold_until = 0.0
stop_cooldown_until = 0.0

# ==========================================
# VARIABLES DE ESTADO GLOBAL
# ==========================================
pi = pigpio.pi()
velocidad_actual = NEUTRO
direccion_actual = NEUTRO
luces_blancas_on = False
intermitentes_emergencia = False
ejecutando = True
emergency_brake = False  # SPACE

# Inicializar Pines
for p in [LED_BLANCO, LED_ROJO, LED_GIRO_IZQ, LED_GIRO_DER]:
    pi.set_mode(p, pigpio.OUTPUT)
    pi.write(p, 0)

# ==========================================
# FUNCIONES DE APOYO
# ==========================================
def clamp(x, lo, hi):
    return max(lo, min(hi, x))

def obtener_distancia_robusta(depth_frame, cx, cy):
    dist = depth_frame.get_distance(cx, cy)
    if dist > 0:
        return dist
    for dx in range(-3, 4, 3):
        for dy in range(-3, 4, 3):
            val = depth_frame.get_distance(cx + dx, cy + dy)
            if val > 0:
                return val
    return 0

def hilo_intermitentes():
    global ejecutando, direccion_actual, intermitentes_emergencia
    while ejecutando:
        if intermitentes_emergencia:
            estado = not pi.read(LED_GIRO_IZQ)
            pi.write(LED_GIRO_IZQ, estado)
            pi.write(LED_GIRO_DER, estado)
        elif direccion_actual < NEUTRO:
            pi.write(LED_GIRO_IZQ, not pi.read(LED_GIRO_IZQ))
            pi.write(LED_GIRO_DER, 0)
        elif direccion_actual > NEUTRO:
            pi.write(LED_GIRO_DER, not pi.read(LED_GIRO_DER))
            pi.write(LED_GIRO_IZQ, 0)
        else:
            pi.write(LED_GIRO_IZQ, 0)
            pi.write(LED_GIRO_DER, 0)
        time.sleep(0.4)

def is_data():
    return select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], [])

# ==========================================
# HILO TECLADO (solo seguridad / luces)
# ==========================================
def hilo_control_teclado():
    global ejecutando, luces_blancas_on, intermitentes_emergencia, emergency_brake

    old_settings = termios.tcgetattr(sys.stdin)
    last_space = 0

    try:
        tty.setcbreak(sys.stdin.fileno())
        while ejecutando:
            if is_data():
                char = sys.stdin.read(1).lower()

                if char == 'l':
                    luces_blancas_on = not luces_blancas_on
                    pi.write(LED_BLANCO, 1 if luces_blancas_on else 0)

                elif char == 'i':
                    intermitentes_emergencia = not intermitentes_emergencia

                elif char == ' ':
                    # freno de emergencia (toggle breve)
                    emergency_brake = True
                    last_space = time.time()

                elif char == 'q':
                    ejecutando = False

            # desactivar emergency brake después de 0.5s
            if emergency_brake and (time.time() - last_space > 0.5):
                emergency_brake = False

            time.sleep(0.01)

    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

# ==========================================
# BUCLE PRINCIPAL (VISIÓN + CONTROL AUTÓNOMO)
# ==========================================
def main():
    global ejecutando, intermitentes_emergencia
    global velocidad_actual, direccion_actual
    global stop_hold_until, stop_cooldown_until

    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

    profile = pipeline.start(config)
    color_sensor = profile.get_device().query_sensors()[1]
    color_sensor.set_option(rs.option.enable_auto_exposure, 0)
    color_sensor.set_option(rs.option.exposure, 150)
    color_sensor.set_option(rs.option.gain, 64)

    align = rs.align(rs.stream.color)

    threading.Thread(target=hilo_intermitentes, daemon=True).start()
    threading.Thread(target=hilo_control_teclado, daemon=True).start()

    try:
        while ejecutando:
            frames = pipeline.wait_for_frames()
            aligned = align.process(frames)
            color_f = aligned.get_color_frame()
            depth_f = aligned.get_depth_frame()
            if not color_f or not depth_f:
                continue

            img = np.asanyarray(color_f.get_data())
            hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

            H, W = img.shape[:2]
            mitad = H // 2

            # =====================================================
            # 1) DETECCIÓN STOP (ROJO) - MISMA LÓGICA QUE TU CÓDIGO
            # =====================================================
            m1 = cv2.inRange(hsv, np.array([0, 130, 50]), np.array([10, 255, 255]))
            m2 = cv2.inRange(hsv, np.array([160, 130, 50]), np.array([180, 255, 255]))
            mask_stop = cv2.dilate(cv2.addWeighted(m1, 1.0, m2, 1.0, 0), KERNEL)

            contours, _ = cv2.findContours(mask_stop, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            hallado_stop = False

            for cnt in contours:
                if cv2.contourArea(cnt) < 600:
                    continue
                approx = cv2.approxPolyDP(cnt, 0.02 * cv2.arcLength(cnt, True), True)

                if len(approx) == 8 and cv2.isContourConvex(approx):
                    x, y, w, h = cv2.boundingRect(approx)
                    if 0.7 <= float(w) / h <= 1.3:
                        dist = obtener_distancia_robusta(depth_f, x + w // 2, y + h // 2)
                        memoria_stop.update({"contador": 0, "distancia": dist, "box": (x, y, w, h), "puntos": approx})
                        hallado_stop = True
                        break

            if not hallado_stop:
                memoria_stop["contador"] += 1

            # --- LÓGICA DE INTERMITENTES AUTOMÁTICAS (igual idea) ---
            if memoria_stop["contador"] < PERSISTENCIA_LIMITE:
                if 0 < memoria_stop["distancia"] < DISTANCIA_INTERMITENTES:
                    intermitentes_emergencia = True
                else:
                    intermitentes_emergencia = False
            else:
                intermitentes_emergencia = False

            # =====================================================
            # 2) CARRIL AZUL EN ROI (mitad inferior)
            # =====================================================
            roi_hsv = hsv[mitad:, :]
            mask_azul = cv2.inRange(roi_hsv, AZUL_LO, AZUL_HI)

            mask_azul = cv2.medianBlur(mask_azul, 5)
            mask_azul = cv2.morphologyEx(mask_azul, cv2.MORPH_OPEN, KERNEL, iterations=1)
            mask_azul = cv2.morphologyEx(mask_azul, cv2.MORPH_CLOSE, KERNEL, iterations=2)

            area_azul = int(cv2.countNonZero(mask_azul))
            carril_ok = area_azul > AREA_MIN_AZUL

            # Centroide para mantener centrado
            direccion_obj = NEUTRO
            cx = None
            if carril_ok:
                M = cv2.moments(mask_azul)
                if M["m00"] > 0:
                    cx = int(M["m10"] / M["m00"])  # 0..W-1 dentro del ROI
                    error = cx - (W // 2)

                    # >>> CORRECCIÓN: giro estaba al revés, invertimos el signo <<<
                    direccion_obj = int(NEUTRO - KP_STEER * error)

                    direccion_obj = clamp(direccion_obj, IZQ, DER)
                else:
                    carril_ok = False

            # =====================================================
            # 3) CONTROL VELOCIDAD:
            #    - Respeta tu bajada progresiva por STOP
            #    - Añade timer 2s cuando ya está parado por completo
            #    - Solo avanza si ve carril
            # =====================================================
            now = time.time()

            # Base: si no ve carril, se queda quieto
            velocidad_obj = VEL_CRUCERO if carril_ok else VEL_SIN_CARRIL

            # --- TU LÓGICA DE SEGURIDAD (sin modificar) ---
            if memoria_stop["contador"] < PERSISTENCIA_LIMITE:
                d = memoria_stop["distancia"]
                if 0 < d <= DISTANCIA_PARADA:
                    velocidad_obj = NEUTRO  # paro total (igual que tú)
                elif DISTANCIA_PARADA < d < DISTANCIA_SEGURA:
                    max_permitida = 1500 + (d - DISTANCIA_PARADA) * ((2000 - 1500) / (DISTANCIA_SEGURA - DISTANCIA_PARADA))
                    if velocidad_obj > max_permitida:
                        velocidad_obj = int(max_permitida)

            # --- TIMER: cuando ya está parado (por d <= parada), sostener 2s ---
            stop_visible = memoria_stop["contador"] < PERSISTENCIA_LIMITE
            d = memoria_stop["distancia"] if stop_visible else 0.0

            # Si cumple condición de paro total y no está en cooldown, disparar timer
            if (now > stop_cooldown_until) and stop_visible and (0 < d <= DISTANCIA_PARADA):
                stop_hold_until = now + STOP_HOLD_SECONDS
                stop_cooldown_until = now + STOP_COOLDOWN_SECONDS

            # Si está en hold, forzar NEUTRO
            if now < stop_hold_until:
                velocidad_obj = NEUTRO
                pi.write(LED_ROJO, 1)
            else:
                pi.write(LED_ROJO, 0)

            # Emergency brake por teclado
            if emergency_brake:
                velocidad_obj = NEUTRO
                direccion_obj = NEUTRO
                pi.write(LED_ROJO, 1)

            # Aplicar salidas
            velocidad_actual = int(velocidad_obj)
            direccion_actual = int(direccion_obj)

            pi.set_servo_pulsewidth(ESC_PIN, velocidad_actual)
            pi.set_servo_pulsewidth(SERVO_PIN, direccion_actual)

            # =====================================================
            # 4) VISUAL: arriba imagen, abajo máscara azul (BN)
            # =====================================================
            img_draw = img.copy()

            # Dibujo STOP
            if memoria_stop["contador"] < PERSISTENCIA_LIMITE and memoria_stop["box"] is not None:
                x, y, w, h = memoria_stop["box"]
                c = (0, 255, 0) if hallado_stop else (0, 255, 255)
                cv2.drawContours(img_draw, [memoria_stop["puntos"]], 0, c, 3)
                cv2.putText(img_draw, f"STOP: {memoria_stop['distancia']:.2f}m", (x, y - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, c, 2)

            # Referencia de centro
            cv2.line(img_draw, (W // 2, mitad), (W // 2, H), (255, 255, 0), 2)
            if cx is not None:
                cv2.circle(img_draw, (cx, mitad + 40), 8, (0, 255, 255), -1)

            cv2.putText(
                img_draw,
                f"carril_ok={carril_ok} area={area_azul} vel={velocidad_actual} dir={direccion_actual}",
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (255, 255, 255),
                2
            )

            top = img_draw[:mitad, :, :]
            bottom_mask_bgr = cv2.cvtColor(mask_azul, cv2.COLOR_GRAY2BGR)
            vista = np.vstack([top, bottom_mask_bgr])

            cv2.imshow("Control Inteligente Pi4B (STOP + Carril Azul)", vista)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                ejecutando = False
                break

    finally:
        ejecutando = False
        pipeline.stop()
        for p in [LED_BLANCO, LED_ROJO, LED_GIRO_IZQ, LED_GIRO_DER]:
            pi.write(p, 0)
        pi.set_servo_pulsewidth(ESC_PIN, NEUTRO)
        pi.set_servo_pulsewidth(SERVO_PIN, NEUTRO)
        pi.stop()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
