import pigpio
import sys
import tty
import termios
import time
import cv2
import threading
import numpy as np
import math

# ==========================================
# CONFIGURACIÓN DE HARDWARE (PWM)
# ==========================================
ESC_PIN = 18
SERVO_PIN = 13
NEUTRO = 1500
MAX_AVANCE_LIBRE = 2000
MAX_REVERSA = 1000
PASO_VELOCIDAD = 1
IZQ = 1250
DER = 1750

# ==========================================
# CONFIGURACIÓN DE VISIÓN (Filtros Octágono)
# ==========================================
CAM_INDEX = 4
S_MIN = 120
V_MIN = 80
MIN_AREA_FRAC = 0.002 
EPS_FRAC = 0.02 
ALLOW_SIDES = {7, 8, 9}
MIN_SOLIDITY = 0.92
MIN_CIRCULARITY = 0.72
ASPECT_MIN = 0.80
ASPECT_MAX = 1.20
MAX_SHAPE_DIST = 0.25 # Menor = más estricto con la forma

# ==========================================
# VARIABLES DE ESTADO GLOBALES
# ==========================================
pi = pigpio.pi()
velocidad_actual = NEUTRO
direccion_actual = NEUTRO
velocidad_max_permitida = MAX_AVANCE_LIBRE
ejecutando = True

# ==========================================
# FUNCIONES AUXILIARES DE VISIÓN
# ==========================================
def make_ideal_octagon(size=200):
    cx, cy = size // 2, size // 2
    r = size * 0.38
    pts = []
    for k in range(8):
        ang = (math.pi / 8.0) + k * (math.pi / 4.0)
        x = cx + r * math.cos(ang)
        y = cy + r * math.sin(ang)
        pts.append([int(x), int(y)])
    return np.array(pts, dtype=np.int32).reshape((-1, 1, 2))

IDEAL_OCT = make_ideal_octagon(240)

def red_mask(frame_bgr):
    hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
    lower_red1 = np.array([0, S_MIN, V_MIN])
    upper_red1 = np.array([10, 255, 255])
    lower_red2 = np.array([170, S_MIN, V_MIN])
    upper_red2 = np.array([180, 255, 255])
    mask = cv2.bitwise_or(cv2.inRange(hsv, lower_red1, upper_red1),
                          cv2.inRange(hsv, lower_red2, upper_red2))
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
    return mask

def circularity(contour):
    a = cv2.contourArea(contour)
    p = cv2.arcLength(contour, True) + 1e-6
    return 4.0 * math.pi * a / (p * p)

# ==========================================
# HILO DE VISIÓN Y VELOCIDAD DINÁMICA
# ==========================================
def hilo_vision_autonoma():
    global ejecutando, velocidad_max_permitida, velocidad_actual
    
    cap = cv2.VideoCapture(CAM_INDEX, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 848)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    if not cap.isOpened():
        print(f"\n[Error] No se pudo abrir la cámara {CAM_INDEX}")
        return

    while ejecutando:
        ret, frame = cap.read()
        if not ret or frame is None: continue

        h, w = frame.shape[:2]
        frame_area = h * w
        mask = red_mask(frame)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        stop_detectado = False
        vis = frame.copy()

        for c in contours:
            area = cv2.contourArea(c)
            if area < MIN_AREA_FRAC * frame_area: continue

            peri = cv2.arcLength(c, True)
            approx = cv2.approxPolyDP(c, EPS_FRAC * peri, True)
            
            # FILTROS GEOMÉTRICOS
            sides = len(approx)
            if sides not in ALLOW_SIDES: continue
            if not cv2.isContourConvex(approx): continue
            
            x, y, bw, bh = cv2.boundingRect(approx)
            aspect = bw / float(bh)
            if not (ASPECT_MIN <= aspect <= ASPECT_MAX): continue

            solidity = area / (cv2.contourArea(cv2.convexHull(approx)) + 1e-6)
            if solidity < MIN_SOLIDITY: continue

            circ = circularity(approx)
            if circ < MIN_CIRCULARITY: continue

            # MATCH CONTRA OCTÁGONO IDEAL
            dist = cv2.matchShapes(approx, IDEAL_OCT, cv2.CONTOURS_MATCH_I1, 0.0)
            if dist > MAX_SHAPE_DIST: continue

            # Si pasa todos los filtros, es un STOP
            stop_detectado = True
            cv2.rectangle(vis, (x, y), (x + bw, y + bh), (0, 255, 0), 2)
            cv2.drawContours(vis, [approx], -1, (0, 255, 0), 2)

            # --- LÓGICA DE VELOCIDAD DINÁMICA BASADA EN ÁREA ---
            if area < 5000:
                velocidad_max_permitida = MAX_AVANCE_LIBRE
            elif 5000 <= area < 30000:
                factor_reduccion = (area - 5000) / 25000
                rango_vel = MAX_AVANCE_LIBRE - 1580
                velocidad_max_permitida = int(MAX_AVANCE_LIBRE - (factor_reduccion * rango_vel))
            else:
                velocidad_max_permitida = NEUTRO # FRENO TOTAL

            # Aplicar freno si la velocidad actual supera el límite
            if velocidad_actual > velocidad_max_permitida:
                velocidad_actual = velocidad_max_permitida
                pi.set_servo_pulsewidth(ESC_PIN, velocidad_actual)
            break 

        if not stop_detectado:
            velocidad_max_permitida = MAX_AVANCE_LIBRE

        # Info en pantalla
        color_txt = (0, 255, 0) if stop_detectado else (255, 255, 255)
        cv2.putText(vis, f"V: {velocidad_actual} | Max: {velocidad_max_permitida}", (10, 450),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, color_txt, 2)
        cv2.imshow("Traxxas AI - Octagon Detection", vis)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            ejecutando = False
            break

    cap.release()
    cv2.destroyAllWindows()

# ==========================================
# CONTROL POR TECLADO
# ==========================================
def getch():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

def controlar_coche():
    global velocidad_actual, direccion_actual, ejecutando, velocidad_max_permitida
    
    t_vision = threading.Thread(target=hilo_vision_autonoma)
    t_vision.start()

    print("--- SISTEMA UNIFICADO ACTIVO ---")
    print("Octagon STOP Filter + Keyboard Control")
    pi.set_servo_pulsewidth(ESC_PIN, NEUTRO)
    pi.set_servo_pulsewidth(SERVO_PIN, NEUTRO)
    time.sleep(3) # Armado del ESC

    try:
        while ejecutando:
            char = getch().lower()
            if char == 'w':
                if velocidad_actual < velocidad_max_permitida:
                    velocidad_actual += PASO_VELOCIDAD
                pi.set_servo_pulsewidth(ESC_PIN, velocidad_actual)
            elif char == 's':
                if velocidad_actual > MAX_REVERSA:
                    velocidad_actual -= PASO_VELOCIDAD
                pi.set_servo_pulsewidth(ESC_PIN, velocidad_actual)
            elif char == 'a':
                direccion_actual = IZQ
                pi.set_servo_pulsewidth(SERVO_PIN, direccion_actual)
            elif char == 'd':
                direccion_actual = DER
                pi.set_servo_pulsewidth(SERVO_PIN, direccion_actual)
            elif char == 'c':
                direccion_actual = NEUTRO
                pi.set_servo_pulsewidth(SERVO_PIN, direccion_actual)
            elif char == ' ':
                velocidad_actual = NEUTRO
                pi.set_servo_pulsewidth(ESC_PIN, NEUTRO)
            elif char == 'q':
                ejecutando = False
                break
    except KeyboardInterrupt:
        ejecutando = False
    finally:
        pi.set_servo_pulsewidth(ESC_PIN, NEUTRO)
        pi.set_servo_pulsewidth(SERVO_PIN, NEUTRO)
        time.sleep(0.5)
        pi.set_servo_pulsewidth(ESC_PIN, 0)
        pi.set_servo_pulsewidth(SERVO_PIN, 0)
        pi.stop()

if __name__ == "__main__":
    controlar_coche()
