import pigpio
import sys
import tty
import termios
import time
import cv2
import threading
import numpy as np
import math
import select  # Necesario para lectura no bloqueante

# ==========================================
# CONFIGURACIÓN DE HARDWARE (GPIO)
# ==========================================
ESC_PIN = 18
SERVO_PIN = 13
LED_BLANCO = 17   # Faros (L)
LED_ROJO = 27     # Freno (S)
LED_GIRO_IZQ = 22 # Intermitente Izquierdo (A)
LED_GIRO_DER = 23 # Intermitente Derecho (D)

# PWM Valores
NEUTRO = 1500
MAX_AVANCE_LIBRE = 1750
MAX_REVERSA = 1300
PASO_VELOCIDAD = 15
IZQ = 1250
DER = 1750

# ==========================================
# VARIABLES DE ESTADO GLOBALES
# ==========================================
pi = pigpio.pi()
velocidad_actual = NEUTRO
direccion_actual = NEUTRO
velocidad_max_permitida = MAX_AVANCE_LIBRE
luces_blancas_on = False
ejecutando = True

# Inicializar Pines LED
for p in [LED_BLANCO, LED_ROJO, LED_GIRO_IZQ, LED_GIRO_DER]:
    pi.set_mode(p, pigpio.OUTPUT)
    pi.write(p, 0)

# ==========================================
# HILO DE INTERMITENTES (AMARILLOS)
# ==========================================
def hilo_intermitentes():
    global ejecutando, direccion_actual
    while ejecutando:
        if direccion_actual == IZQ:
            pi.write(LED_GIRO_IZQ, not pi.read(LED_GIRO_IZQ))
            pi.write(LED_GIRO_DER, 0)
        elif direccion_actual == DER:
            pi.write(LED_GIRO_DER, not pi.read(LED_GIRO_DER))
            pi.write(LED_GIRO_IZQ, 0)
        else:
            pi.write(LED_GIRO_IZQ, 0)
            pi.write(LED_GIRO_DER, 0)
        time.sleep(0.4)

# ==========================================
# LÓGICA DE VISIÓN (Filtros Octágono)
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
MAX_SHAPE_DIST = 0.25

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

def hilo_vision_autonoma():
    global ejecutando, velocidad_max_permitida, velocidad_actual
    cap = cv2.VideoCapture(CAM_INDEX, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

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
            
            if len(approx) in ALLOW_SIDES and cv2.isContourConvex(approx):
                x, y, bw, bh = cv2.boundingRect(approx)
                aspect = bw / float(bh)
                if ASPECT_MIN <= aspect <= ASPECT_MAX:
                    solidity = area / (cv2.contourArea(cv2.convexHull(approx)) + 1e-6)
                    if solidity >= MIN_SOLIDITY and circularity(approx) >= MIN_CIRCULARITY:
                        if cv2.matchShapes(approx, IDEAL_OCT, 1, 0.0) < MAX_SHAPE_DIST:
                            stop_detectado = True
                            cv2.rectangle(vis, (x, y), (x + bw, y + bh), (0, 255, 0), 2)
                            
                            if area < 5000: velocidad_max_permitida = MAX_AVANCE_LIBRE
                            elif 5000 <= area < 30000:
                                factor = (area - 5000) / 25000
                                velocidad_max_permitida = int(MAX_AVANCE_LIBRE - (factor * (MAX_AVANCE_LIBRE - 1580)))
                            else: velocidad_max_permitida = NEUTRO
                            
                            if velocidad_actual > velocidad_max_permitida:
                                velocidad_actual = velocidad_max_permitida
                                pi.set_servo_pulsewidth(ESC_PIN, velocidad_actual)
                            break 

        if not stop_detectado: velocidad_max_permitida = MAX_AVANCE_LIBRE

        color_txt = (0, 255, 0) if stop_detectado else (255, 255, 255)
        cv2.putText(vis, f"V: {velocidad_actual} | Max: {velocidad_max_permitida}", (20, 440),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, color_txt, 2)
        cv2.imshow("Traxxas AI", vis)
        if cv2.waitKey(1) & 0xFF == ord('q'): ejecutando = False

    cap.release()
    cv2.destroyAllWindows()

# ==========================================
# CONTROL POR TECLADO (NUEVA LÓGICA NO BLOQUEANTE)
# ==========================================
def is_data():
    """Revisa si hay una tecla esperando en el buffer"""
    return select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], [])

def controlar_coche():
    global velocidad_actual, direccion_actual, ejecutando, velocidad_max_permitida, luces_blancas_on
    
    threading.Thread(target=hilo_vision_autonoma, daemon=True).start()
    threading.Thread(target=hilo_intermitentes, daemon=True).start()

    # Guardar configuración original de la terminal
    old_settings = termios.tcgetattr(sys.stdin)
    
    print("--- SISTEMA UNIFICADO CON LEDS REACTIVOS ---")
    pi.set_servo_pulsewidth(ESC_PIN, NEUTRO)
    pi.set_servo_pulsewidth(SERVO_PIN, NEUTRO)
    time.sleep(2)

    last_s_press = 0 # Marca de tiempo para la tecla S

    try:
        tty.setcbreak(sys.stdin.fileno())
        while ejecutando:
            # 1. LÓGICA DE TIEMPO PARA LUZ DE FRENO
            # Si pasaron más de 0.12s desde la última 's', apagar luz
            if time.time() - last_s_press > 0.12:
                pi.write(LED_ROJO, 0)

            # 2. LEER TECLADO (SI HAY DATOS)
            if is_data():
                char = sys.stdin.read(1).lower()
                
                if char == 's':
                    pi.write(LED_ROJO, 1) # Encender luz inmediatamente
                    last_s_press = time.time() # Actualizar cronómetro
                    if velocidad_actual > MAX_REVERSA:
                        velocidad_actual -= PASO_VELOCIDAD
                
                elif char == 'w':
                    # Acelera como un coche (sin luces)
                    if velocidad_actual < velocidad_max_permitida:
                        velocidad_actual += PASO_VELOCIDAD
                
                elif char == 'a':
                    direccion_actual = IZQ
                elif char == 'd':
                    direccion_actual = DER
                elif char == 'c':
                    direccion_actual = NEUTRO
                elif char == 'l':
                    luces_blancas_on = not luces_blancas_on
                    pi.write(LED_BLANCO, 1 if luces_blancas_on else 0)
                elif char == ' ':
                    velocidad_actual = NEUTRO
                elif char == 'q':
                    ejecutando = False
            
            # 3. ACTUALIZAR HARDWARE CONSTANTEMENTE
            pi.set_servo_pulsewidth(ESC_PIN, velocidad_actual)
            pi.set_servo_pulsewidth(SERVO_PIN, direccion_actual)
            
            # Pequeña pausa para no saturar el CPU
            time.sleep(0.01)
            
    finally:
        # Restaurar terminal
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        # Limpieza y seguridad
        pi.set_servo_pulsewidth(ESC_PIN, NEUTRO)
        pi.set_servo_pulsewidth(SERVO_PIN, NEUTRO)
        for p in [LED_BLANCO, LED_ROJO, LED_GIRO_IZQ, LED_GIRO_DER]:
            pi.write(p, 0)
        pi.stop()

if __name__ == "__main__":
    controlar_coche()
