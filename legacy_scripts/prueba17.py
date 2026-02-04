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
# CONFIGURACIÓN DE HARDWARE (GPIO)
# ==========================================
ESC_PIN = 18
SERVO_PIN = 13
LED_BLANCO = 17   # Faros frontales
LED_ROJO = 27     # Freno/Reversa
LED_GIRO_IZQ = 22 # Intermitente Izquierdo
LED_GIRO_DER = 23 # Intermitente Derecho

# PWM Valores
NEUTRO = 1500
MAX_AVANCE_LIBRE = 1650
MAX_REVERSA = 1400
PASO_VELOCIDAD = 1
IZQ = 1250
DER = 1750

# ==========================================
# VARIABLES DE ESTADO GLOBALES
# ==========================================
pi = pigpio.pi()
velocidad_actual = NEUTRO
direccion_actual = NEUTRO
velocidad_max_permitida = MAX_AVANCE_LIBRE
luces_headlights = False
ejecutando = True

# Configurar pines de salida
for pin in [LED_BLANCO, LED_ROJO, LED_GIRO_IZQ, LED_GIRO_DER]:
    pi.set_mode(pin, pigpio.OUTPUT)

# ==========================================
# HILO DE INTERMITENTES (DIRECCIONALES)
# ==========================================
def hilo_intermitentes():
    global ejecutando, direccion_actual
    estado_led = 0
    while ejecutando:
        # Parpadeo cada 0.4 segundos
        estado_led = 1 - estado_led
        
        if direccion_actual == IZQ:
            pi.write(LED_GIRO_IZQ, estado_led)
            pi.write(LED_GIRO_DER, 0)
        elif direccion_actual == DER:
            pi.write(LED_GIRO_DER, estado_led)
            pi.write(LED_GIRO_IZQ, 0)
        else:
            # Apagar ambos si está centrado
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
    return mask

def hilo_vision_autonoma():
    global ejecutando, velocidad_max_permitida, velocidad_actual
    cap = cv2.VideoCapture(CAM_INDEX)
    if not cap.isOpened(): return

    while ejecutando:
        ret, frame = cap.read()
        if not ret: continue
        
        h, w = frame.shape[:2]
        mask = red_mask(frame)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        stop_detectado = False
        vis = frame.copy()

        for c in contours:
            area = cv2.contourArea(c)
            if area < (MIN_AREA_FRAC * h * w): continue
            
            peri = cv2.arcLength(c, True)
            approx = cv2.approxPolyDP(c, EPS_FRAC * peri, True)
            
            if len(approx) in ALLOW_SIDES and cv2.isContourConvex(approx):
                x, y, bw, bh = cv2.boundingRect(approx)
                aspect = bw / float(bh)
                if ASPECT_MIN <= aspect <= ASPECT_MAX:
                    solidity = area / (cv2.contourArea(cv2.convexHull(approx)) + 1e-6)
                    if solidity > MIN_SOLIDITY and cv2.matchShapes(approx, IDEAL_OCT, 1, 0.0) < MAX_SHAPE_DIST:
                        stop_detectado = True
                        cv2.rectangle(vis, (x, y), (x + bw, y + bh), (0, 255, 0), 2)
                        
                        # Velocidad dinámica
                        if area < 5000: velocidad_max_permitida = MAX_AVANCE_LIBRE
                        elif 5000 <= area < 30000:
                            factor = (area - 5000) / 25000
                            velocidad_max_permitida = int(MAX_AVANCE_LIBRE - (factor * (MAX_AVANCE_LIBRE - 1580)))
                        else: velocidad_max_permitida = NEUTRO
                        break

        if not stop_detectado: velocidad_max_permitida = MAX_AVANCE_LIBRE

        # Actualizar video
        cv2.putText(vis, f"V: {velocidad_actual} | Headlights: {'ON' if luces_headlights else 'OFF'}", 
                    (10, 450), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.imshow("Traxxas AI", vis)
        cv2.waitKey(1)

    cap.release()
    cv2.destroyAllWindows()

# ==========================================
# CONTROL POR TECLADO Y LUCES
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
    global velocidad_actual, direccion_actual, ejecutando, velocidad_max_permitida, luces_headlights
    
    # Iniciar hilos
    threading.Thread(target=hilo_vision_autonoma, daemon=True).start()
    threading.Thread(target=hilo_intermitentes, daemon=True).start()

    print("--- SISTEMA UNIFICADO + LUCES ACTIVO ---")
    print("Controles: W/S (Vel), A/D (Giro), L (Faros), C (Centrar), Q (Salir)")
    
    # Armado
    pi.set_servo_pulsewidth(ESC_PIN, NEUTRO)
    pi.set_servo_pulsewidth(SERVO_PIN, NEUTRO)
    time.sleep(2)

    try:
        while ejecutando:
            char = getch().lower()
            
            # --- MANEJO DE LUCES DE FRENO/REVERSA ---
            # Se encienden si presiono 'S' o si el coche va en reversa
            if char == 's' or velocidad_actual < NEUTRO:
                pi.write(LED_ROJO, 1)
            else:
                pi.write(LED_ROJO, 0)

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
            elif char == 'l': # TOGGLE HEADLIGHTS
                luces_headlights = not luces_headlights
                pi.write(LED_BLANCO, 1 if luces_headlights else 0)
            elif char == ' ':
                velocidad_actual = NEUTRO
                pi.set_servo_pulsewidth(ESC_PIN, NEUTRO)
            elif char == 'q':
                ejecutando = False
                break
                
    except KeyboardInterrupt:
        ejecutando = False
    finally:
        # Apagar todo al salir
        for p in [ESC_PIN, SERVO_PIN, LED_BLANCO, LED_ROJO, LED_GIRO_IZQ, LED_GIRO_DER]:
            pi.set_servo_pulsewidth(p, 0) if p <= 18 else pi.write(p, 0)
        pi.stop()

if __name__ == "__main__":
    controlar_coche()
