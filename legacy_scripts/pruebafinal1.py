import cv2
import numpy as np
import math
import time
import pigpio
import sys
import tty
import termios
import threading
import select

# ==========================================
# CONFIGURACIÓN DE HARDWARE (GPIO)
# ==========================================
ESC_PIN = 18      
SERVO_PIN = 13    
LED_BLANCO = 17   # Faros (L)
LED_ROJO = 27     # Freno (S o Stop Detectado)
LED_GIRO_IZQ = 22 # Intermitente Izquierdo (A)
LED_GIRO_DER = 23 # Intermitente Derecho (D)

# Valores PWM
NEUTRO = 1500
MAX_AVANCE = 1650
MAX_REVERSA = 1400
PASO_VELOCIDAD = 1
IZQ = 1250
DER = 1750

# ======================
# CONFIGURACIÓN VISIÓN
# ======================
CAM_INDEX = 4 # Index para RealSense D435 como V4L2
S_MIN, V_MIN = 120, 80
MIN_AREA_FRAC = 0.002
EPS_FRAC = 0.02
ALLOW_SIDES = {7, 8, 9}
MIN_SOLIDITY = 0.92
MIN_CIRCULARITY = 0.72
ASPECT_MIN, ASPECT_MAX = 0.80, 1.20
MAX_SHAPE_DIST = 0.25

# ==========================================
# VARIABLES DE ESTADO GLOBALES
# ==========================================
pi = pigpio.pi()
velocidad_actual = NEUTRO
direccion_actual = NEUTRO
luces_blancas_on = False
stop_detectado = False
ejecutando = True

# Inicializar Pines
for p in [LED_BLANCO, LED_ROJO, LED_GIRO_IZQ, LED_GIRO_DER]:
    pi.set_mode(p, pigpio.OUTPUT)
    pi.write(p, 0)

# ======================
# FUNCIONES AUXILIARES CV
# ======================
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
    l1, u1 = np.array([0, S_MIN, V_MIN]), np.array([10, 255, 255])
    l2, u2 = np.array([170, S_MIN, V_MIN]), np.array([180, 255, 255])
    mask = cv2.bitwise_or(cv2.inRange(hsv, l1, u1), cv2.inRange(hsv, l2, u2))
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
    return cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)

def circularity(contour):
    a = cv2.contourArea(contour)
    p = cv2.arcLength(contour, True) + 1e-6
    return 4.0 * math.pi * a / (p * p)

# ==========================================
# HILO 1: VISIÓN ARTIFICIAL (STOP DETECTION)
# ==========================================
def hilo_vision():
    global stop_detectado, ejecutando
    cap = cv2.VideoCapture(CAM_INDEX, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 848)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    while ejecutando:
        ret, frame = cap.read()
        if not ret: continue

        h, w = frame.shape[:2]
        mask = red_mask(frame)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        found_in_frame = False
        for c in contours:
            area = cv2.contourArea(c)
            if area < MIN_AREA_FRAC * (h * w): continue
            
            peri = cv2.arcLength(c, True)
            approx = cv2.approxPolyDP(c, EPS_FRAC * peri, True)
            
            if len(approx) in ALLOW_SIDES and cv2.isContourConvex(approx):
                circ = circularity(approx)
                dist = cv2.matchShapes(approx, IDEAL_OCT, cv2.CONTOURS_MATCH_I1, 0.0)
                
                if circ > MIN_CIRCULARITY and dist < MAX_SHAPE_DIST:
                    found_in_frame = True
                    break
        
        stop_detectado = found_in_frame
        time.sleep(0.03) # Liberar CPU
    cap.release()

# ==========================================
# HILO 2: INTERMITENTES
# ==========================================
def hilo_intermitentes():
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
# HILO PRINCIPAL: CONTROL Y TECLADO
# ==========================================
def is_data():
    return select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], [])

def main():
    global velocidad_actual, direccion_actual, ejecutando, luces_blancas_on
    
    # Iniciar hilos secundarios
    threading.Thread(target=hilo_vision, daemon=True).start()
    threading.Thread(target=hilo_intermitentes, daemon=True).start()

    old_settings = termios.tcgetattr(sys.stdin)
    
    # Armado del ESC
    pi.set_servo_pulsewidth(ESC_PIN, NEUTRO)
    pi.set_servo_pulsewidth(SERVO_PIN, NEUTRO)
    print("Sistema Iniciado. Esperando ESC (2s)...")
    time.sleep(2)

    last_s_press = 0 

    try:
        tty.setcbreak(sys.stdin.fileno())
        while ejecutando:
            # LÓGICA DE PRIORIDAD: STOP SIGN
            if stop_detectado:
                velocidad_actual = NEUTRO
                pi.write(LED_ROJO, 1) # Encender freno por seguridad
            else:
                # Si no hay stop, apagar freno después de un tiempo de soltar 's'
                if time.time() - last_s_press > 0.8:
                    pi.write(LED_ROJO, 0)

            if is_data():
                char = sys.stdin.read(1).lower()
                
                if char == 's':
                    pi.write(LED_ROJO, 1)
                    last_s_press = time.time()
                    if velocidad_actual > MAX_REVERSA:
                        velocidad_actual -= PASO_VELOCIDAD
                
                elif char == 'w' and not stop_detectado: # Solo avanza si no hay Stop
                    if velocidad_actual < MAX_AVANCE:
                        velocidad_actual += PASO_VELOCIDAD
                
                elif char == 'a': direccion_actual = IZQ
                elif char == 'd': direccion_actual = DER
                elif char == 'c': direccion_actual = NEUTRO
                elif char == 'l':
                    luces_blancas_on = not luces_blancas_on
                    pi.write(LED_BLANCO, 1 if luces_blancas_on else 0)
                elif char == ' ': velocidad_actual = NEUTRO
                elif char == 'q': ejecutando = False
            
            # Actualización de hardware
            pi.set_servo_pulsewidth(ESC_PIN, velocidad_actual)
            pi.set_servo_pulsewidth(SERVO_PIN, direccion_actual)
            
            time.sleep(0.01)
            
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        pi.set_servo_pulsewidth(ESC_PIN, NEUTRO)
        pi.set_servo_pulsewidth(SERVO_PIN, NEUTRO)
        for p in [LED_BLANCO, LED_ROJO, LED_GIRO_IZQ, LED_GIRO_DER]:
            pi.write(p, 0)
        pi.stop()
        print("\nSistema apagado.")

if __name__ == "__main__":
    main()
