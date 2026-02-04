import pigpio
import sys
import tty
import termios
import time
import cv2
import threading
import numpy as np
import math

# --- Configuración de Hardware (PWM) ---
ESC_PIN = 18
SERVO_PIN = 13
# --- Configuración de LEDs ---
PIN_BLANCAS = 17
PIN_ROJAS = 27
PIN_AMARILLO_I = 22
PIN_AMARILLO_D = 23

# --- Parámetros de Control ---
NEUTRO = 1500
MAX_AVANCE_LIBRE = 1750
MAX_REVERSA = 1300
PASO_VELOCIDAD = 15
IZQ = 1250
DER = 1750

# --- Variables de estado globales ---
pi = pigpio.pi()
velocidad_actual = NEUTRO
direccion_actual = NEUTRO
velocidad_max_permitida = MAX_AVANCE_LIBRE
ejecutando = True
luces_blancas_on = False

# --- Configuración de Visión ---
CAM_INDEX = 4
S_MIN, V_MIN = 120, 80
ALLOW_SIDES = {7, 8, 9}
IDEAL_OCT = None # Se inicializa en main

# ==========================================
# GESTIÓN DE LUCES (Hilo independiente)
# ==========================================
def hilo_luces():
    global ejecutando, direccion_actual, velocidad_actual, luces_blancas_on
    
    estado_parpadeo = False
    
    while ejecutando:
        # 1. Control de Luces Blancas (Frontales)
        pi.write(PIN_BLANCAS, 1 if luces_blancas_on else 0)
        
        # 2. Control de Luces Rojas (Freno/Reversa)
        # Se encienden si la velocidad es menor al neutro (marcha atrás)
        pi.write(PIN_ROJAS, 1 if velocidad_actual < NEUTRO else 0)
        
        # 3. Control de Intermitentes (Parpadeo 500ms)
        estado_parpadeo = not estado_parpadeo
        
        # Izquierda
        if direccion_actual == IZQ:
            pi.write(PIN_AMARILLO_I, 1 if estado_parpadeo else 0)
            pi.write(PIN_AMARILLO_D, 0)
        # Derecha
        elif direccion_actual == DER:
            pi.write(PIN_AMARILLO_D, 1 if estado_parpadeo else 0)
            pi.write(PIN_AMARILLO_I, 0)
        # Neutro
        else:
            pi.write(PIN_AMARILLO_I, 0)
            pi.write(PIN_AMARILLO_D, 0)
            
        time.sleep(0.3) # Velocidad del parpadeo

# ==========================================
# FUNCIONES DE VISIÓN (Mismo que el anterior)
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

def red_mask(frame_bgr):
    hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
    lower1, upper1 = np.array([0, S_MIN, V_MIN]), np.array([10, 255, 255])
    lower2, upper2 = np.array([170, S_MIN, V_MIN]), np.array([180, 255, 255])
    mask = cv2.bitwise_or(cv2.inRange(hsv, lower1, upper1), cv2.inRange(hsv, lower2, upper2))
    return cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((5,5), np.uint8))

def hilo_vision_autonoma():
    global ejecutando, velocidad_max_permitida, velocidad_actual
    cap = cv2.VideoCapture(CAM_INDEX)
    ideal_oct = make_ideal_octagon(240)
    
    while ejecutando:
        ret, frame = cap.read()
        if not ret: continue
        
        mask = red_mask(frame)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        stop_detectado = False

        for c in contours:
            area = cv2.contourArea(c)
            if area < (frame.shape[0]*frame.shape[1] * 0.002): continue
            
            peri = cv2.arcLength(c, True)
            approx = cv2.approxPolyDP(c, 0.02 * peri, True)
            
            if len(approx) in ALLOW_SIDES and cv2.isContourConvex(approx):
                dist = cv2.matchShapes(approx, ideal_oct, 1, 0.0)
                if dist < 0.25:
                    stop_detectado = True
                    # Lógica de velocidad dinámica
                    if area < 5000: velocidad_max_permitida = MAX_AVANCE_LIBRE
                    elif area < 30000: velocidad_max_permitida = 1580
                    else: velocidad_max_permitida = NEUTRO
                    
                    if velocidad_actual > velocidad_max_permitida:
                        velocidad_actual = velocidad_max_permitida
                        pi.set_servo_pulsewidth(ESC_PIN, velocidad_actual)
                    break

        if not stop_detectado: velocidad_max_permitida = MAX_AVANCE_LIBRE
        cv2.imshow("RealSense D435", frame)
        cv2.waitKey(1)
    cap.release()

# ==========================================
# CONTROL Y MAIN
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
    global velocidad_actual, direccion_actual, ejecutando, luces_blancas_on
    
    # Iniciar hilos
    threading.Thread(target=hilo_vision_autonoma, daemon=True).start()
    threading.Thread(target=hilo_luces, daemon=True).start()

    print("--- TRAXXAS PI 4: IA + LUCES + CONTROL ---")
    print("W/S: Motor | A/D: Giro | L: Luces | Q: Salir")
    
    try:
        while ejecutando:
            char = getch().lower()
            if char == 'w':
                if velocidad_actual < velocidad_max_permitida: velocidad_actual += PASO_VELOCIDAD
                pi.set_servo_pulsewidth(ESC_PIN, velocidad_actual)
            elif char == 's':
                if velocidad_actual > MAX_REVERSA: velocidad_actual -= PASO_VELOCIDAD
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
            elif char == 'l':
                luces_blancas_on = not luces_blancas_on
            elif char == ' ':
                velocidad_actual = NEUTRO
                pi.set_servo_pulsewidth(ESC_PIN, NEUTRO)
            elif char == 'q':
                ejecutando = False
    finally:
        pi.set_servo_pulsewidth(ESC_PIN, NEUTRO)
        pi.set_servo_pulsewidth(SERVO_PIN, NEUTRO)
        for p in [PIN_BLANCAS, PIN_ROJAS, PIN_AMARILLO_I, PIN_AMARILLO_D]:
            pi.write(p, 0)
        pi.stop()

if __name__ == "__main__":
    controlar_coche()
