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
LED_BLANCO = 17   # Faros
LED_ROJO = 27     # Freno
LED_GIRO_IZQ = 22 # Intermitente Izquierdo
LED_GIRO_DER = 23 # Intermitente Derecho

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
CAM_INDEX = 4
S_MIN, V_MIN = 120, 80
MIN_AREA_FRAC = 0.002
EPS_FRAC = 0.02
ALLOW_SIDES = {7, 8, 9}
MIN_SOLIDITY = 0.92
MIN_CIRCULARITY = 0.72
ASPECT_MIN, ASPECT_MAX = 0.80, 1.20
MAX_SHAPE_DIST = 0.25

# ==========================================
# VARIABLES GLOBALES DE ESTADO
# ==========================================
pi = pigpio.pi()
velocidad_actual = NEUTRO
direccion_actual = NEUTRO
luces_blancas_on = False
ejecutando = True
stop_detectado = False

# Inicializar Pines
for p in [LED_BLANCO, LED_ROJO, LED_GIRO_IZQ, LED_GIRO_DER]:
    pi.set_mode(p, pigpio.OUTPUT)
    pi.write(p, 0)

# Preparar Octágono Ideal para comparación
def make_ideal_octagon(size=200):
    cx, cy = size // 2, size // 2
    r = size * 0.38
    pts = []
    for k in range(8):
        ang = (math.pi / 8.0) + k * (math.pi / 4.0)
        pts.append([int(cx + r * math.cos(ang)), int(cy + r * math.sin(ang))])
    return np.array(pts, dtype=np.int32).reshape((-1, 1, 2))

IDEAL_OCT = make_ideal_octagon(240)

# ==========================================
# FUNCIONES DE VISIÓN
# ==========================================
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

def detectar_stop(frame):
    global stop_detectado
    h, w = frame.shape[:2]
    mask = red_mask(frame)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    found = False
    for c in contours:
        area = cv2.contourArea(c)
        if area < MIN_AREA_FRAC * (h * w): continue
        
        peri = cv2.arcLength(c, True)
        approx = cv2.approxPolyDP(c, EPS_FRAC * peri, True)
        
        if len(approx) in ALLOW_SIDES and cv2.isContourConvex(approx):
            x, y, bw, bh = cv2.boundingRect(approx)
            aspect = bw / float(bh)
            solidity = area / (cv2.contourArea(cv2.convexHull(approx)) + 1e-6)
            dist = cv2.matchShapes(approx, IDEAL_OCT, cv2.CONTOURS_MATCH_I1, 0.0)
            
            if ASPECT_MIN <= aspect <= ASPECT_MAX and solidity > MIN_SOLIDITY and dist < MAX_SHAPE_DIST:
                cv2.rectangle(frame, (x, y), (x+bw, y+bh), (0, 255, 0), 2)
                cv2.putText(frame, "STOP DETECTED", (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                found = True
                break
    stop_detectado = found
    return frame

# ==========================================
# HILOS DE CONTROL Y LUCES
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

def is_data():
    return select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], [])

def hilo_control_teclado():
    global velocidad_actual, direccion_actual, ejecutando, luces_blancas_on
    old_settings = termios.tcgetattr(sys.stdin)
    last_s_press = 0
    
    try:
        tty.setcbreak(sys.stdin.fileno())
        while ejecutando:
            if time.time() - last_s_press > 0.8:
                pi.write(LED_ROJO, 0)

            if is_data():
                char = sys.stdin.read(1).lower()
                if char == 's':
                    pi.write(LED_ROJO, 1)
                    last_s_press = time.time()
                    if velocidad_actual > MAX_REVERSA: velocidad_actual -= PASO_VELOCIDAD
                elif char == 'w' and not stop_detectado:
                    if velocidad_actual < MAX_AVANCE: velocidad_actual += PASO_VELOCIDAD
                elif char == 'a': direccion_actual = IZQ
                elif char == 'd': direccion_actual = DER
                elif char == 'c': direccion_actual = NEUTRO
                elif char == 'l':
                    luces_blancas_on = not luces_blancas_on
                    pi.write(LED_BLANCO, 1 if luces_blancas_on else 0)
                elif char == ' ': velocidad_actual = NEUTRO
                elif char == 'q': ejecutando = False
            
            # Si hay STOP, frenar automáticamente
            if stop_detectado and velocidad_actual > NEUTRO:
                velocidad_actual = NEUTRO
                pi.write(LED_ROJO, 1)

            pi.set_servo_pulsewidth(ESC_PIN, velocidad_actual)
            pi.set_servo_pulsewidth(SERVO_PIN, direccion_actual)
            time.sleep(0.01)
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

# ==========================================
# BUCLE PRINCIPAL (VISIÓN)
# ==========================================
def main():
    global ejecutando
    cap = cv2.VideoCapture(CAM_INDEX)
    
    # Iniciar hilos secundarios
    threading.Thread(target=hilo_intermitentes, daemon=True).start()
    threading.Thread(target=hilo_control_teclado, daemon=True).start()

    # Armado ESC
    pi.set_servo_pulsewidth(ESC_PIN, NEUTRO)
    time.sleep(2)

    print("Sistema Iniciado. 'Q' para salir. Control: WASD")

    while ejecutando:
        ret, frame = cap.read()
        if not ret: break

        # Procesar Frame
        frame_procesado = detectar_stop(frame)

        # Mostrar Ventana
        cv2.imshow("RealSense D435 - RC Control", frame_procesado)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            ejecutando = False
            break

    # Limpieza final
    cap.release()
    cv2.destroyAllWindows()
    pi.set_servo_pulsewidth(ESC_PIN, NEUTRO)
    pi.set_servo_pulsewidth(SERVO_PIN, NEUTRO)
    for p in [LED_BLANCO, LED_ROJO, LED_GIRO_IZQ, LED_GIRO_DER]: pi.write(p, 0)
    pi.stop()

if __name__ == "__main__":
    main()
