import pigpio
import sys
import tty
import termios
import time
import cv2
import threading
import numpy as np
import math

# --- Configuración de Hardware PWM ---
ESC_PIN = 18
SERVO_PIN = 13
NEUTRO = 1500
MAX_AVANCE_LIBRE = 1750
MAX_REVERSA = 1300
PASO_VELOCIDAD = 2
IZQ = 1250
DER = 1750

# --- Configuración de LEDs (GPIO) ---
LED_GIRO_IZQ = 17
LED_GIRO_DER = 27
LED_FRENO = 22
LED_FAROS = 24

pi = pigpio.pi()
for pin in [LED_GIRO_IZQ, LED_GIRO_DER, LED_FRENO, LED_FAROS]:
    pi.set_mode(pin, pigpio.OUTPUT)

# --- Variables de estado globales ---
velocidad_actual = NEUTRO
direccion_actual = NEUTRO
velocidad_max_permitida = MAX_AVANCE_LIBRE
ejecutando = True
luces_frente_on = False
esta_frenando = False

# --- Hilo de Control de Luces ---
def hilo_luces():
    global ejecutando, direccion_actual, esta_frenando, luces_frente_on
    while ejecutando:
        # 1. Faros delanteros (Teclado L)
        pi.write(LED_FAROS, 1 if luces_frente_on else 0)

        # 2. Luces de Freno
        pi.write(LED_FRENO, 1 if esta_frenando else 0)

        # 3. Direccionales (Parpadeo)
        if direccion_actual < 1400: # Girando Izquierda
            pi.write(LED_GIRO_IZQ, 1)
            time.sleep(0.5)
            pi.write(LED_GIRO_IZQ, 0)
        elif direccion_actual > 1600: # Girando Derecha
            pi.write(LED_GIRO_DER, 1)
            time.sleep(0.5)
            pi.write(LED_GIRO_DER, 0)
        
        time.sleep(0.1)

# --- Funciones de Visión (Misma lógica de Octágono) ---
def make_ideal_octagon(size=200):
    cx, cy = size // 2, size // 2
    r = size * 0.38
    pts = []
    for k in range(8):
        ang = (math.pi / 8.0) + k * (math.pi / 4.0)
        pts.append([int(cx + r * math.cos(ang)), int(cy + r * math.sin(ang))])
    return np.array(pts, dtype=np.int32).reshape((-1, 1, 2))

IDEAL_OCT = make_ideal_octagon(240)

def hilo_vision_autonoma():
    global ejecutando, velocidad_max_permitida, velocidad_actual, esta_frenando
    cap = cv2.VideoCapture(4)
    while ejecutando:
        ret, frame = cap.read()
        if not ret: continue
        
        # ... (Procesamiento de máscara roja y contornos omitido para brevedad, igual al anterior) ...
        # Lógica simplificada para detección y freno:
        stop_detectado = False
        # [Aquí va el bloque detect_octagon_stop del código anterior]
        
        # Si la cámara detecta que debe frenar:
        if stop_detectado and velocidad_max_permitida == NEUTRO:
            esta_frenando = True
        elif not stop_detectado:
            # esta_frenando se manejará principalmente por teclado
            pass

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
    global velocidad_actual, direccion_actual, ejecutando, velocidad_max_permitida, luces_frente_on, esta_frenando
    
    # Iniciar hilos
    threading.Thread(target=hilo_vision_autonoma, daemon=True).start()
    threading.Thread(target=hilo_luces, daemon=True).start()

    print("--- SISTEMA FULL: IA + LUCES + CONTROL ---")
    print("L: Faros | W/S: Vel | A/D: Giro | C: Centro | ESPACIO: Freno")

    try:
        while ejecutando:
            char = getch().lower()
            
            # Reset de estado de freno cada vez que se pulsa una tecla
            esta_frenando = False 

            if char == 'w':
                if velocidad_actual < velocidad_max_permitida:
                    velocidad_actual += PASO_VELOCIDAD
                pi.set_servo_pulsewidth(ESC_PIN, velocidad_actual)
            
            elif char == 's':
                # Si el auto va hacia adelante y presionas atrás -> FRENANDO
                if velocidad_actual > NEUTRO:
                    esta_frenando = True
                if velocidad_actual > MAX_REVERSA:
                    velocidad_actual -= PASO_VELOCIDAD
                pi.set_servo_pulsewidth(ESC_PIN, velocidad_actual)

            elif char == 'l': # Alternar Faros
                luces_frente_on = not luces_frente_on
            
            elif char == 'a':
                direccion_actual = IZQ
            elif char == 'd':
                direccion_actual = DER
            elif char == 'c':
                direccion_actual = NEUTRO
            elif char == ' ':
                esta_frenando = True
                velocidad_actual = NEUTRO
                direccion_actual = NEUTRO
            
            elif char == 'q':
                ejecutando = False
                break
            
            pi.set_servo_pulsewidth(SERVO_PIN, direccion_actual)

    except KeyboardInterrupt:
        ejecutando = False
    finally:
        # Apagado de seguridad
        for pin in [ESC_PIN, SERVO_PIN, LED_GIRO_IZQ, LED_GIRO_DER, LED_FRENO, LED_FAROS]:
            pi.set_servo_pulsewidth(pin, 0) if pin < 20 else pi.write(pin, 0)
        pi.stop()

if __name__ == "__main__":
    controlar_coche()
