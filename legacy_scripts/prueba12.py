import pigpio
import sys
import tty
import termios
import time
import cv2
import threading
import numpy as np

# --- Configuración de Hardware ---
ESC_PIN = 18
SERVO_PIN = 13
NEUTRO = 1500
MAX_AVANCE_LIBRE = 1800  # Velocidad máxima cuando no hay STOP
MAX_REVERSA = 1200
PASO_VELOCIDAD = 1
IZQ = 1200
DER = 1800

pi = pigpio.pi()

# --- Variables de estado globales ---
velocidad_actual = NEUTRO
direccion_actual = NEUTRO
# Esta variable limitará qué tan rápido puedes ir según la cámara
velocidad_max_permitida = MAX_AVANCE_LIBRE
ejecutando = True

def getch():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

# --- Hilo de Visión Artificial (Detección de STOP) ---
def hilo_vision_autonoma():
    global ejecutando, velocidad_max_permitida, velocidad_actual
    
    # Abrir cámara RealSense (Index 4 como pediste)
    cap = cv2.VideoCapture(4)
    if not cap.isOpened():
        print("\n[Error] No se detecta la cámara en el índice 4")
        return

    print("[Visión] Buscando señales de STOP...")

    while ejecutando:
        ret, frame = cap.read()
        if not ret: break

        # 1. Procesamiento para detectar el color ROJO
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # Rangos del rojo (el rojo está en dos partes del espectro HSV)
        bajo_rojo1 = np.array([0, 120, 70])
        alto_rojo1 = np.array([10, 255, 255])
        bajo_rojo2 = np.array([170, 120, 70])
        alto_rojo2 = np.array([180, 255, 255])
        
        mask1 = cv2.inRange(hsv, bajo_rojo1, alto_rojo1)
        mask2 = cv2.inRange(hsv, bajo_rojo2, alto_rojo2)
        mask = mask1 + mask2
        
        # 2. Encontrar contornos del objeto rojo más grande
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        stop_detectado = False
        if contours:
            c = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(c)
            
            if area > 2000: # Subimos un poco el área mínima
                # A. Filtro de Proporción (¿Es cuadrado?)
                x, y, w, h = cv2.boundingRect(c)
                proporcion = float(w) / h
                
                # B. Filtro de Complejidad (¿Es un octágono/círculo?)
                perimetro = cv2.arcLength(c, True)
                aproximacion = cv2.approxPolyDP(c, 0.02 * perimetro, True)
                vertices = len(aproximacion)

                # Solo aceptamos si la proporción es cercana a un cuadrado (0.8 a 1.2)
                # y si tiene entre 6 y 10 vértices (un octágono real suele verse así por el ruido)
                if 0.7 < proporcion < 1.3 and 6 <= vertices <= 10:
                    stop_detectado = True
                    cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
                
                # --- LÓGICA DE VELOCIDAD DINÁMICA ---
                # A mayor área, el STOP está más cerca.
                # Área pequeña (lejos) -> ej: 2000 | Área grande (frente) -> ej: 40000
                if area < 5000:
                    velocidad_max_permitida = MAX_AVANCE_LIBRE
                elif 5000 <= area < 30000:
                    # Mapeo: reduce la velocidad gradualmente
                    # Cuanto más grande es el área, más baja el límite
                    factor_reduccion = (area - 5000) / 25000
                    rango_velocidad = MAX_AVANCE_LIBRE - 1580
                    velocidad_max_permitida = int(MAX_AVANCE_LIBRE - (factor_reduccion * rango_velocidad))
                else:
                    # Demasiado cerca: Frenado total
                    velocidad_max_permitida = NEUTRO
                
                # Aplicar el límite inmediatamente si la velocidad actual es mayor
                if velocidad_actual > velocidad_max_permitida:
                    velocidad_actual = velocidad_max_permitida
                    pi.set_servo_pulsewidth(ESC_PIN, velocidad_actual)

                cv2.putText(frame, f"STOP! Max V: {velocidad_max_permitida}", (x, y-10), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

        if not stop_detectado:
            # Si no hay STOP, la velocidad máxima vuelve a ser la normal
            velocidad_max_permitida = MAX_AVANCE_LIBRE

        # Info en pantalla
        cv2.putText(frame, f"V. Actual: {velocidad_actual} | Limite: {velocidad_max_permitida}", 
                    (10, 450), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        
        cv2.imshow("Conduccion Inteligente - RGB", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            ejecutando = False
            break

    cap.release()
    cv2.destroyAllWindows()

# --- Control de Teclado ---
def controlar_coche():
    global velocidad_actual, direccion_actual, ejecutando, velocidad_max_permitida
    
    # Hilo de visión
    t = threading.Thread(target=hilo_vision_autonoma)
    t.start()

    print("--- MODO IA ACTIVO (DETECCION DE STOP) ---")
    pi.set_servo_pulsewidth(ESC_PIN, NEUTRO)
    pi.set_servo_pulsewidth(SERVO_PIN, NEUTRO)
    time.sleep(3)

    try:
        while ejecutando:
            char = getch().lower()

            if char == 'w':
                # Solo aumentamos si no hemos superado el límite impuesto por la cámara
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
        pi.stop()

if __name__ == "__main__":
    controlar_coche()
