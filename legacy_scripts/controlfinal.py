import pigpio
import time
import cv2
import numpy as np
import pyrealsense2 as rs

# ==========================================
# CONFIGURACIÓN DE PINES (GPIO)
# ==========================================
ESC_PIN = 18
SERVO_PIN = 13
LED_BLANCO = 17
LED_ROJO = 27
LED_GIRO_IZQ = 22
LED_GIRO_DER = 23
BTN_AVANZAR = 5
BTN_RETROCEDER = 6
BTN_IZQ = 26
BTN_DER = 16

# Valores PWM
NEUTRO = 1500
MAX_AVANCE_BASE = 1700 # Velocidad máxima original
MAX_REVERSA = 1300
IZQ = 1250
DER = 1750

pi = pigpio.pi()

# Configuración de Salidas y Entradas
for p in [ESC_PIN, SERVO_PIN, LED_BLANCO, LED_ROJO, LED_GIRO_IZQ, LED_GIRO_DER]:
    pi.set_mode(p, pigpio.OUTPUT)
for p in [BTN_AVANZAR, BTN_RETROCEDER, BTN_IZQ, BTN_DER]:
    pi.set_mode(p, pigpio.INPUT)
    pi.set_pull_up_down(p, pigpio.PUD_UP)

pi.write(LED_BLANCO, 1)

# ==========================================
# CONFIGURACIÓN DE INTEL REALSENSE
# ==========================================
pipeline = rs.pipeline()
config = rs.config()

# Forzar el uso del dispositivo en el índice deseado o por USB
# Nota: RealSense se gestiona por número de serie, pero configuramos el stream RGB y Depth
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

# Iniciar pipeline
profile = pipeline.start(config)
align_to = rs.stream.color
align = rs.align(align_to)

def get_distance_and_detect():
    """Detecta octágonos rojos y devuelve la distancia al objeto."""
    frames = pipeline.wait_for_frames()
    aligned_frames = align.process(frames)
    
    color_frame = aligned_frames.get_color_frame()
    depth_frame = aligned_frames.get_depth_frame()
    
    if not color_frame or not depth_frame:
        return None, None

    # Convertir a numpy
    img = np.asanyarray(color_frame.get_data())
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    # Rango de color rojo (el rojo está en ambos extremos del espectro HSV)
    lower_red1 = np.array([0, 120, 70])
    upper_red1 = np.array([10, 255, 255])
    lower_red2 = np.array([170, 120, 70])
    upper_red2 = np.array([180, 255, 255])

    mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
    mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
    mask = mask1 + mask2

    # Suavizado para reducir ruido
    mask = cv2.GaussianBlur(mask, (5, 5), 0)
    contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    distancia_detectada = None

    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area > 1000: # Filtro de tamaño mínimo
            # Aproximación de polígono
            epsilon = 0.02 * cv2.arcLength(cnt, True)
            approx = cv2.approxPolyDP(cnt, epsilon, True)
            
            # Si tiene 8 lados, es un octágono
            if len(approx) == 8:
                x, y, w, h = cv2.boundingRect(approx)
                cx, cy = x + w//2, y + h//2
                
                # Obtener distancia en metros del centro del octágono
                distancia_detectada = depth_frame.get_distance(cx, cy)
                
                # Dibujar para feedback (opcional si tienes monitor)
                cv2.drawContours(img, [approx], 0, (0, 255, 0), 3)
                break

    return distancia_detectada, img

def calcular_limite_velocidad(distancia):
    """Calcula el PWM máximo basado en la distancia (en metros)."""
    if distancia is None:
        return MAX_AVANCE_BASE
    
    # Lógica de seguridad:
    if distancia < 0.5:     # Menos de 50cm: Frenado total
        return NEUTRO
    elif distancia < 1.5:   # Entre 50cm y 1.5m: Velocidad muy reducida
        return 1560
    elif distancia < 3.0:   # Entre 1.5m y 3m: Velocidad media
        return 1620
    else:                   # Lejos: Velocidad máxima permitida
        return MAX_AVANCE_BASE

def controlar():
    print("Sistema iniciado con RealSense D435.")
    max_avance_dinamico = MAX_AVANCE_BASE
    
    try:
        while True:
            # 0. PROCESAMIENTO DE IMAGEN
            dist, frame = get_distance_and_detect()
            max_avance_dinamico = calcular_limite_velocidad(dist)
            
            if dist:
                print(f"Octágono Rojo detectado a: {dist:.2f}m. Max PWM: {max_avance_dinamico}")

            # 1. LÓGICA DE VELOCIDAD
            if pi.read(BTN_AVANZAR) == 0:
                velocidad = max_avance_dinamico
                pi.write(LED_ROJO, 0)
            elif pi.read(BTN_RETROCEDER) == 0:
                velocidad = MAX_REVERSA
                pi.write(LED_ROJO, 1)
            else:
                velocidad = NEUTRO
                pi.write(LED_ROJO, 0)

            # 2. LÓGICA DE DIRECCIÓN
            parpadeo = (int(time.time() * 2) % 2)
            if pi.read(BTN_IZQ) == 0:
                direccion = IZQ
                pi.write(LED_GIRO_IZQ, parpadeo)
                pi.write(LED_GIRO_DER, 0)
            elif pi.read(BTN_DER) == 0:
                direccion = DER
                pi.write(LED_GIRO_DER, parpadeo)
                pi.write(LED_GIRO_IZQ, 0)
            else:
                direccion = NEUTRO
                pi.write(LED_GIRO_IZQ, 0)
                pi.write(LED_GIRO_DER, 0)

            # 3. ACTUALIZAR HARDWARE
            pi.set_servo_pulsewidth(ESC_PIN, velocidad)
            pi.set_servo_pulsewidth(SERVO_PIN, direccion)
            
            # Mostrar cámara (opcional, comentar para ahorrar recursos)
            if frame is not None:
                cv2.imshow("Deteccion", frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

            time.sleep(0.05)

    except KeyboardInterrupt:
        print("\nDeteniendo sistema...")
    finally:
        pipeline.stop()
        pi.set_servo_pulsewidth(ESC_PIN, NEUTRO)
        pi.set_servo_pulsewidth(SERVO_PIN, NEUTRO)
        for p in [LED_BLANCO, LED_ROJO, LED_GIRO_IZQ, LED_GIRO_DER]:
            pi.write(p, 0)
        pi.stop()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    controlar()
