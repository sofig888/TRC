import cv2
import numpy as np
import math
import time
import pigpio

# ==========================================
# 1. CONFIGURACIÓN DE HARDWARE (MOTOR/ESC)
# ==========================================
ESC_PIN = 18
NEUTRO = 1500
ADELANTE = 1620  # Ajusta este valor según la velocidad deseada
pi = pigpio.pi()

# ==========================================
# 2. CONFIGURACIÓN DE VISIÓN (PUNTOS 1, 4 y 6)
# ==========================================
S_MIN, V_MIN = 100, 70      # Punto 6: Sensibilidad en baja saturación
MIN_AREA_FRAC = 0.0004      # Punto 1: Detecta señales muy pequeñas (lejos)
MAX_SHAPE_DIST = 0.35       # Punto 1: Tolerancia a la forma pixelada

# Configuración de Doble ROI (Regiones de Interés)
ROI_TOP = 0.05              # Punto 2: Empieza casi arriba de la imagen (5%)
ROI_BOTTOM = 0.40           # Termina al 40% de la altura (esquinas superiores)
ROI_WIDTH_FRAC = 0.30       # Cada ROI ocupa el 30% del ancho lateral

# ==========================================
# 3. FUNCIONES DE APOYO
# ==========================================
def make_ideal_octagon(size=240):
    """Crea un contorno de octágono perfecto para comparar formas."""
    cx, cy = size // 2, size // 2
    r = size * 0.38
    pts = []
    for k in range(8):
        ang = (math.pi / 8.0) + k * (math.pi / 4.0)
        pts.append([int(cx + r * math.cos(ang)), int(cy + r * math.sin(ang))])
    return np.array(pts, dtype=np.int32).reshape((-1, 1, 2))

IDEAL_OCT = make_ideal_octagon()

def detect_octagon_stop(roi_frame):
    """Procesa una ROI buscando el octágono rojo."""
    if roi_frame.size == 0: return None
    
    h, w = roi_frame.shape[:2]
    area_total = h * w
    
    # Máscara de color rojo
    hsv = cv2.cvtColor(roi_frame, cv2.COLOR_BGR2HSV)
    m1 = cv2.inRange(hsv, np.array([0, S_MIN, V_MIN]), np.array([10, 255, 255]))
    m2 = cv2.inRange(hsv, np.array([170, S_MIN, V_MIN]), np.array([180, 255, 255]))
    mask = cv2.bitwise_or(m1, m2)
    
    # Suavizado de máscara
    kernel = np.ones((3,3), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    for c in contours:
        area = cv2.contourArea(c)
        if area < MIN_AREA_FRAC * area_total: continue
        
        peri = cv2.arcLength(c, True)
        approx = cv2.approxPolyDP(c, 0.025 * peri, True)
        
        # Comparar forma con octágono ideal
        dist = cv2.matchShapes(approx, IDEAL_OCT, cv2.CONTOURS_MATCH_I1, 0.0)
        
        if dist < MAX_SHAPE_DIST:
            return cv2.boundingRect(c) # Retorna (x, y, w, h)
    return None

# ==========================================
# 4. BUCLE PRINCIPAL
# ==========================================
def main():
    if not pi.connected:
        print("Error: pigpiod no detectado. Ejecuta 'sudo pigpiod'")
        return

    # Punto 4: Resolución nativa RealSense
    cap = cv2.VideoCapture(4, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 848)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    print("--- Iniciando Auto Escala 1:10 ---")
    print("Armando ESC (3 segundos)...")
    pi.set_servo_pulsewidth(ESC_PIN, NEUTRO)
    time.sleep(3)
    
    try:
        while True:
            ret, frame = cap.read()
            if not ret: break

            h, w = frame.shape[:2]
            
            # Definir coordenadas de las ROIs
            y1, y2 = int(h * ROI_TOP), int(h * ROI_BOTTOM)
            # Izquierda
            x1_L, x2_L = 0, int(w * ROI_WIDTH_FRAC)
            # Derecha
            x1_R, x2_R = int(w * (1 - ROI_WIDTH_FRAC)), w
            
            # Extraer ROIs
            roi_L = frame[y1:y2, x1_L:x2_L]
            roi_R = frame[y1:y2, x1_R:x2_R]

            # Detectar en ambas
            det_L = detect_octagon_stop(roi_L)
            det_R = detect_octagon_stop(roi_R)

            # --- LÓGICA DE CONTROL ---
            if det_L or det_R:
                pi.set_servo_pulsewidth(ESC_PIN, NEUTRO)
                estado = "STOP DETECTADO"
                color = (0, 0, 255)
            else:
                pi.set_servo_pulsewidth(ESC_PIN, ADELANTE)
                estado = "AVANZANDO"
                color = (0, 255, 0)

            # --- FEEDBACK VISUAL ---
            # Dibujar rectángulos de las ROIs
            cv2.rectangle(frame, (x1_L, y1), (x2_L, y2), (255, 255, 0), 2)
            cv2.rectangle(frame, (x1_R, y1), (x2_R, y2), (255, 255, 0), 2)
            
            # Dibujar detección si existe
            if det_L:
                x, y, bw, bh = det_L
                cv2.rectangle(frame, (x + x1_L, y + y1), (x + bw + x1_L, y + bh + y1), (0, 0, 255), 3)
            if det_R:
                x, y, bw, bh = det_R
                cv2.rectangle(frame, (x + x1_R, y + y1), (x + bw + x1_R, y + bh + y1), (0, 0, 255), 3)

            cv2.putText(frame, estado, (20, 450), cv2.FONT_HERSHEY_SIMPLEX, 1, color, 2)
            cv2.imshow("Control Autónomo 1:10", frame)

            if cv2.waitKey(1) & 0xFF == 27: # Presionar ESC para salir
                break

    finally:
        print("\nDeteniendo motor y cerrando...")
        pi.set_servo_pulsewidth(ESC_PIN, NEUTRO)
        time.sleep(0.1)
        pi.set_servo_pulsewidth(ESC_PIN, 0)
        pi.stop()
        cap.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
