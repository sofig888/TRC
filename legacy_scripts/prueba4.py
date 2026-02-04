import cv2
import numpy as np
import math
import time
import pigpio

# ======================
# CONFIGURACIÓN MOTOR (ESC)
# ======================
ESC_PIN = 18
NEUTRO = 1500
ADELANTE = 1620 # Valor suave
pi = pigpio.pi()

if not pi.connected:
    print("Error: Ejecuta 'sudo pigpiod' antes de iniciar.")
    exit()

# ======================
# CONFIGURACIÓN DOBLE ROI
# ======================
ROI_TOP = 0.05      # 5% desde arriba (subido)
ROI_BOTTOM = 0.35   # Termina al 35% de la imagen (más corto)
ROI_WIDTH = 0.30    # 30% de ancho para cada lado

# Parámetros de detección (Punto 1 y 6)
MIN_AREA_FRAC = 0.0005
MAX_SHAPE_DIST = 0.35
S_MIN, V_MIN = 100, 70

def make_ideal_octagon(size=200):
    cx, cy = size // 2, size // 2
    r = size * 0.38
    pts = []
    for k in range(8):
        ang = (math.pi / 8.0) + k * (math.pi / 4.0)
        pts.append([int(cx + r * math.cos(ang)), int(cy + r * math.sin(ang))])
    return np.array(pts, dtype=np.int32).reshape((-1, 1, 2))

IDEAL_OCT = make_ideal_octagon(240)

def detect_octagon_stop(frame_roi):
    if frame_roi.size == 0: return None
    h, w = frame_roi.shape[:2]
    area_total = h * w
    
    hsv = cv2.cvtColor(frame_roi, cv2.COLOR_BGR2HSV)
    m1 = cv2.inRange(hsv, np.array([0, S_MIN, V_MIN]), np.array([10, 255, 255]))
    m2 = cv2.inRange(hsv, np.array([170, S_MIN, V_MIN]), np.array([180, 255, 255]))
    mask = cv2.bitwise_or(m1, m2)
    
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for c in contours:
        area = cv2.contourArea(c)
        if area < MIN_AREA_FRAC * area_total: continue
        
        peri = cv2.arcLength(c, True)
        approx = cv2.approxPolyDP(c, 0.02 * peri, True)
        
        dist = cv2.matchShapes(approx, IDEAL_OCT, cv2.CONTOURS_MATCH_I1, 0.0)
        if dist < MAX_SHAPE_DIST:
            x, y, bw, bh = cv2.boundingRect(approx)
            return (x, y, bw, bh, approx)
    return None

def main():
    # Punto 4: Resolución nativa RealSense
    cap = cv2.VideoCapture(4, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 848)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    
    print("Armando ESC... Espera 3 segundos")
    pi.set_servo_pulsewidth(ESC_PIN, NEUTRO)
    time.sleep(3)

    try:
        while True:
            ret, frame = cap.read()
            if not ret: continue

            h, w = frame.shape[:2]
            y1, y2 = int(h * ROI_TOP), int(h * ROI_BOTTOM)
            
            # ROI Izquierda
            x1_L, x2_L = 0, int(w * ROI_WIDTH)
            roi_left = frame[y1:y2, x1_L:x2_L]
            
            # ROI Derecha
            x1_R, x2_R = int(w * (1 - ROI_WIDTH)), w
            roi_right = frame[y1:y2, x1_R:x2_R]

            # Detección en ambas zonas
            res_L = detect_octagon_stop(roi_left)
            res_R = detect_octagon_stop(roi_right)

            if res_L or res_R:
                # ACCIÓN: STOP
                pi.set_servo_pulsewidth(ESC_PIN, NEUTRO)
                
                # Feedback visual
                if res_L:
                    x, y, bw, bh, _ = res_L
                    cv2.rectangle(frame, (x + x1_L, y + y1), (x + bw + x1_L, y + bh + y1), (0, 0, 255), 3)
                if res_R:
                    x, y, bw, bh, _ = res_R
                    cv2.rectangle(frame, (x + x1_R, y + y1), (x + bw + x1_R, y + bh + y1), (0, 0, 255), 3)
                
                cv2.putText(frame, "STOP DETECTADO", (int(w/2)-100, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,255), 3)
            else:
                # ACCIÓN: AVANZAR
                pi.set_servo_pulsewidth(ESC_PIN, ADELANTE)

            # Dibujar rectángulos de las ROI para calibración
            cv2.rectangle(frame, (x1_L, y1), (x2_L, y2), (255, 255, 0), 2)
            cv2.rectangle(frame, (x1_R, y1), (x2_R, y2), (255, 255, 0), 2)
            
            cv2.imshow("Vision Dual ROI", frame)
            if cv2.waitKey(1) & 0xFF == 27: break

    finally:
        print("\nDeteniendo motor...")
        pi.set_servo_pulsewidth(ESC_PIN, NEUTRO)
        time.sleep(0.5)
        pi.set_servo_pulsewidth(ESC_PIN, 0)
        pi.stop()
        cap.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
