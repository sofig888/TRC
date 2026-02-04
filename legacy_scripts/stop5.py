import cv2
import numpy as np
import math

def detectar_stop_estricto(frame, depth_frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    # Filtro de rojo muy saturado para evitar tonos piel o naranja suave
    bajo_rojo1 = np.array([0, 160, 100])
    alto_rojo1 = np.array([6, 255, 255])
    bajo_rojo2 = np.array([170, 160, 100])
    alto_rojo2 = np.array([180, 255, 255])
    
    mask = cv2.add(cv2.inRange(hsv, bajo_rojo1, alto_rojo1), 
                   cv2.inRange(hsv, bajo_rojo2, alto_rojo2))

    # Limpieza agresiva: elimina motas pequeñas rojas
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.dilate(mask, kernel, iterations=1)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    for cnt in contours:
        area = cv2.contourArea(cnt)
        perimetro = cv2.arcLength(cnt, True)
        
        if area > 1200 and perimetro > 0:
            # 1. FILTRO DE CIRCULARIDAD
            # Un octágono regular tiene una circularidad de ~0.906
            circularidad = (4 * math.pi * area) / (perimetro * perimetro)
            
            # 2. FILTRO DE EXTENT (Llenado del rectángulo)
            x, y, w, h = cv2.boundingRect(cnt)
            extent = float(area) / (w * h)

            # Si pasa los filtros de forma general
            if 0.75 < circularidad < 0.95 and 0.70 < extent < 0.85:
                
                # 3. APROXIMACIÓN FINAL POR VÉRTICES
                epsilon = 0.025 * perimetro
                approx = cv2.approxPolyDP(cnt, epsilon, True)
                lados = len(approx)

                # Un octágono bajo perspectiva puede verse de 7 a 9 lados
                if 7 <= lados <= 9:
                    # Validar distancia con RealSense
                    centro_x, centro_y = int(x + w/2), int(y + h/2)
                    distancia = depth_frame.get_distance(centro_x, centro_y)
                    
                    if distancia > 0: # Evitar lecturas nulas
                        cv2.drawContours(frame, [approx], 0, (0, 255, 0), 4)
                        cv2.putText(frame, f"STOP {distancia:.2f}m", (x, y-10),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                                    
    return frame
