import pyrealsense2 as rs
import numpy as np
import cv2

# ==========================================
# CONFIGURACIÓN CALIBRADA
# ==========================================
AZUL_BAJO = np.array([82, 62, 110])
AZUL_ALTO = np.array([135, 255, 255])
AREA_MINIMA_LINEA = 500  # Ignorar ruidos pequeños

pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
pipeline.start(config)

print("Modo Calibración de Carril - Activo")
print("Coloca el carro en el centro ideal de la pista.")

try:
    while True:
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame: continue

        img = np.asanyarray(color_frame.get_data())
        h, w, _ = img.shape
        centro_camara = w // 2
        mitad_y = h // 2

        # 1. Procesamiento de Máscara (Solo mitad inferior para ahorrar CPU)
        roi = img[mitad_y:, :]
        blur = cv2.GaussianBlur(roi, (5, 5), 0)
        hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, AZUL_BAJO, AZUL_ALTO)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((5,5), np.uint8), iterations=2)

        # 2. Encontrar Contornos (Las líneas)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        centros_lineas = []

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > AREA_MINIMA_LINEA:
                M = cv2.moments(cnt)
                if M["m00"] != 0:
                    # Calcular centro X de la línea
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"]) + mitad_y # Ajustar por el corte de ROI
                    centros_lineas.append(cx)
                    
                    # Dibujar círculo en el centro de cada línea detectada
                    cv2.circle(img, (cx, cy), 10, (255, 0, 0), -1)

        # 3. Lógica de Centrado
        img_final = img.copy()
        error = 0

        if len(centros_lineas) >= 2:
            # Ordenamos los centros de izquierda a derecha
            centros_lineas.sort()
            linea_izq = centros_lineas[0]
            linea_der = centros_lineas[-1] # Tomamos la más lejana como derecha
            
            punto_medio_pista = (linea_izq + linea_der) // 2
            error = punto_medio_pista - centro_camara
            
            # Dibujar visualización de la pista
            cv2.line(img_final, (linea_izq, mitad_y), (linea_der, mitad_y), (0, 255, 255), 2)
            cv2.circle(img_final, (punto_medio_pista, mitad_y), 15, (0, 255, 0), -1) # Objetivo
        
        elif len(centros_lineas) == 1:
            # Si solo ve una línea, el error se basa en una distancia fija pre-calibrada
            punto_medio_pista = centros_lineas[0]
            cv2.putText(img_final, "SOLO 1 LINEA - PERDIENDO REFERENCIA", (10, 60), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

        # 4. Interfaz de usuario de Calibración
        # Línea central de la cámara (Roja)
        cv2.line(img_final, (centro_camara, 0), (centro_camara, h), (0, 0, 255), 1)
        
        # Mostrar Error
        color_texto = (0, 255, 0) if abs(error) < 10 else (0, 255, 255)
        cv2.putText(img_final, f"ERROR: {error} px", (10, 30), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, color_texto, 2)

        cv2.imshow("Calibracion de Pista", img_final)
        if cv2.waitKey(1) & 0xFF == ord('q'): break

finally:
    pipeline.stop()
    cv2.destroyAllWindows()
