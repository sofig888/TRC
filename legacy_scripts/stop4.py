import cv2
import numpy as np
import pyrealsense2 as rs

# --- Configuración de RealSense ---
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# Iniciar la cámara
profile = pipeline.start(config)
align_to = rs.stream.color
align = rs.align(align_to)

def detectar_stop(frame, depth_frame):
    # 1. Preprocesamiento: Convertir a HSV y filtrar color Rojo
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    # El rojo está en dos rangos del espectro HSV
    bajo_rojo1 = np.array([0, 110, 80])
    alto_rojo1 = np.array([10, 255, 255])
    bajo_rojo2 = np.array([170, 110, 80])
    alto_rojo2 = np.array([180, 255, 255])
    
    mask1 = cv2.inRange(hsv, bajo_rojo1, alto_rojo1)
    mask2 = cv2.inRange(hsv, bajo_rojo2, alto_rojo2)
    mask = cv2.add(mask1, mask2)
    
    # Limpieza de ruido
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

    # 2. Hallar Contornos
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area > 1000:  # Ignorar objetos muy pequeños
            # Aproximar el polígono
            epsilon = 0.02 * cv2.arcLength(cnt, True)
            approx = cv2.approxPolyDP(cnt, epsilon, True)
            
            # 3. Verificar si es un Octágono (8 lados)
            # A veces por perspectiva detecta entre 7 y 10 vértices
            if 7 <= len(approx) <= 10:
                x, y, w, h = cv2.boundingRect(approx)
                
                # Obtener la distancia en el centro del objeto
                centro_x = int(x + w/2)
                centro_y = int(y + h/2)
                distancia = depth_frame.get_distance(centro_x, centro_y)

                # Dibujar resultados
                cv2.drawContours(frame, [approx], 0, (0, 255, 0), 3)
                cv2.putText(frame, f"STOP! Dist: {distancia:.2f}m", (x, y - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                cv2.circle(frame, (centro_x, centro_y), 5, (255, 0, 0), -1)

    return frame

try:
    while True:
        # Esperar frames
        frames = pipeline.wait_for_frames()
        aligned_frames = align.process(frames)
        
        depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()
        
        if not depth_frame or not color_frame:
            continue

        # Convertir a numpy
        color_image = np.asanyarray(color_frame.get_data())
        
        # Procesar detección
        resultado = detectar_stop(color_image, depth_frame)

        # Mostrar imagen
        cv2.imshow('Deteccion RealSense', resultado)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    pipeline.stop()
    cv2.destroyAllWindows()
