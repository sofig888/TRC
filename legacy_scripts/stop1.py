import pyrealsense2 as rs
import numpy as np
import cv2
from ultralytics import YOLO

# 1. Cargar el modelo YOLOv8 (el modelo nano es el más rápido para RPi)
# La clase 11 en el dataset COCO es 'stop sign'
model = YOLO('yolov8n.pt')

# 2. Configurar la RealSense D435
pipeline = rs.pipeline()
config = rs.config()

# Si tienes múltiples cámaras, aquí podrías usar el índice/serial
# config.enable_device('SERIAL_NUMBER') 

config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# Iniciar la cámara
profile = pipeline.start(config)

# Crear un objeto de alineación (alinear profundidad al color)
align_to = rs.stream.color
align = rs.align(align_to)

try:
    while True:
        # Esperar frames y alinearlos
        frames = pipeline.wait_for_frames()
        aligned_frames = align.process(frames)
        
        depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()
        
        if not depth_frame or not color_frame:
            continue

        # Convertir a arreglos de numpy
        color_image = np.asanyarray(color_frame.get_data())
        
        # 3. Realizar la detección
        results = model(color_image, verbose=False)

        for r in results:
            for box in r.boxes:
                # Obtener el ID de la clase detectada
                class_id = int(box.cls[0])
                
                # Clase 11 es 'stop sign' en el modelo pre-entrenado
                if class_id == 11:
                    conf = box.conf[0]
                    if conf > 0.5: # Umbral de confianza
                        # Coordenadas de la caja (bounding box)
                        x1, y1, x2, y2 = map(int, box.xyxy[0])
                        
                        # Calcular el centro de la señal para medir distancia
                        center_x = int((x1 + x2) / 2)
                        center_y = int((y1 + y2) / 2)
                        
                        # Obtener la distancia en metros
                        distancia = depth_frame.get_distance(center_x, center_y)

                        # Dibujar resultados
                        cv2.rectangle(color_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                        label = f"STOP: {distancia:.2f}m"
                        cv2.putText(color_image, label, (x1, y1 - 10), 
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                        
                        # Círculo en el punto de medición
                        cv2.circle(color_image, (center_x, center_y), 5, (0, 0, 255), -1)

        # Mostrar la imagen
        cv2.imshow('Deteccion de Señales - RealSense', color_image)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    pipeline.stop()
    cv2.destroyAllWindows()
