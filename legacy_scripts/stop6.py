import pyrealsense2 as rs
import numpy as np
import cv2

# Configuración de la RealSense (Index 4 según solicitado)
pipeline = rs.pipeline()
config = rs.config()
# Forzar el índice del dispositivo si es necesario (generalmente se maneja por serial)
# config.enable_device('SERIAL_NUMBER') 

config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

# Iniciar la cámara
profile = pipeline.start(config)
align_to = rs.stream.color
align = rs.align(align_to)

print("Iniciando detección de octágonos rojos...")

try:
    while True:
        # Esperar frames y alinearlos
        frames = pipeline.wait_for_frames()
        aligned_frames = align.process(frames)
        
        color_frame = aligned_frames.get_color_frame()
        depth_frame = aligned_frames.get_depth_frame()
        
        if not color_frame or not depth_frame:
            continue

        # Convertir a numpy arrays
        img = np.asanyarray(color_frame.get_data())
        
        # 1. PASO A HSV Y FILTRADO DE ROJO
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        
        # El rojo tiene dos rangos en OpenCV HSV (0-10 y 160-180)
        lower_red1 = np.array([0, 120, 70])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([160, 120, 70])
        upper_red2 = np.array([180, 255, 255])
        
        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask = cv2.addWeighted(mask1, 1.0, mask2, 1.0, 0)
        
        # Limpiar ruido (Morfología)
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

        # 2. DETECCIÓN DE CONTORNOS
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < 500: # Ignorar objetos muy pequeños
                continue
                
            # Aproximación de polígono
            epsilon = 0.02 * cv2.arcLength(cnt, True)
            approx = cv2.approxPolyDP(cnt, epsilon, True)
            
            # 3. FILTRO ESTRICTO: OCTÁGONO (8 vértices)
            if len(approx) == 8:
                # Verificar que sea convexo (evita formas extrañas o "manos")
                if cv2.isContourConvex(approx):
                    x, y, w, h = cv2.boundingRect(approx)
                    
                    # Calcular centro para la distancia
                    cx, cy = x + w // 2, y + h // 2
                    
                    # 4. OBTENER DISTANCIA
                    distancia = depth_frame.get_distance(cx, cy)
                    
                    # Dibujar resultados
                    cv2.drawContours(img, [approx], 0, (0, 255, 0), 3)
                    cv2.putText(img, f"STOP: {distancia:.2f}m", (x, y - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                    
                    print(f"Señal detectada a: {distancia:.2f} metros")

        # Mostrar imagen
        cv2.imshow("Deteccion RealSense", img)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    pipeline.stop()
    cv2.destroyAllWindows()
