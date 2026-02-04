import pyrealsense2 as rs
import numpy as np
import cv2

# Configuración de la RealSense
pipeline = rs.pipeline()
config = rs.config()

# Resolución óptima para FPS estables en Raspberry Pi 4B
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

profile = pipeline.start(config)

# --- CONFIGURACIÓN DE SENSORES PARA MOVIMIENTO ---
color_sensor = profile.get_device().query_sensors()[1]
color_sensor.set_option(rs.option.enable_auto_exposure, 0)
color_sensor.set_option(rs.option.exposure, 150) # Exposición baja = No motion blur
color_sensor.set_option(rs.option.gain, 64)

# Alineación de profundidad al color
align = rs.align(rs.stream.color)

def obtener_distancia_robusta(depth_frame, cx, cy):
    """Busca una distancia válida en un área de 5x5 alrededor del centro"""
    dist = depth_frame.get_distance(cx, cy)
    if dist > 0:
        return dist
    
    # Si el centro es 0, buscamos en los píxeles vecinos
    for dx in range(-2, 3, 2):
        for dy in range(-2, 3, 2):
            val = depth_frame.get_distance(cx + dx, cy + dy)
            if val > 0:
                return val
    return 0

try:
    while True:
        frames = pipeline.wait_for_frames()
        aligned_frames = align.process(frames)
        
        color_frame = aligned_frames.get_color_frame()
        depth_frame = aligned_frames.get_depth_frame()
        
        if not color_frame or not depth_frame:
            continue

        img = np.asanyarray(color_frame.get_data())
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        
        # Filtrado de Rojo
        mask1 = cv2.inRange(hsv, np.array([0, 130, 60]), np.array([10, 255, 255]))
        mask2 = cv2.inRange(hsv, np.array([160, 130, 60]), np.array([180, 255, 255]))
        mask = cv2.addWeighted(mask1, 1.0, mask2, 1.0, 0)
        
        # Limpieza rápida de imagen
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.dilate(mask, kernel, iterations=1)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        for cnt in contours:
            if cv2.contourArea(cnt) < 600:
                continue
                
            epsilon = 0.025 * cv2.arcLength(cnt, True)
            approx = cv2.approxPolyDP(cnt, epsilon, True)
            
            # Filtro: 8 vértices, Convexo y Relación de Aspecto Cuadrada
            if len(approx) == 8 and cv2.isContourConvex(approx):
                x, y, w, h = cv2.boundingRect(approx)
                
                if 0.75 <= float(w)/h <= 1.25:
                    cx, cy = x + w // 2, y + h // 2
                    
                    # Evitar errores si el objeto está en el borde de la imagen
                    if 0 < cx < 640 and 0 < cy < 480:
                        distancia = obtener_distancia_robusta(depth_frame, cx, cy)
                        
                        # Dibujar en pantalla (Verde si hay distancia, Rojo si es 0)
                        color_dibujo = (0, 255, 0) if distancia > 0 else (0, 0, 255)
                        cv2.drawContours(img, [approx], 0, color_dibujo, 3)
                        
                        texto = f"STOP: {distancia:.2f}m" if distancia > 0 else "STOP: FUERA DE RANGO"
                        cv2.putText(img, texto, (x, y - 10),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, color_dibujo, 2)

        # Mostrar FPS en pantalla (Opcional, ayuda a monitorear la estabilidad)
        cv2.imshow("Deteccion RealSense (Octagonos)", img)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    pipeline.stop()
    cv2.destroyAllWindows()
