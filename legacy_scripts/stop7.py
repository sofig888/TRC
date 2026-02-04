import pyrealsense2 as rs
import numpy as np
import cv2

# Configuración de la RealSense
pipeline = rs.pipeline()
config = rs.config()

# Usamos una resolución equilibrada para mantener altos FPS en la Pi 4B
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

profile = pipeline.start(config)

# --- OPTIMIZACIÓN PARA MOVIMIENTO RÁPIDO ---
# Acceder al sensor de color para ajustar parámetros manuales
color_sensor = profile.get_device().query_sensors()[1] 
# Desactivar auto-exposición para evitar desenfoque (motion blur)
color_sensor.set_option(rs.option.enable_auto_exposure, 0)
# Ajustar exposición (Un valor bajo como 150-200 evita el blur, pero oscurece la imagen)
color_sensor.set_option(rs.option.exposure, 150) 
# Aumentar la ganancia para compensar la oscuridad de la exposición baja
color_sensor.set_option(rs.option.gain, 64)

align = rs.align(rs.stream.color)

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
        
        # Rangos de rojo (ajustados levemente por la exposición manual)
        lower_red1 = np.array([0, 130, 60])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([160, 130, 60])
        upper_red2 = np.array([180, 255, 255])
        
        mask = cv2.addWeighted(cv2.inRange(hsv, lower_red1, upper_red1), 1.0, 
                               cv2.inRange(hsv, lower_red2, upper_red2), 1.0, 0)
        
        # --- MEJORA DE MÁSCARA ---
        # Dilatamos un poco más para cerrar huecos causados por el movimiento
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.dilate(mask, kernel, iterations=1)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        for cnt in contours:
            if cv2.contourArea(cnt) < 600:
                continue
                
            # Suavizamos la aproximación para que sea menos sensible a bordes rugosos
            epsilon = 0.025 * cv2.arcLength(cnt, True)
            approx = cv2.approxPolyDP(cnt, epsilon, True)
            
            # Buscamos el octágono
            if len(approx) == 8 and cv2.isContourConvex(approx):
                x, y, w, h = cv2.boundingRect(approx)
                
                # Relación de aspecto (evita que detecte cosas muy alargadas)
                aspect_ratio = float(w)/h
                if 0.8 <= aspect_ratio <= 1.2:
                    cx, cy = x + w // 2, y + h // 2
                    distancia = depth_frame.get_distance(cx, cy)
                    
                    # Visualización (opcional, pero no afecta a la terminal)
                    cv2.drawContours(img, [approx], 0, (0, 255, 0), 3)
                    cv2.putText(img, f"{distancia:.2f}m", (x, y - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        cv2.imshow("Deteccion Optimizada", img)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    pipeline.stop()
    cv2.destroyAllWindows()
