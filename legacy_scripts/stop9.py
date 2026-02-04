import pyrealsense2 as rs
import numpy as np
import cv2

# ==========================================
# CONFIGURACIÓN AJUSTABLE
# ==========================================
PERSISTENCIA_LIMITE = 10  # Cuántos frames recordar el objeto si se pierde
EXPOSICION_VAL = 150      # Menor valor = menos desenfoque, pero más oscuro
GANANCIA_VAL = 64         # Aumentar si la imagen se ve muy oscura por la exposición
AREA_MINIMA = 600         # Tamaño mínimo del octágono para ser detectado

# Variables de estado para la persistencia
memoria_deteccion = {
    "contador_frames": PERSISTENCIA_LIMITE, # Empieza vacío
    "box": None,       # (x, y, w, h)
    "distancia": 0,
    "puntos": None     # Contorno para dibujar
}

# Inicializar RealSense
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

profile = pipeline.start(config)
color_sensor = profile.get_device().query_sensors()[1]
color_sensor.set_option(rs.option.enable_auto_exposure, 0)
color_sensor.set_option(rs.option.exposure, EXPOSICION_VAL)
color_sensor.set_option(rs.option.gain, GANANCIA_VAL)

align = rs.align(rs.stream.color)

def obtener_distancia_robusta(depth_frame, cx, cy):
    dist = depth_frame.get_distance(cx, cy)
    if dist > 0: return dist
    for dx in range(-3, 4, 3):
        for dy in range(-3, 4, 3):
            val = depth_frame.get_distance(cx + dx, cy + dy)
            if val > 0: return val
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
        
        # Filtro de color Rojo
        mask1 = cv2.inRange(hsv, np.array([0, 130, 50]), np.array([10, 255, 255]))
        mask2 = cv2.inRange(hsv, np.array([160, 130, 50]), np.array([180, 255, 255]))
        mask = cv2.addWeighted(mask1, 1.0, mask2, 1.0, 0)
        
        mask = cv2.dilate(mask, np.ones((5,5), np.uint8), iterations=1)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        encontrado_en_este_frame = False

        for cnt in contours:
            if cv2.contourArea(cnt) < AREA_MINIMA:
                continue
                
            epsilon = 0.02 * cv2.arcLength(cnt, True)
            approx = cv2.approxPolyDP(cnt, epsilon, True)
            
            # Validación de Octágono Rojo
            if len(approx) == 8 and cv2.isContourConvex(approx):
                x, y, w, h = cv2.boundingRect(approx)
                if 0.7 <= float(w)/h <= 1.3:
                    cx, cy = x + w // 2, y + h // 2
                    dist = obtener_distancia_robusta(depth_frame, cx, cy)
                    
                    # Actualizar Memoria
                    memoria_deteccion["box"] = (x, y, w, h)
                    memoria_deteccion["distancia"] = dist
                    memoria_deteccion["puntos"] = approx
                    memoria_deteccion["contador_frames"] = 0
                    encontrado_en_este_frame = True
                    break # Priorizar el primer octágono encontrado

        # Lógica de Persistencia: Si no se encontró nada, aumentar contador
        if not encontrado_en_este_frame:
            memoria_deteccion["contador_frames"] += 1

        # Dibujar si estamos dentro del límite de persistencia
        if memoria_deteccion["contador_frames"] < PERSISTENCIA_LIMITE:
            x, y, w, h = memoria_deteccion["box"]
            dist = memoria_deteccion["distancia"]
            puntos = memoria_deteccion["puntos"]
            
            # Color verde si es detección real, amarillo si es persistencia (memoria)
            color = (0, 255, 0) if encontrado_en_este_frame else (0, 255, 255)
            
            cv2.drawContours(img, [puntos], 0, color, 3)
            label = f"STOP: {dist:.2f}m" if dist > 0 else "STOP: CALCULANDO..."
            cv2.putText(img, label, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

        cv2.imshow("D435 - Octagonos con Persistencia", img)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    pipeline.stop()
    cv2.destroyAllWindows()
