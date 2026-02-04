import pyrealsense2 as rs
import numpy as np
import cv2

# ==========================================
# CONFIGURACIÓN AJUSTABLE
# ==========================================
PERSISTENCIA_LIMITE = 10
EXPOSICION_VAL = 150
GANANCIA_VAL = 64
AREA_MINIMA = 600

# Rango del color "Azul Cinta" (Ajustable según tu iluminación)
AZUL_BAJO = np.array([90, 110, 50])
AZUL_ALTO = np.array([140, 255, 255])

# Variables de estado para la persistencia
memoria_deteccion = {
    "contador_frames": PERSISTENCIA_LIMITE,
    "box": None,
    "distancia": 0,
    "puntos": None
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
        h, w_img, _ = img.shape
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        
        # --- 1. PROCESAMIENTO SEÑAL DE STOP (ROJO) ---
        mask1 = cv2.inRange(hsv, np.array([0, 130, 50]), np.array([10, 255, 255]))
        mask2 = cv2.inRange(hsv, np.array([160, 130, 50]), np.array([180, 255, 255]))
        mask_red = cv2.addWeighted(mask1, 1.0, mask2, 1.0, 0)
        mask_red = cv2.dilate(mask_red, np.ones((5,5), np.uint8), iterations=1)
        
        contours, _ = cv2.findContours(mask_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        encontrado_en_este_frame = False

        for cnt in contours:
            if cv2.contourArea(cnt) < AREA_MINIMA:
                continue
            epsilon = 0.02 * cv2.arcLength(cnt, True)
            approx = cv2.approxPolyDP(cnt, epsilon, True)
            
            if len(approx) == 8 and cv2.isContourConvex(approx):
                x, y, w, h_box = cv2.boundingRect(approx)
                if 0.7 <= float(w)/h_box <= 1.3:
                    cx, cy = x + w // 2, y + h_box // 2
                    dist = obtener_distancia_robusta(depth_frame, cx, cy)
                    memoria_deteccion.update({
                        "box": (x, y, w, h_box),
                        "distancia": dist,
                        "puntos": approx,
                        "contador_frames": 0
                    })
                    encontrado_en_este_frame = True
                    break

        if not encontrado_en_este_frame:
            memoria_deteccion["contador_frames"] += 1

        # --- 2. PROCESAMIENTO MITAD INFERIOR (FILTRO AZUL B/N) ---
        # Definimos la mitad de la pantalla
        mitad_y = h // 2
        
# 1. Crear la máscara normal
mask_azul = cv2.inRange(hsv, AZUL_BAJO, AZUL_ALTO)

# 2. Definir un kernel (el tamaño del "pincel" que limpia)
kernel = np.ones((7, 7), np.uint8)

# 3. Aplicar MORPH_CLOSE para rellenar los puntos negros internos
mask_azul = cv2.morphologyEx(mask_azul, cv2.MORPH_CLOSE, kernel)

# 4. (Opcional) Aplicar un difuminado para que los bordes sean fluidos
mask_azul = cv2.GaussianBlur(mask_azul, (5, 5), 0)        
        # Creamos una copia de la imagen final
        # La parte superior es 'img' original, la parte inferior es la máscara azul
        img_final = img.copy()
        img_final[mitad_y:, :] = mask_azul_bgr[mitad_y:, :]

        # --- 3. DIBUJAR UI SOBRE LA IMAGEN FINAL ---
        if memoria_deteccion["contador_frames"] < PERSISTENCIA_LIMITE:
            x, y, w, h_box = memoria_deteccion["box"]
            dist = memoria_deteccion["distancia"]
            puntos = memoria_deteccion["puntos"]
            color = (0, 255, 0) if encontrado_en_este_frame else (0, 255, 255)
            
            cv2.drawContours(img_final, [puntos], 0, color, 3)
            label = f"STOP: {dist:.2f}m" if dist > 0 else "STOP: CALCULANDO..."
            cv2.putText(img_final, label, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

        # Línea divisoria opcional para ver dónde empieza el filtro
        cv2.line(img_final, (0, mitad_y), (w_img, mitad_y), (255, 255, 255), 1)

        cv2.imshow("D435 - Stop + Mascara Azul Inferior", img_final)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    pipeline.stop()
    cv2.destroyAllWindows()
