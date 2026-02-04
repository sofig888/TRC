import pyrealsense2 as rs
import numpy as np
import cv2

# ==========================================
# CONFIGURACIÓN DEFINIDA POR CALIBRACIÓN
# ==========================================
AZUL_BAJO = np.array([82, 62, 110])
AZUL_ALTO = np.array([135, 255, 255])

# Parámetros de cámara (Mantener consistentes con la calibración)
EXPOSICION_VAL = 150
GANANCIA_VAL = 64

# Configuración de procesamiento
KERNEL = np.ones((5,5), np.uint8)

# Inicializar RealSense
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

profile = pipeline.start(config)
color_sensor = profile.get_device().query_sensors()[1]
color_sensor.set_option(rs.option.enable_auto_exposure, 0)
color_sensor.set_option(rs.option.exposure, EXPOSICION_VAL)
color_sensor.set_option(rs.option.gain, GANANCIA_VAL)

print("Filtro Azul Optimizado - Iniciado")
print("Presiona 'q' para cerrar.")

try:
    while True:
        # 1. Captura de frames
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame:
            continue

        # Convertir a formato numpy
        img = np.asanyarray(color_frame.get_data())
        h, w, _ = img.shape
        mitad_y = h // 2

        # 2. Pre-procesamiento (Fluidez)
        # Solo desenfocamos para suavizar bordes
        img_blur = cv2.GaussianBlur(img, (5, 5), 0)
        hsv = cv2.cvtColor(img_blur, cv2.COLOR_BGR2HSV)

        # 3. Aplicar Máscara con tus valores calibrados
        mask = cv2.inRange(hsv, AZUL_BAJO, AZUL_ALTO)

        # 4. Relleno de "puntos negros" y limpieza
        # MORPH_CLOSE rellena los huecos internos de la cinta
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, KERNEL, iterations=2)
        # MORPH_OPEN elimina ruiditos blancos fuera de la cinta
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, KERNEL, iterations=1)

        # 5. Crear la composición de pantalla dividida
        # Convertimos la máscara de 1 canal (B/N) a 3 canales (BGR) para poder pegarla
        mask_bgr = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)

        # Creamos la imagen de salida
        # Parte superior: Original | Parte inferior: Máscara procesada
        img_final = img.copy()
        img_final[mitad_y:, :] = mask_bgr[mitad_y:, :]

        # Dibujar línea divisoria estética
        cv2.line(img_final, (0, mitad_y), (w, mitad_y), (0, 255, 0), 2)
        cv2.putText(img_final, "VISTA NORMAL", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.putText(img_final, "FILTRO NAVEGACION (AZUL)", (10, mitad_y + 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

        # 6. Mostrar resultado
        cv2.imshow("Sistema de Vision - Carrito Autonomo", img_final)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    pipeline.stop()
    cv2.destroyAllWindows()
