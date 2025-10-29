# detect_stop_realsense.py
import cv2
import numpy as np
import pyrealsense2 as rs
import time
import math
import os

# ---------- Parámetros ajustables ----------
MIN_CONTOUR_AREA = 1500         # area mínima del contorno para considerar
APPROX_VERTEX_TOLERANCE = 0.02  # epsilon factor para approxPolyDP (0.01-0.04)
VERTEX_MIN = 6                  # tolerancia mínima de vértices (aceptamos 6-10)
VERTEX_MAX = 10
ASPECT_RATIO_TOL = 0.6          # relación ancho/alto razonable para una señal cuadrada/octagonal
USE_TEMPLATE_MATCHING = True    # activar template matching si tienes la plantilla
TEMPLATE_THRESHOLD = 0.5        # correlación mínima para aceptar (0..1)
# -----------------------------------------

# carga plantilla si existe
template_path = r"C:\Users\sofia\OneDrive\VScode\python\stop_template.png"

if USE_TEMPLATE_MATCHING and os.path.exists(template_path):
    template = cv2.imread(template_path, cv2.IMREAD_COLOR)
    if template is not None:
        template_gray = cv2.cvtColor(template, cv2.COLOR_BGR2GRAY)
        tw, th = template_gray.shape[::-1]
    else:
        template = None
        USE_TEMPLATE_MATCHING = False
else:
    USE_TEMPLATE_MATCHING = False


# Configurar pipeline RealSense
pipeline = rs.pipeline()
config = rs.config()
# Pedimos color y profundidad a 640x480 para rendimiento
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

# Iniciar dispositivo
profile = pipeline.start(config)

# Para alinear frames de profundidad a color
align_to = rs.stream.color
align = rs.align(align_to)

# Opcional: filtrado de profundidad para quitar ruido (puedes ajustar)
decimation = rs.decimation_filter()
spatial = rs.spatial_filter()
temporal = rs.temporal_filter()

print("Iniciando cámara RealSense. Presiona 'q' para salir.")
time.sleep(1.0)

try:
    while True:
        frames = pipeline.wait_for_frames()
        # alinear depth->color
        aligned_frames = align.process(frames)

        depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()
        if not depth_frame or not color_frame:
            continue

        # opcional filtros
        depth_frame = decimation.process(depth_frame)
        depth_frame = spatial.process(depth_frame)
        depth_frame = temporal.process(depth_frame)

        # convertir a numpy
        color_image = np.asanyarray(color_frame.get_data())
        depth_image = np.asanyarray(depth_frame.get_data())  # en unidades internas (mm o unidades internas)
        # Pero usaremos depth_frame.get_distance(x,y) que devuelve metros.

        # Preprocesado para detección de rojo
        img_blur = cv2.GaussianBlur(color_image, (5,5), 0)
        hsv = cv2.cvtColor(img_blur, cv2.COLOR_BGR2HSV)

        # Rangos para rojo (dos rangos en HSV)
        lower_red1 = np.array([0, 80, 70])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([160, 80, 70])
        upper_red2 = np.array([179, 255, 255])

        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask = cv2.bitwise_or(mask1, mask2)

        # limpieza morfológica
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5,5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)

        # contornos
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        detected = False
        detection_text = ""
        detected_distance_m = None

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < MIN_CONTOUR_AREA:
                continue

            # aproximar polígono
            peri = cv2.arcLength(cnt, True)
            epsilon = APPROX_VERTEX_TOLERANCE * peri
            approx = cv2.approxPolyDP(cnt, epsilon, True)
            vertices = len(approx)

            # caja delimitadora
            x,y,w,h = cv2.boundingRect(approx)
            aspect = float(w)/float(h) if h != 0 else 0

            # comprobaciones básicas de forma
            if vertices >= VERTEX_MIN and vertices <= VERTEX_MAX and aspect > ASPECT_RATIO_TOL and aspect < (1.0/ASPECT_RATIO_TOL):
                # candidato probable de alto (óctagono)
                # confianza adicional: comprobar template matching si está disponible
                confirm = True
                if USE_TEMPLATE_MATCHING and template is not None:
                    # extraer ROI color y convertir a gris
                    roi = color_image[y:y+h, x:x+w]
                    if roi.size == 0:
                        confirm = False
                    else:
                        roi_gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
                        # redimensionar template si ROI menor/ mayor
                        try:
                            resized_template = cv2.resize(template_gray, (max(1, w), max(1, h)))
                            res = cv2.matchTemplate(roi_gray, resized_template, cv2.TM_CCOEFF_NORMED)
                            min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(res)
                            if max_val < TEMPLATE_THRESHOLD:
                                confirm = False
                        except Exception as e:
                            # en caso de error en template matching no confirmamos
                            confirm = False

                if confirm:
                    # calcular distancia: median de varios puntos en la caja para estabilidad
                    sample_points = []
                    # Muestreamos una rejilla 5x5 dentro del bbox
                    steps_x = max(2, w//10)
                    steps_y = max(2, h//10)
                    for xi in range(x+2, x+w-2, max(1, steps_x)):
                        for yi in range(y+2, y+h-2, max(1, steps_y)):
                            d = depth_frame.get_distance(xi, yi)
                            # descartamos ceros y distancias absurdas
                            if d and d > 0.05 and d < 30.0:
                                sample_points.append(d)
                    if len(sample_points) == 0:
                        # fallback: centro
                        cx = x + w//2
                        cy = y + h//2
                        d = depth_frame.get_distance(cx, cy)
                        if d and d>0.05 and d<30.0:
                            detected_distance_m = d
                        else:
                            detected_distance_m = None
                    else:
                        detected_distance_m = float(np.median(sample_points))

                    # marcador en la imagen
                    cv2.drawContours(color_image, [approx], -1, (0,255,0), 2)
                    cv2.rectangle(color_image, (x,y), (x+w, y+h), (0,255,0), 2)
                    label = "ALTO"
                    if detected_distance_m is not None:
                        label += f" - {detected_distance_m:.2f} m"
                    cv2.putText(color_image, label, (x, max(y-10,10)), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,255,0), 2)

                    detected = True
                    detection_text = label
                    # romper si queremos solo el primer candidato
                    break
            # si no cumple forma, seguimos buscando otros contornos

        # Mostrar ventana con máscara y video
        cv2.imshow("Color", color_image)
        cv2.imshow("Mask", mask)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break

finally:
    pipeline.stop()
    cv2.destroyAllWindows()
