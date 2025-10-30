import cv2
import numpy as np
import pyrealsense2 as rs
import time

MIN_CONTOUR_AREA = 400  # más bajo para pruebas
APPROX_VERTEX_TOLERANCE = 0.02
VERTEX_MIN = 6
VERTEX_MAX = 10
ASPECT_RATIO_TOL = 0.5

pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
profile = pipeline.start(config)

align = rs.align(rs.stream.color)
print("Iniciando cámara RealSense. Presiona 'q' para salir.")
time.sleep(1.0)

try:
    while True:
        frames = pipeline.wait_for_frames()
        aligned = align.process(frames)
        depth = aligned.get_depth_frame()
        color = aligned.get_color_frame()
        if not depth or not color:
            continue

        color_image = np.asanyarray(color.get_data())
        hsv = cv2.cvtColor(color_image, cv2.COLOR_BGR2HSV)

        # rangos ajustados para rojo brillante
        lower_red1 = np.array([0, 100, 100])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([160, 100, 100])
        upper_red2 = np.array([179, 255, 255])

        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask = cv2.bitwise_or(mask1, mask2)

        # limpieza ligera
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=1)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < MIN_CONTOUR_AREA:
                continue

            peri = cv2.arcLength(cnt, True)
            approx = cv2.approxPolyDP(cnt, APPROX_VERTEX_TOLERANCE * peri, True)
            vertices = len(approx)
            x, y, w, h = cv2.boundingRect(approx)

            print(f"Contorno: área={area:.1f}, vértices={vertices}")

            if VERTEX_MIN <= vertices <= VERTEX_MAX:
                cv2.drawContours(color_image, [approx], -1, (0, 255, 0), 2)
                cx, cy = x + w // 2, y + h // 2
                d = depth.get_distance(cx, cy)
                cv2.putText(color_image, f"ALTO {d:.2f}m", (x, y - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        cv2.imshow("Color", color_image)
        cv2.imshow("Mask", mask)
        key = cv2.waitKey(1)
        if key == ord('q'):
            break

finally:
    pipeline.stop()
    cv2.destroyAllWindows()