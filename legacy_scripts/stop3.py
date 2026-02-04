import pyrealsense2 as rs
import numpy as np
import cv2
from ultralytics import YOLO

def main():
    # 1. Cargar el modelo en formato OpenVINO (debes haberlo exportado antes)
    # Si aún no lo exportas, usa 'yolov8n.pt', pero será más lento.
    try:
        model = YOLO('yolov8n_openvino_model/')
    except:
        model = YOLO('yolov8n.pt')

    pipeline = rs.pipeline()
    config = rs.config()

    # BAJAMOS RESOLUCIÓN: 424x240 es suficiente para señales de STOP
    config.enable_stream(rs.stream.depth, 424, 240, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 424, 240, rs.format.bgr8, 30)

    pipeline.start(config)
    align = rs.align(rs.stream.color)

    frame_count = 0
    detecciones_recientes = [] # Para guardar la última caja y que no parpadee

    try:
        while True:
            frames = pipeline.wait_for_frames()
            aligned_frames = align.process(frames)
            depth_frame = aligned_frames.get_depth_frame()
            color_frame = aligned_frames.get_color_frame()

            if not depth_frame or not color_frame:
                continue

            color_image = np.asanyarray(color_frame.get_data())
            
            # SOLO DETECTAMOS CADA 4 CUADROS
            if frame_count % 4 == 0:
                # imgsz=240 baja la resolución interna de la IA para ir más rápido
                results = model.predict(color_image, conf=0.45, imgsz=240, verbose=False)
                detecciones_recientes = results

            frame_count += 1

            # Dibujamos las detecciones (aunque no estemos procesando este frame exacto)
            for r in detecciones_recientes:
                for box in r.boxes:
                    if int(box.cls[0]) == 11: # STOP SIGN
                        x1, y1, x2, y2 = map(int, box.xyxy[0])
                        cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
                        
                        distancia = depth_frame.get_distance(cx, cy)

                        cv2.rectangle(color_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                        cv2.putText(color_image, f"STOP: {distancia:.2f}m", (x1, y1 - 10),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            cv2.imshow('Deteccion Optimizada', color_image)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    finally:
        pipeline.stop()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
