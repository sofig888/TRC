import pyrealsense2 as rs
import numpy as np
import cv2
from ultralytics import YOLO

def main():
    # 1. Cargar modelo (lo ponemos dentro de la función)
    model = YOLO('yolov8n.pt')

    # 2. Configurar RealSense
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

    pipeline.start(config)
    align = rs.align(rs.stream.color)

    print("Cámara iniciada. Buscando señales de STOP...")

    try:
        while True:
            frames = pipeline.wait_for_frames()
            aligned_frames = align.process(frames)
            depth_frame = aligned_frames.get_depth_frame()
            color_frame = aligned_frames.get_color_frame()

            if not depth_frame or not color_frame:
                continue

            color_image = np.asanyarray(color_frame.get_data())

            # Ejecutar detección (agregamos workers=0 para evitar problemas en RPi)
            results = model.predict(color_image, conf=0.5, verbose=False)

            for r in results:
                for box in r.boxes:
                    class_id = int(box.cls[0])
                    
                    # Clase 11 = STOP SIGN
                    if class_id == 11:
                        x1, y1, x2, y2 = map(int, box.xyxy[0])
                        cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
                        
                        distancia = depth_frame.get_distance(cx, cy)

                        # Dibujar
                        cv2.rectangle(color_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                        cv2.putText(color_image, f"STOP: {distancia:.2f}m", (x1, y1 - 10),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            cv2.imshow('Deteccion RealSense', color_image)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    finally:
        pipeline.stop()
        cv2.destroyAllWindows()

# ESTA ES LA PARTE IMPORTANTE QUE EVITA QUE SE ABRAN MIL TABS
if __name__ == "__main__":
    main()
