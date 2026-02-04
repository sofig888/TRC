from ultralytics import YOLO

# Carga el modelo original
model = YOLO('yolov8n.pt')

# Exporta a formato OpenVINO
model.export(format='openvino')
