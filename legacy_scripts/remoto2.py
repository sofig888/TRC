import pigpio
import time
import threading
import cv2
import numpy as np
import pyrealsense2 as rs
from flask import Flask, Response, request

# ==========================================
# CONFIGURACIÓN DE HARDWARE (GPIO)
# ==========================================
ESC_PIN = 18      
SERVO_PIN = 13    
LED_BLANCO = 26   
LED_ROJO = 27     
LED_GIRO_IZQ = 22 
LED_GIRO_DER = 23 

# Valores PWM
NEUTRO = 1500
MAX_AVANCE = 2000
MAX_REVERSA = 1400
PASO_VELOCIDAD = 80
IZQ = 1200
DER = 1800
PASO_DIRECCION = 150

# ==========================================
# INICIALIZACIÓN DE COMPONENTES
# ==========================================
pi = pigpio.pi()
velocidad_actual = NEUTRO
direccion_actual = NEUTRO
luces_blancas_on = False
intermitentes_emergencia = False

# Configurar Pines
for p in [LED_BLANCO, LED_ROJO, LED_GIRO_IZQ, LED_GIRO_DER]:
    pi.set_mode(p, pigpio.OUTPUT)
    pi.write(p, 0)

pi.set_servo_pulsewidth(ESC_PIN, NEUTRO)
pi.set_servo_pulsewidth(SERVO_PIN, NEUTRO)

# Configuración Cámara RealSense D435
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

try:
    pipeline.start(config)
    print(">>> Cámara RealSense D435 iniciada correctamente.")
except Exception as e:
    print(f">>> ERROR al iniciar RealSense: {e}")

app = Flask(__name__)

# ==========================================
# LÓGICA DE INTERMITENTES (HILO SEPARADO)
# ==========================================
def hilo_intermitentes():
    global direccion_actual, intermitentes_emergencia
    while True:
        if intermitentes_emergencia:
            estado = not pi.read(LED_GIRO_IZQ)
            pi.write(LED_GIRO_IZQ, estado)
            pi.write(LED_GIRO_DER, estado)
        elif direccion_actual < NEUTRO - 50:
            pi.write(LED_GIRO_IZQ, not pi.read(LED_GIRO_IZQ))
            pi.write(LED_GIRO_DER, 0)
        elif direccion_actual > NEUTRO + 50:
            pi.write(LED_GIRO_DER, not pi.read(LED_GIRO_DER))
            pi.write(LED_GIRO_IZQ, 0)
        else:
            pi.write(LED_GIRO_IZQ, 0)
            pi.write(LED_GIRO_DER, 0)
        time.sleep(0.4)

threading.Thread(target=hilo_intermitentes, daemon=True).start()

# ==========================================
# GENERADOR DE VIDEO PARA WEB
# ==========================================
def gen_frames():
    while True:
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame:
            continue

        # Convertir a imagen OpenCV
        frame = np.asanyarray(color_frame.get_data())
        
        # Codificar en JPG con compresión para fluidez
        ret, buffer = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 60])
        frame_bytes = buffer.tobytes()
        
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')

# ==========================================
# RUTAS DEL SERVIDOR WEB
# ==========================================
@app.route('/')
def index():
    return """
    <html>
        <head>
            <title>RealSense Robot Control</title>
            <meta name="viewport" content="width=device-width, initial-scale=1">
            <style>
                body { font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif; text-align: center; background: #1a1a1a; color: white; margin: 0; padding: 20px; }
                .video-container { border: 4px solid #333; border-radius: 15px; overflow: hidden; display: inline-block; background: #000; }
                .controls { margin-top: 25px; display: grid; grid-template-columns: repeat(3, 100px); justify-content: center; gap: 10px; }
                .btn { width: 100%; height: 60px; font-weight: bold; border-radius: 8px; border: none; cursor: pointer; color: white; font-size: 14px; }
                .btn:active { transform: scale(0.95); opacity: 0.8; }
                .btn-up { background: #2ecc71; grid-column: 2; }
                .btn-left { background: #3498db; grid-column: 1; }
                .btn-center { background: #9b59b6; grid-column: 2; }
                .btn-right { background: #3498db; grid-column: 3; }
                .btn-down { background: #e67e22; grid-column: 2; }
                .extra-controls { margin-top: 20px; }
                .btn-action { background: #f1c40f; color: black; width: 120px; margin: 5px; }
                .btn-stop { background: #e74c3c; width: 120px; margin: 5px; }
                h1 { color: #f1c40f; margin-bottom: 10px; }
            </style>
        </head>
        <body>
            <h1>COCHESITO TRC</h1>
            <div class="video-container">
                <img src="/video_feed" style="max-width: 100%; height: auto;">
            </div>
            
            <div class="controls">
                <button class="btn btn-up" onclick="fetch('/move?dir=w')">ADELANTE</button>
                <button class="btn btn-left" onclick="fetch('/move?dir=a')">IZQUIERDA</button>
                <button class="btn btn-center" onclick="fetch('/move?dir=c')">CENTRAR</button>
                <button class="btn btn-right" onclick="fetch('/move?dir=d')">DERECHA</button>
                <button class="btn btn-down" onclick="fetch('/move?dir=s')">ATRÁS/FRENO</button>
            </div>

            <div class="extra-controls">
                <button class="btn btn-action" onclick="fetch('/move?dir=l')">LUCES (L)</button>
                <button class="btn btn-action" onclick="fetch('/move?dir=i')">INTERM. (I)</button>
                <button class="btn btn-stop" onclick="fetch('/move?dir=stop')">STOP TOTAL</button>
            </div>
        </body>
    </html>
    """

@app.route('/video_feed')
def video_feed():
    return Response(gen_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/move')
def move():
    global velocidad_actual, direccion_actual, luces_blancas_on, intermitentes_emergencia
    cmd = request.args.get('dir')
    
    if cmd == 'w':
        velocidad_actual = min(MAX_AVANCE, velocidad_actual + PASO_VELOCIDAD)
    elif cmd == 's':
        pi.write(LED_ROJO, 1)
        velocidad_actual = max(MAX_REVERSA, velocidad_actual - PASO_VELOCIDAD)
        threading.Timer(0.5, lambda: pi.write(LED_ROJO, 0)).start()
    elif cmd == 'a':
        direccion_actual = max(IZQ, direccion_actual - PASO_DIRECCION)
    elif cmd == 'd':
        direccion_actual = min(DER, direccion_actual + PASO_DIRECCION)
    elif cmd == 'c':
        direccion_actual = NEUTRO
    elif cmd == 'l':
        luces_blancas_on = not luces_blancas_on
        pi.write(LED_BLANCO, 1 if luces_blancas_on else 0)
    elif cmd == 'i':
        intermitentes_emergencia = not intermitentes_emergencia
    elif cmd == 'stop':
        velocidad_actual = NEUTRO
        direccion_actual = NEUTRO

    pi.set_servo_pulsewidth(ESC_PIN, velocidad_actual)
    pi.set_servo_pulsewidth(SERVO_PIN, direccion_actual)
    return "OK"

if __name__ == '__main__':
    try:
        # Iniciamos el servidor en el puerto 5000
        app.run(host='0.0.0.0', port=5000, debug=False, threaded=True)
    finally:
        # Limpieza al cerrar
        print("Cerrando sistema...")
        pipeline.stop()
        pi.set_servo_pulsewidth(ESC_PIN, NEUTRO)
        pi.set_servo_pulsewidth(SERVO_PIN, NEUTRO)
        pi.stop()
