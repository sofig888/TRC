import pigpio
import time
import threading
import cv2
from flask import Flask, render_template, Response, request

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
# INICIALIZACIÓN
# ==========================================
pi = pigpio.pi()
velocidad_actual = NEUTRO
direccion_actual = NEUTRO
luces_blancas_on = False
intermitentes_emergencia = False

app = Flask(__name__)
camera = cv2.VideoCapture(0) # 0 para la cámara por defecto

for p in [LED_BLANCO, LED_ROJO, LED_GIRO_IZQ, LED_GIRO_DER]:
    pi.set_mode(p, pigpio.OUTPUT)
    pi.write(p, 0)

pi.set_servo_pulsewidth(ESC_PIN, NEUTRO)
pi.set_servo_pulsewidth(SERVO_PIN, NEUTRO)

# ==========================================
# TRANSMISIÓN DE VIDEO
# ==========================================
def gen_frames():
    while True:
        success, frame = camera.read()
        if not success:
            break
        else:
            # Reducir resolución para mejorar la fluidez vía WiFi
            frame = cv2.resize(frame, (480, 320))
            ret, buffer = cv2.imencode('.jpg', frame)
            frame = buffer.tobytes()
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')

# ==========================================
# LÓGICA DE INTERMITENTES
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
# RUTAS DE FLASK (INTERFAZ WEB)
# ==========================================
@app.route('/')
def index():
    # Página principal con botones y video
    return """
    <html>
        <head>
            <title>Robot Control</title>
            <meta name="viewport" content="width=device-width, initial-scale=1">
            <style>
                body { font-family: sans-serif; text-align: center; background: #222; color: white; }
                .btn { width: 80px; height: 60px; margin: 5px; font-weight: bold; border-radius: 10px; border: none; cursor: pointer; }
                .btn-up { background: #4CAF50; }
                .btn-side { background: #2196F3; }
                .btn-stop { background: #f44336; width: 175px; }
                .btn-light { background: #ffeb3b; color: black; }
                .controls { margin-top: 20px; }
            </style>
        </head>
        <body>
            <h1>Raspberry Pi Robot Cam</h1>
            <img src="/video_feed" width="80%">
            <div class="controls">
                <button class="btn btn-up" onclick="fetch('/move?dir=w')">W (ADELANTE)</button><br>
                <button class="btn btn-side" onclick="fetch('/move?dir=a')">A (IZQ)</button>
                <button class="btn btn-stop" onclick="fetch('/move?dir=s')">S (FRENO)</button>
                <button class="btn btn-side" onclick="fetch('/move?dir=d')">D (DER)</button><br><br>
                <button class="btn btn-light" onclick="fetch('/move?dir=l')">LUCES</button>
                <button class="btn btn-light" onclick="fetch('/move?dir=i')">INTERM.</button>
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
    # Ejecuta el servidor en la IP local de la Raspberry
    app.run(host='0.0.0.0', port=5000, debug=False, threaded=True)
