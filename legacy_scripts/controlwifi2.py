import asyncio 
import time
import threading
from aiohttp import web
import numpy as np
import cv2
import pigpio
import pyrealsense2 as rs

# ==========================================
# CONFIGURACIÓN DE HARDWARE (GPIO)
# ==========================================
ESC_PIN = 18
SERVO_PIN = 13

LED_BLANCO = 26      # Faros
LED_ROJO = 27        # Freno
LED_GIRO_IZQ = 22    # Intermitente Izquierdo
LED_GIRO_DER = 23    # Intermitente Derecho

# Valores PWM
NEUTRO = 1500
MAX_AVANCE = 2000
MAX_REVERSA = 1300
PASO_VELOCIDAD = 80

IZQ = 1200
DER = 1800
PASO_DIRECCION = 50

# ==========================================
# ESTADO
# ==========================================
pi = pigpio.pi()
if not pi.connected:
    raise RuntimeError("No se pudo conectar a pigpio. ¿Está corriendo pigpiod? (sudo systemctl start pigpiod)")

velocidad_actual = NEUTRO
direccion_actual = NEUTRO

luces_blancas_on = False
intermitentes_emergencia = False
ejecutando = True

# Inicializar LEDs
for p in [LED_BLANCO, LED_ROJO, LED_GIRO_IZQ, LED_GIRO_DER]:
    pi.set_mode(p, pigpio.OUTPUT)
    pi.write(p, 0)

# Arranque neutro
pi.set_servo_pulsewidth(ESC_PIN, NEUTRO)
pi.set_servo_pulsewidth(SERVO_PIN, NEUTRO)
time.sleep(2)

# ==========================================
# HILO DE INTERMITENTES
# ==========================================
def hilo_intermitentes():
    global ejecutando, direccion_actual, intermitentes_emergencia
    while ejecutando:
        if intermitentes_emergencia:
            estado = not pi.read(LED_GIRO_IZQ)
            pi.write(LED_GIRO_IZQ, estado)
            pi.write(LED_GIRO_DER, estado)
        elif direccion_actual < NEUTRO:
            pi.write(LED_GIRO_IZQ, not pi.read(LED_GIRO_IZQ))
            pi.write(LED_GIRO_DER, 0)
        elif direccion_actual > NEUTRO:
            pi.write(LED_GIRO_DER, not pi.read(LED_GIRO_DER))
            pi.write(LED_GIRO_IZQ, 0)
        else:
            pi.write(LED_GIRO_IZQ, 0)
            pi.write(LED_GIRO_DER, 0)
        time.sleep(0.4)

threading.Thread(target=hilo_intermitentes, daemon=True).start()

# ==========================================
# RealSense D435 (COLOR)
# ==========================================
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
pipeline.start(config)

# ==========================================
# Control: comandos desde la web
# ==========================================
_last_brake = 0.0

def aplicar_pwm():
    pi.set_servo_pulsewidth(ESC_PIN, velocidad_actual)
    pi.set_servo_pulsewidth(SERVO_PIN, direccion_actual)

def comando(c: str):
    """
    c: 'w','s','a','d','l','i'
    """
    global velocidad_actual, direccion_actual, luces_blancas_on, intermitentes_emergencia, _last_brake

    c = (c or "").lower().strip()
    if not c:
        return

    # Velocidad
    if c == 'w':
        if velocidad_actual < MAX_AVANCE:
            velocidad_actual += PASO_VELOCIDAD

    elif c == 's':
        # Freno (LED rojo breve)
        pi.write(LED_ROJO, 1)
        _last_brake = time.time()
        if velocidad_actual > MAX_REVERSA:
            velocidad_actual -= PASO_VELOCIDAD

    # Dirección
    elif c == 'a':
        direccion_actual = max(IZQ, direccion_actual - PASO_DIRECCION)

    elif c == 'd':
        direccion_actual = min(DER, direccion_actual + PASO_DIRECCION)

    # Luces
    elif c == 'l':
        luces_blancas_on = not luces_blancas_on
        pi.write(LED_BLANCO, 1 if luces_blancas_on else 0)

    # Intermitentes emergencia
    elif c == 'i':
        intermitentes_emergencia = not intermitentes_emergencia

    aplicar_pwm()

def brake_led_manager():
    global ejecutando, _last_brake
    while ejecutando:
        if time.time() - _last_brake > 0.7:
            pi.write(LED_ROJO, 0)
        time.sleep(0.05)

threading.Thread(target=brake_led_manager, daemon=True).start()

# ==========================================
# Web: página + endpoints
# ==========================================
async def index(request):
    # 6 botones + video
    html = """
    <html>
    <head>
      <meta name="viewport" content="width=device-width, initial-scale=1"/>
      <title>Carro - Control + Cam</title>
      <style>
        body{font-family:Arial;margin:12px}
        .grid{display:grid;grid-template-columns:repeat(3,1fr);gap:10px;max-width:420px}
        button{padding:14px;font-size:18px}
        img{max-width:100%;border:1px solid #ccc;margin-bottom:10px}
      </style>
    </head>
    <body>
      <h3>Carro - Video + Control</h3>

      <img src="/mjpeg" />

      <div class="grid">
        <div></div>
        <button onclick="send('w')">Adelante</button>
        <div></div>

        <button onclick="send('a')">Izquierda</button>
        <div></div>
        <button onclick="send('d')">Derecha</button>

        <div></div>
        <button onclick="send('s')">Atrás</button>
        <div></div>

        <button onclick="send('l')">Luces</button>
        <div></div>
        <button onclick="send('i')">Intermitentes</button>
      </div>

      <p id="status"></p>

      <script>
        async function send(c){
          try{
            const r = await fetch("/cmd?c=" + encodeURIComponent(c));
            const t = await r.text();
            document.getElementById("status").innerText = t;
          }catch(e){
            document.getElementById("status").innerText = "Error: " + e;
          }
        }
      </script>
    </body>
    </html>
    """
    return web.Response(text=html, content_type="text/html")

async def cmd_handler(request):
    c = request.query.get("c", "")
    comando(c)
    return web.Response(text=f"OK {c}")

async def mjpeg_handler(request):
    boundary = "frame"
    resp = web.StreamResponse(
        status=200,
        headers={
            "Content-Type": f"multipart/x-mixed-replace; boundary={boundary}",
            "Cache-Control": "no-cache",
            "Pragma": "no-cache",
        },
    )
    await resp.prepare(request)

    try:
        while True:
            frames = pipeline.wait_for_frames()
            color = frames.get_color_frame()
            if not color:
                continue

            img = np.asanyarray(color.get_data())  # BGR

            # Baja calidad/tamaño si se traba por el WiFi de la ESP32
            # img = cv2.resize(img, (480, 360))

            ok, jpg = cv2.imencode(".jpg", img, [int(cv2.IMWRITE_JPEG_QUALITY), 70])
            if not ok:
                continue

            data = jpg.tobytes()
            await resp.write(
                (f"--{boundary}\r\n"
                 "Content-Type: image/jpeg\r\n"
                 f"Content-Length: {len(data)}\r\n\r\n").encode()
            )
            await resp.write(data)
            await resp.write(b"\r\n")

            await asyncio.sleep(1/30)
    except Exception as e:
        print("MJPEG error:", e)

    return resp

# Limpieza al cerrar
async def on_cleanup(app):
    global ejecutando
    ejecutando = False
    try:
        pi.set_servo_pulsewidth(ESC_PIN, NEUTRO)
        pi.set_servo_pulsewidth(SERVO_PIN, NEUTRO)
        for p in [LED_BLANCO, LED_ROJO, LED_GIRO_IZQ, LED_GIRO_DER]:
            pi.write(p, 0)
        pi.stop()
    except:
        pass
    try:
        pipeline.stop()
    except:
        pass

app = web.Application()
app.router.add_get("/", index)
app.router.add_get("/cmd", cmd_handler)
app.router.add_get("/mjpeg", mjpeg_handler)
app.on_cleanup.append(on_cleanup)

if __name__ == "__main__":
    web.run_app(app, host="0.0.0.0", port=8080)


