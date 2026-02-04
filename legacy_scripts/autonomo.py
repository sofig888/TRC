import asyncio
import time
import threading

from aiohttp import web
import numpy as np
import cv2

import pigpio
import pyrealsense2 as rs

# ============================================================
# PWM / GPIO
# ============================================================
ESC_PIN = 18
SERVO_PIN = 13

LED_BLANCO = 26
LED_ROJO = 27
LED_GIRO_IZQ = 22
LED_GIRO_DER = 23

NEUTRO = 1500
IZQ = 1200
DER = 1800

# En autónomo limita velocidad (más seguro)
VEL_CRUCERO = 1620     # avanza suave
VEL_FRENO = NEUTRO     # stop
PASO_VEL = 10          # rampa suave

# Dirección: cuánto gira cuando evita
GIRO_SUAVE = 70        # microsegundos aprox (ajusta)
GIRO_FUERTE = 140      # para giro más agresivo (ajusta)

# ============================================================
# Evasión
# ============================================================
DIST_STOP = 0.30       # 30 cm: se considera bloqueado
DIST_RELEASE = 0.35    # histéresis
MIN_CLEAR_SIDE = 0.35  # mínimo “aceptable” para elegir lado (ajusta)

# ROIs (en pixeles) sobre el frame color-alineado
# Centro: obstáculo al frente
ROI_C_W, ROI_C_H = 160, 160
# Laterales: “espacio libre” izquierda/derecha
ROI_S_W, ROI_S_H = 160, 220
ROI_STEP = 6           # muestreo depth: más bajo = más CPU

# ============================================================
# Stream
# ============================================================
JPEG_QUALITY = 75
STREAM_FPS = 20

# ============================================================
# Estado
# ============================================================
pi = pigpio.pi()
PIGPIO_OK = pi.connected

velocidad_actual = NEUTRO
direccion_actual = NEUTRO

anti_choque = False
decision = "FORWARD"   # FORWARD / LEFT / RIGHT / BRAKE
dist_front = 999.0
dist_left = 999.0
dist_right = 999.0

ejecutando = True

latest_jpeg = None
latest_lock = threading.Lock()

# ============================================================
# Init GPIO
# ============================================================
if PIGPIO_OK:
    for p in [LED_BLANCO, LED_ROJO, LED_GIRO_IZQ, LED_GIRO_DER]:
        pi.set_mode(p, pigpio.OUTPUT)
        pi.write(p, 0)

    pi.set_servo_pulsewidth(ESC_PIN, NEUTRO)
    pi.set_servo_pulsewidth(SERVO_PIN, NEUTRO)
    time.sleep(1.5)
else:
    print("⚠️ pigpio NO conectado. Solo stream, sin control PWM real.")

def aplicar_pwm():
    if not PIGPIO_OK:
        return
    pi.set_servo_pulsewidth(ESC_PIN, int(velocidad_actual))
    pi.set_servo_pulsewidth(SERVO_PIN, int(direccion_actual))

def set_turn_signals(left_on: bool, right_on: bool):
    if not PIGPIO_OK:
        return
    pi.write(LED_GIRO_IZQ, 1 if left_on else 0)
    pi.write(LED_GIRO_DER, 1 if right_on else 0)

# ============================================================
# RealSense setup (Color + Depth aligned)
# ============================================================
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
pipeline.start(config)

align = rs.align(rs.stream.color)

# ============================================================
# Depth ROI distance (percentil robusto)
# ============================================================
def roi_distance(depth_frame, x0, y0, x1, y1, step=ROI_STEP):
    dists = []
    for y in range(y0, y1, step):
        for x in range(x0, x1, step):
            d = depth_frame.get_distance(x, y)
            if 0.05 < d < 5.0:
                dists.append(d)
    if not dists:
        return None
    dists.sort()
    idx = int(0.10 * len(dists))  # percentil 10
    return float(dists[idx])

def clamp(v, lo, hi):
    return max(lo, min(hi, v))

# ============================================================
# Evasión reactiva
# ============================================================
def decidir_y_controlar(df, w, h):
    global velocidad_actual, direccion_actual, anti_choque, decision
    global dist_front, dist_left, dist_right

    cx, cy = w // 2, h // 2

    # ROI Centro
    c_x0 = clamp(cx - ROI_C_W // 2, 0, w-1)
    c_y0 = clamp(cy - ROI_C_H // 2, 0, h-1)
    c_x1 = clamp(cx + ROI_C_W // 2, 0, w)
    c_y1 = clamp(cy + ROI_C_H // 2, 0, h)

    # ROI Izq (un poco más hacia la izquierda)
    l_cx = int(w * 0.25)
    l_cy = cy
    l_x0 = clamp(l_cx - ROI_S_W // 2, 0, w-1)
    l_y0 = clamp(l_cy - ROI_S_H // 2, 0, h-1)
    l_x1 = clamp(l_cx + ROI_S_W // 2, 0, w)
    l_y1 = clamp(l_cy + ROI_S_H // 2, 0, h)

    # ROI Der
    r_cx = int(w * 0.75)
    r_cy = cy
    r_x0 = clamp(r_cx - ROI_S_W // 2, 0, w-1)
    r_y0 = clamp(r_cy - ROI_S_H // 2, 0, h-1)
    r_x1 = clamp(r_cx + ROI_S_W // 2, 0, w)
    r_y1 = clamp(r_cy + ROI_S_H // 2, 0, h)

    dF = roi_distance(df, c_x0, c_y0, c_x1, c_y1)
    dL = roi_distance(df, l_x0, l_y0, l_x1, l_y1)
    dR = roi_distance(df, r_x0, r_y0, r_x1, r_y1)

    # Si no hay depth confiable -> fail-safe: frena
    if dF is None and dL is None and dR is None:
        anti_choque = True
        decision = "BRAKE"
        target_vel = VEL_FRENO
        target_dir = NEUTRO
        set_turn_signals(False, False)
        if PIGPIO_OK:
            pi.write(LED_ROJO, 1)
        return (dF, dL, dR), (c_x0,c_y0,c_x1,c_y1), (l_x0,l_y0,l_x1,l_y1), (r_x0,r_y0,r_x1,r_y1), target_vel, target_dir

    # Normaliza None -> valor pequeño para comparación (cauteloso)
    dist_front = 0.0 if dF is None else dF
    dist_left  = 0.0 if dL is None else dL
    dist_right = 0.0 if dR is None else dR

    # Si frente está libre, avanza centrado
    if dist_front >= DIST_RELEASE:
        anti_choque = False
        decision = "FORWARD"
        target_vel = VEL_CRUCERO
        target_dir = NEUTRO
        set_turn_signals(False, False)
        if PIGPIO_OK:
            pi.write(LED_ROJO, 0)

    else:
        # Frente bloqueado: elegir lado más libre
        anti_choque = True

        # Si ambos lados están muy cerca, frena
        if (dist_left < MIN_CLEAR_SIDE) and (dist_right < MIN_CLEAR_SIDE):
            decision = "BRAKE"
            target_vel = VEL_FRENO
            target_dir = NEUTRO
            set_turn_signals(False, False)
            if PIGPIO_OK:
                pi.write(LED_ROJO, 1)

        else:
            # Decide girar hacia el lado con más distancia
            if dist_left > dist_right:
                decision = "LEFT"
                target_dir = clamp(NEUTRO - GIRO_FUERTE, IZQ, DER)
                set_turn_signals(True, False)
            else:
                decision = "RIGHT"
                target_dir = clamp(NEUTRO + GIRO_FUERTE, IZQ, DER)
                set_turn_signals(False, True)

            # Mientras evade, baja un poco velocidad (más control)
            target_vel = clamp(VEL_CRUCERO - 40, NEUTRO, MAX_AVANCE := 1700)
            if PIGPIO_OK:
                pi.write(LED_ROJO, 0)

    # Rampa de velocidad suave
    if velocidad_actual < target_vel:
        velocidad_actual = min(target_vel, velocidad_actual + PASO_VEL)
    elif velocidad_actual > target_vel:
        velocidad_actual = max(target_vel, velocidad_actual - PASO_VEL)

    # Dirección suave hacia target (evita golpes)
    if direccion_actual < target_dir:
        direccion_actual = min(target_dir, direccion_actual + GIRO_SUAVE)
    elif direccion_actual > target_dir:
        direccion_actual = max(target_dir, direccion_actual - GIRO_SUAVE)

    aplicar_pwm()

    return (dF, dL, dR), (c_x0,c_y0,c_x1,c_y1), (l_x0,l_y0,l_x1,l_y1), (r_x0,r_y0,r_x1,r_y1), target_vel, target_dir

# ============================================================
# Thread: captura + control + overlay + JPEG
# ============================================================
def hilo_captura_control():
    global latest_jpeg, ejecutando

    frame_interval = 1.0 / float(STREAM_FPS)
    last_t = 0.0

    while ejecutando:
        try:
            frames = pipeline.wait_for_frames()
            frames = align.process(frames)

            cf = frames.get_color_frame()
            df = frames.get_depth_frame()
            if not cf:
                continue

            img = np.asanyarray(cf.get_data())
            h, w = img.shape[:2]

            # Decide + controla (si no hay depth, frena)
            if df:
                (dF, dL, dR), rc, rl, rr, tv, td = decidir_y_controlar(df, w, h)
            else:
                # sin depth -> frena
                dF=dL=dR=None
                if PIGPIO_OK:
                    pi.write(LED_ROJO, 1)
                tv, td = NEUTRO, NEUTRO

            # Overlay ROIs
            def rect(r, color):
                x0,y0,x1,y1 = r
                cv2.rectangle(img, (x0,y0), (x1,y1), color, 2)

            # Colores BGR
            rect(rc, (0,255,255))  # center
            rect(rl, (255,255,0))  # left
            rect(rr, (255,0,255))  # right

            # Text overlay
            def fmt(d):
                return "N/A" if d is None else f"{d:.2f}m"
            cv2.putText(img, f"DECISION: {decision}", (12, 28),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255,255,255), 2)
            cv2.putText(img, f"Front: {fmt(dF)}  Left: {fmt(dL)}  Right: {fmt(dR)}", (12, 60),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255), 2)
            cv2.putText(img, f"PWM ESC:{int(velocidad_actual)}  SERVO:{int(direccion_actual)}", (12, 92),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255), 2)
            cv2.putText(img, f"Stop@{DIST_STOP:.2f}m  Release@{DIST_RELEASE:.2f}m", (12, 124),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255,255,255), 2)

            # Encode JPEG una vez (fluido)
            ok, jpg = cv2.imencode(".jpg", img, [int(cv2.IMWRITE_JPEG_QUALITY), int(JPEG_QUALITY)])
            if ok:
                with latest_lock:
                    latest_jpeg = jpg.tobytes()

            # Mantener FPS
            now = time.time()
            if last_t:
                dt = now - last_t
                if dt < frame_interval:
                    time.sleep(frame_interval - dt)
            last_t = time.time()

        except Exception as e:
            print("Capture/Control error:", e)
            # Fail-safe: freno
            try:
                if PIGPIO_OK:
                    pi.write(LED_ROJO, 1)
                globals()["velocidad_actual"] = NEUTRO
                globals()["direccion_actual"] = NEUTRO
                aplicar_pwm()
            except:
                pass
            time.sleep(0.2)

threading.Thread(target=hilo_captura_control, daemon=True).start()

# ============================================================
# Web server
# ============================================================
async def index(request):
    html = """
    <html>
    <head>
      <meta name="viewport" content="width=device-width, initial-scale=1"/>
      <title>Autónomo Evasión</title>
      <style>
        body{font-family:Arial;margin:12px}
        img{max-width:100%;border:1px solid #ccc}
        .badge{display:inline-block;padding:6px 10px;border-radius:10px;background:#eee}
      </style>
    </head>
    <body>
      <h3>Autónomo: Evitar Obstáculos (RealSense Depth)</h3>
      <div class="badge" id="info">Cargando...</div>
      <p><img src="/mjpeg"/></p>
      <script>
        async function tick(){
          try{
            const r = await fetch('/status');
            const j = await r.json();
            document.getElementById('info').innerText =
              `Decision: ${j.decision} | F:${j.front} L:${j.left} R:${j.right} | ESC:${j.esc} SERVO:${j.servo}`;
          }catch(e){}
          setTimeout(tick, 300);
        }
        tick();
      </script>
    </body>
    </html>
    """
    return web.Response(text=html, content_type="text/html")

async def status(request):
    return web.json_response({
        "decision": decision,
        "front": None if dist_front >= 900 else round(float(dist_front), 3),
        "left": None if dist_left >= 900 else round(float(dist_left), 3),
        "right": None if dist_right >= 900 else round(float(dist_right), 3),
        "esc": int(velocidad_actual),
        "servo": int(direccion_actual),
    })

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
            with latest_lock:
                frame = latest_jpeg

            if frame is None:
                await asyncio.sleep(0.05)
                continue

            await resp.write(
                (f"--{boundary}\r\n"
                 "Content-Type: image/jpeg\r\n"
                 f"Content-Length: {len(frame)}\r\n\r\n").encode()
            )
            await resp.write(frame)
            await resp.write(b"\r\n")

            await asyncio.sleep(1.0 / float(STREAM_FPS))
    except Exception as e:
        print("MJPEG error:", e)

    return resp

async def on_cleanup(app):
    global ejecutando
    ejecutando = False
    try:
        pipeline.stop()
    except:
        pass
    try:
        if PIGPIO_OK:
            pi.set_servo_pulsewidth(ESC_PIN, NEUTRO)
            pi.set_servo_pulsewidth(SERVO_PIN, NEUTRO)
            for p in [LED_BLANCO, LED_ROJO, LED_GIRO_IZQ, LED_GIRO_DER]:
                pi.write(p, 0)
            pi.stop()
    except:
        pass

app = web.Application()
app.router.add_get("/", index)
app.router.add_get("/status", status)
app.router.add_get("/mjpeg", mjpeg_handler)
app.on_cleanup.append(on_cleanup)

if __name__ == "__main__":
    print("Servidor: http://0.0.0.0:8080")
    web.run_app(app, host="0.0.0.0", port=8080)
