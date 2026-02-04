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

# Velocidad en autónomo (ajusta a tu ESC)
VEL_CRUCERO = 2000
VEL_FRENO = NEUTRO
PASO_VEL = 10

# Suavizado dirección
PASO_DIR = 10

# Control de carril (P)
KP_STEER = 0.60          # baja si zigzaguea (0.35-0.60)
STEER_LIMIT = 260        # limite de giro en us (seguridad)

# ============================================================
# Pista / visión
# ============================================================
# Usar mitad inferior (reduce reflejos + mejora estabilidad)
ROI_Y_START_FRAC = 0.55

# Realce "claros" (piso gris) + contraste local
CLAHE_CLIP = 2.0
CLAHE_TILE = (8, 8)
GAMMA = 0.75             # <1 ilumina (0.65-0.85 típico)

# Segmentación azul (HSV) - AJUSTA si tu cinta es distinta
HSV_BLUE_LO = (95, 70, 40)
HSV_BLUE_HI = (140, 255, 255)

# Limpieza morfológica
MORPH_K = 5
MIN_BLOB_AREA = 90       # mínimo área del fragmento azul para considerar
MIN_BBOX_H = 12          # para eliminar ruido chiquito

# Agrupar fragmentos por X (para formar "bordes")
CLUSTER_X_DIST = 55      # px: si subes esto, agrupa más (45-70 típico)

# Criterio de carriles:
# Queremos 2 bordes del carril actual; si detectamos 3-4 bordes (ambos carriles),
# seleccionamos el par más consistente para carril izquierdo o derecho.
LANE_WIDTH_MIN = 140     # px (ajusta según FOV/altura)
LANE_WIDTH_MAX = 420     # px

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

ejecutando = True
autonomo_activo = False
carril_objetivo = "LEFT"   # "LEFT" o "RIGHT"

# Telemetría
lane_status = "IDLE"       # IDLE / TRACKING / LOST
x_lane_center = None
x_left_edge = None
x_right_edge = None
x_img_center = None
last_err = 0.0

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
    time.sleep(1.2)
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

def clamp(v, lo, hi):
    return max(lo, min(hi, v))

def set_brake():
    global velocidad_actual, direccion_actual
    velocidad_actual = NEUTRO
    direccion_actual = NEUTRO
    aplicar_pwm()
    if PIGPIO_OK:
        pi.write(LED_ROJO, 1)
        set_turn_signals(False, False)

def ramp_speed(target_vel):
    global velocidad_actual
    if velocidad_actual < target_vel:
        velocidad_actual = min(target_vel, velocidad_actual + PASO_VEL)
    elif velocidad_actual > target_vel:
        velocidad_actual = max(target_vel, velocidad_actual - PASO_VEL)

def ramp_dir(target_dir):
    global direccion_actual
    if direccion_actual < target_dir:
        direccion_actual = min(target_dir, direccion_actual + PASO_DIR)
    elif direccion_actual > target_dir:
        direccion_actual = max(target_dir, direccion_actual - PASO_DIR)

# ============================================================
# RealSense: solo COLOR (para carril). Si quieres depth luego lo sumamos.
# ============================================================
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
pipeline.start(config)

# ============================================================
# Prepro: realzar brillo en ROI inferior (CLAHE + gamma)
# ============================================================
clahe = cv2.createCLAHE(clipLimit=CLAHE_CLIP, tileGridSize=CLAHE_TILE)

def enhance_roi(img_bgr, y0):
    out = img_bgr.copy()
    roi = out[y0:, :, :]

    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    h, s, v = cv2.split(hsv)

    v = clahe.apply(v)

    v_f = v.astype(np.float32) / 255.0
    v_f = np.power(v_f, GAMMA)
    v2 = (v_f * 255.0).astype(np.uint8)

    hsv2 = cv2.merge([h, s, v2])
    roi2 = cv2.cvtColor(hsv2, cv2.COLOR_HSV2BGR)

    out[y0:, :, :] = roi2
    return out

# ============================================================
# Clustering por X
# ============================================================
def cluster_x_positions(xs, dist=CLUSTER_X_DIST):
    xs = sorted(xs)
    clusters = []
    for x in xs:
        if not clusters or abs(x - clusters[-1][-1]) > dist:
            clusters.append([x])
        else:
            clusters[-1].append(x)
    # centroides
    return [int(sum(c) / len(c)) for c in clusters]

# ============================================================
# Elegir par de bordes para carril izquierdo o derecho
# ============================================================
def choose_lane_edges(cluster_centers, w):
    """
    cluster_centers: lista de x (bordes detectados, ordenados)
    Devuelve (xL, xR) o (None, None)
    """
    if len(cluster_centers) < 2:
        return None, None

    xs = sorted(cluster_centers)

    # generar pares candidatos (xL < xR) con ancho plausible
    pairs = []
    for i in range(len(xs)):
        for j in range(i + 1, len(xs)):
            xL, xR = xs[i], xs[j]
            width = xR - xL
            if LANE_WIDTH_MIN <= width <= LANE_WIDTH_MAX:
                pairs.append((xL, xR, width))

    if not pairs:
        return None, None

    # Para carril LEFT, preferimos el par más a la izquierda.
    # Para carril RIGHT, preferimos el par más a la derecha.
    if carril_objetivo == "LEFT":
        pairs.sort(key=lambda p: (p[0], p[2]))      # menor xL primero
    else:
        pairs.sort(key=lambda p: (-p[1], p[2]))     # mayor xR primero

    # tomar el mejor candidato
    xL, xR, _ = pairs[0]
    return xL, xR

# ============================================================
# Detección de carril (fragmentos azules)
# ============================================================
def detect_lane(img_bgr):
    """
    Devuelve:
      lane_center_x, xL, xR, debug
    """
    h, w = img_bgr.shape[:2]
    y0 = int(h * ROI_Y_START_FRAC)

    enh = enhance_roi(img_bgr, y0)
    roi = enh[y0:, :, :]

    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, HSV_BLUE_LO, HSV_BLUE_HI)

    k = cv2.getStructuringElement(cv2.MORPH_RECT, (MORPH_K, MORPH_K))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k, iterations=1)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k, iterations=2)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    xs = []
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < MIN_BLOB_AREA:
            continue
        x, y, ww, hh = cv2.boundingRect(cnt)
        if hh < MIN_BBOX_H:
            continue
        cx = x + ww // 2
        xs.append(cx)

    if not xs:
        return None, None, None, {"mask": mask, "roi_y0": y0, "enh": enh, "clusters": []}

    clusters = cluster_x_positions(xs, dist=CLUSTER_X_DIST)
    clusters = sorted(clusters)

    xL, xR = choose_lane_edges(clusters, w)
    if xL is None or xR is None:
        return None, None, None, {"mask": mask, "roi_y0": y0, "enh": enh, "clusters": clusters}

    lane_center = int((xL + xR) / 2)
    return lane_center, xL, xR, {"mask": mask, "roi_y0": y0, "enh": enh, "clusters": clusters}

# ============================================================
# Control carril -> servo
# ============================================================
def lane_steering(lane_center_x, img_center_x):
    err = float(lane_center_x - img_center_x)
    steer = NEUTRO + KP_STEER * err
    steer = clamp(steer, NEUTRO - STEER_LIMIT, NEUTRO + STEER_LIMIT)
    steer = clamp(steer, IZQ, DER)
    return int(steer), err

# ============================================================
# Thread: captura + control + overlay + JPEG
# ============================================================
def hilo_captura_control():
    global latest_jpeg, ejecutando
    global lane_status, x_lane_center, x_left_edge, x_right_edge, x_img_center, last_err
    global velocidad_actual, direccion_actual

    frame_interval = 1.0 / float(STREAM_FPS)
    last_t = 0.0

    while ejecutando:
        try:
            frames = pipeline.wait_for_frames()
            cf = frames.get_color_frame()
            if not cf:
                continue

            img = np.asanyarray(cf.get_data())
            h, w = img.shape[:2]
            x_img_center = w // 2

            lane_center, xL, xR, dbg = detect_lane(img)
            x_lane_center = lane_center
            x_left_edge = xL
            x_right_edge = xR

            y0 = dbg["roi_y0"]

            # Overlay ROI line
            cv2.line(img, (0, y0), (w, y0), (200, 200, 200), 2)

            # Dibujar clusters (posibles bordes)
            for xc in dbg["clusters"]:
                cv2.line(img, (xc, y0), (xc, h), (0, 255, 255), 1)

            # Centro imagen
            cv2.line(img, (x_img_center, y0), (x_img_center, h), (255, 255, 255), 2)

            # Dibujar bordes seleccionados y centro carril
            if xL is not None and xR is not None:
                cv2.line(img, (xL, y0), (xL, h), (255, 0, 0), 3)   # borde izq
                cv2.line(img, (xR, y0), (xR, h), (0, 0, 255), 3)   # borde der
                if lane_center is not None:
                    cv2.circle(img, (lane_center, int((y0 + h) / 2)), 10, (0, 255, 0), -1)

            cv2.putText(img, f"Autonomo:{autonomo_activo}  Carril:{carril_objetivo}  Estado:{lane_status}",
                        (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)

            if autonomo_activo:
                if lane_center is None:
                    lane_status = "LOST"
                    set_brake()
                    cv2.putText(img, "LANE LOST -> BRAKE", (12, 64),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.85, (0, 0, 255), 2)
                else:
                    lane_status = "TRACKING"
                    target_dir, err = lane_steering(lane_center, x_img_center)
                    last_err = err

                    # LEDs de giro según error
                    if err < -25:
                        set_turn_signals(True, False)
                    elif err > 25:
                        set_turn_signals(False, True)
                    else:
                        set_turn_signals(False, False)

                    if PIGPIO_OK:
                        pi.write(LED_ROJO, 0)
                        pi.write(LED_BLANCO, 1)

                    ramp_speed(VEL_CRUCERO)
                    ramp_dir(target_dir)
                    aplicar_pwm()

                    cv2.putText(img, f"err(px):{err:.1f}  ESC:{int(velocidad_actual)}  SERVO:{int(direccion_actual)}",
                                (12, 64), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (255, 255, 255), 2)
            else:
                lane_status = "IDLE"
                set_brake()
                if PIGPIO_OK:
                    pi.write(LED_BLANCO, 0)

            # Debug mask en esquina
            mask = dbg["mask"]
            mask_bgr = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
            mask_small = cv2.resize(mask_bgr, (220, 120))
            img[10:10+120, w-230:w-10] = mask_small
            cv2.rectangle(img, (w-230, 10), (w-10, 10+120), (255, 255, 255), 2)
            cv2.putText(img, "mask azul", (w-225, 145),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1)

            # Encode JPEG
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
            try:
                set_brake()
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
      <title>Lane Keeping</title>
      <style>
        body{font-family:Arial;margin:12px}
        img{max-width:100%;border:1px solid #ccc}
        .row{display:flex;gap:10px;flex-wrap:wrap;margin:10px 0}
        button{padding:12px 14px;border-radius:10px;border:1px solid #bbb;font-size:16px;cursor:pointer}
        .green{background:#e7ffe7}
        .blue{background:#e7f0ff}
        .badge{display:inline-block;padding:6px 10px;border-radius:10px;background:#eee;margin:6px 0}
      </style>
    </head>
    <body>
      <h3>Autónomo: Mantener carril (gris + cinta azul discontinua)</h3>

      <div class="row">
        <button class="green" onclick="cmd('/cmd/start')">▶ Iniciar Autónomo</button>
        <button onclick="cmd('/cmd/stop')">⏹ Stop</button>
        <button class="blue" onclick="cmd('/cmd/lane_left')">⬅ Carril Izquierdo</button>
        <button class="blue" onclick="cmd('/cmd/lane_right')">➡ Carril Derecho</button>
      </div>

      <div class="badge" id="info">Cargando...</div>
      <p><img src="/mjpeg"/></p>

      <script>
        async function cmd(url){
          try{ await fetch(url); }catch(e){}
        }
        async function tick(){
          try{
            const r = await fetch('/status');
            const j = await r.json();
            document.getElementById('info').innerText =
              `Autonomo:${j.autonomo} | Carril:${j.carril} | Estado:${j.estado} | xL:${j.xL} xR:${j.xR} | LaneX:${j.lane_x} ImgX:${j.img_x} | err:${j.err} | ESC:${j.esc} SERVO:${j.servo}`;
          }catch(e){}
          setTimeout(tick, 250);
        }
        tick();
      </script>
    </body>
    </html>
    """
    return web.Response(text=html, content_type="text/html")

async def status(request):
    return web.json_response({
        "autonomo": autonomo_activo,
        "carril": carril_objetivo,
        "estado": lane_status,
        "xL": None if x_left_edge is None else int(x_left_edge),
        "xR": None if x_right_edge is None else int(x_right_edge),
        "lane_x": None if x_lane_center is None else int(x_lane_center),
        "img_x": None if x_img_center is None else int(x_img_center),
        "err": round(float(last_err), 2),
        "esc": int(velocidad_actual),
        "servo": int(direccion_actual),
    })

async def cmd_start(request):
    global autonomo_activo
    autonomo_activo = True
    if PIGPIO_OK:
        pi.write(LED_BLANCO, 1)
        pi.write(LED_ROJO, 0)
    return web.json_response({"ok": True, "autonomo": autonomo_activo})

async def cmd_stop(request):
    global autonomo_activo
    autonomo_activo = False
    set_brake()
    if PIGPIO_OK:
        pi.write(LED_BLANCO, 0)
    return web.json_response({"ok": True, "autonomo": autonomo_activo})

async def cmd_lane_left(request):
    global carril_objetivo
    carril_objetivo = "LEFT"
    return web.json_response({"ok": True, "carril": carril_objetivo})

async def cmd_lane_right(request):
    global carril_objetivo
    carril_objetivo = "RIGHT"
    return web.json_response({"ok": True, "carril": carril_objetivo})

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
app.router.add_get("/cmd/start", cmd_start)
app.router.add_get("/cmd/stop", cmd_stop)
app.router.add_get("/cmd/lane_left", cmd_lane_left)
app.router.add_get("/cmd/lane_right", cmd_lane_right)
app.router.add_get("/mjpeg", mjpeg_handler)
app.on_cleanup.append(on_cleanup)

if __name__ == "__main__":
    print("Servidor: http://0.0.0.0:8080")
    web.run_app(app, host="0.0.0.0", port=8080)


