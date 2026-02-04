import pyrealsense2 as rs
import numpy as np
import cv2
import pigpio
import time
import threading

# ==========================================
# GPIO / PWM
# ==========================================
ESC_PIN = 18
SERVO_PIN = 13

LED_BLANCO = 26
LED_ROJO = 27
LED_GIRO_IZQ = 22
LED_GIRO_DER = 23

NEUTRO = 1500
MAX_AVANCE = 2000
MAX_REVERSA = 1300
IZQ = 1200
DER = 1800

# Velocidad base autónoma (ajusta)
VEL_CRUCERO = 1650

# ==========================================
# STOP (tu lógica de distancia)
# ==========================================
PERSISTENCIA_LIMITE = 15
DISTANCIA_SEGURA = 1.8
DISTANCIA_PARADA = 0.3
DISTANCIA_INTERMITENTES = 0.6

memoria_stop = {
    "contador": PERSISTENCIA_LIMITE,
    "distancia": 0.0,
    "box": None,
    "puntos": None
}

# Timer STOP
STOP_HOLD_SECONDS = 2.0
stop_hold_until = 0.0
stop_armed = True   # solo permite disparar 1 vez hasta perder la señal

# ==========================================
# CARRIL AZUL (HSV)
# ==========================================
# Ajusta si hace falta por iluminación
AZUL_LO = np.array([90, 70, 40])     # H,S,V
AZUL_HI = np.array([140, 255, 255])

KERNEL = np.ones((5, 5), np.uint8)

# “ver” 2 líneas: umbrales por lado (izq y der)
AREA_MIN_SIDE = 1200   # si no avanza aunque se ven, baja a 700-1000

# Control dirección
KP_STEER = 1.2         # si zigzaguea: 0.7-1.0 / si gira poco: 1.4-1.8

# ==========================================
# GLOBAL
# ==========================================
pi = pigpio.pi()
ejecutando = True
intermitentes_emergencia = False

# Pines
for p in [LED_BLANCO, LED_ROJO, LED_GIRO_IZQ, LED_GIRO_DER]:
    pi.set_mode(p, pigpio.OUTPUT)
    pi.write(p, 0)

def clamp(x, lo, hi):
    return max(lo, min(hi, x))

def obtener_distancia_robusta(depth_frame, cx, cy):
    dist = depth_frame.get_distance(cx, cy)
    if dist > 0:
        return dist
    for dx in range(-3, 4, 3):
        for dy in range(-3, 4, 3):
            val = depth_frame.get_distance(cx + dx, cy + dy)
            if val > 0:
                return val
    return 0

def hilo_intermitentes():
    """Solo emergencia (ambas) cuando esté cerca del STOP."""
    global ejecutando, intermitentes_emergencia
    estado = False
    while ejecutando:
        if intermitentes_emergencia:
            estado = not estado
            pi.write(LED_GIRO_IZQ, estado)
            pi.write(LED_GIRO_DER, estado)
        else:
            pi.write(LED_GIRO_IZQ, 0)
            pi.write(LED_GIRO_DER, 0)
            estado = False
        time.sleep(0.35)

def main():
    global ejecutando, intermitentes_emergencia
    global stop_hold_until, stop_armed

    # Neutro inicial
    pi.set_servo_pulsewidth(ESC_PIN, NEUTRO)
    pi.set_servo_pulsewidth(SERVO_PIN, NEUTRO)
    time.sleep(2.0)

    # RealSense
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

    profile = pipeline.start(config)

    # Ajustes cámara (los tuyos)
    color_sensor = profile.get_device().query_sensors()[1]
    color_sensor.set_option(rs.option.enable_auto_exposure, 0)
    color_sensor.set_option(rs.option.exposure, 150)
    color_sensor.set_option(rs.option.gain, 64)

    align = rs.align(rs.stream.color)

    threading.Thread(target=hilo_intermitentes, daemon=True).start()

    try:
        while ejecutando:
            frames = pipeline.wait_for_frames()
            aligned = align.process(frames)
            color_f = aligned.get_color_frame()
            depth_f = aligned.get_depth_frame()
            if not color_f or not depth_f:
                continue

            img = np.asanyarray(color_f.get_data())
            hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

            H, W = img.shape[:2]
            mitad = H // 2
            now = time.time()

            # =====================================================
            # 1) STOP (ROJO) - tu detección
            # =====================================================
            m1 = cv2.inRange(hsv, np.array([0, 130, 50]), np.array([10, 255, 255]))
            m2 = cv2.inRange(hsv, np.array([160, 130, 50]), np.array([180, 255, 255]))
            mask_stop = cv2.dilate(cv2.addWeighted(m1, 1.0, m2, 1.0, 0), KERNEL)

            contours, _ = cv2.findContours(mask_stop, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            hallado_stop = False

            for cnt in contours:
                if cv2.contourArea(cnt) < 600:
                    continue
                approx = cv2.approxPolyDP(cnt, 0.02 * cv2.arcLength(cnt, True), True)
                if len(approx) == 8 and cv2.isContourConvex(approx):
                    x, y, w, h = cv2.boundingRect(approx)
                    if 0.7 <= float(w) / h <= 1.3:
                        dist = obtener_distancia_robusta(depth_f, x + w // 2, y + h // 2)
                        memoria_stop.update({"contador": 0, "distancia": dist, "box": (x, y, w, h), "puntos": approx})
                        hallado_stop = True
                        break

            if not hallado_stop:
                memoria_stop["contador"] += 1

            stop_visible = memoria_stop["contador"] < PERSISTENCIA_LIMITE
            d_stop = memoria_stop["distancia"] if stop_visible else 0.0

            # Rearmar STOP cuando se pierde la señal
            if not stop_visible:
                stop_armed = True

            # Intermitentes por cercanía
            if stop_visible and (0 < d_stop < DISTANCIA_INTERMITENTES):
                intermitentes_emergencia = True
            else:
                intermitentes_emergencia = False

            # =====================================================
            # 2) CARRIL AZUL (ROI mitad inferior) - 2 líneas
            # =====================================================
            roi_hsv = hsv[mitad:, :]
            mask_azul = cv2.inRange(roi_hsv, AZUL_LO, AZUL_HI)
            mask_azul = cv2.medianBlur(mask_azul, 5)
            mask_azul = cv2.morphologyEx(mask_azul, cv2.MORPH_OPEN, KERNEL, iterations=1)
            mask_azul = cv2.morphologyEx(mask_azul, cv2.MORPH_CLOSE, KERNEL, iterations=2)

            # separar izquierda/derecha
            halfW = W // 2
            mask_left = mask_azul[:, :halfW]
            mask_right = mask_azul[:, halfW:]

            area_left = int(cv2.countNonZero(mask_left))
            area_right = int(cv2.countNonZero(mask_right))

            ve_2_lineas = (area_left > AREA_MIN_SIDE) and (area_right > AREA_MIN_SIDE)

            lane_center = None
            cx_left = None
            cx_right = None

            if ve_2_lineas:
                M1 = cv2.moments(mask_left)
                M2 = cv2.moments(mask_right)

                if M1["m00"] > 0 and M2["m00"] > 0:
                    cx_left = int(M1["m10"] / M1["m00"])
                    cx_right = int(M2["m10"] / M2["m00"]) + halfW
                    lane_center = int((cx_left + cx_right) / 2)
                else:
                    ve_2_lineas = False

            # Dirección objetivo (mantener centrado)
            direccion_obj = NEUTRO
            if ve_2_lineas and lane_center is not None:
                error = lane_center - (W // 2)

                # OJO: CORREGIDO (antes te giraba al revés)
                direccion_obj = int(NEUTRO - KP_STEER * error)
                direccion_obj = clamp(direccion_obj, IZQ, DER)

            # =====================================================
            # 3) VELOCIDAD objetivo:
            #    - Solo avanza si ve 2 líneas
            #    - Respeta tu bajada progresiva por STOP
            #    - Timer 2s cuando ya se detuvo por completo
            # =====================================================
            # Base por carril
            velocidad_obj = VEL_CRUCERO if ve_2_lineas else NEUTRO

            # --- TU LÓGICA DE BAJAR VELOCIDAD POR STOP (sin cambiar) ---
            if stop_visible:
                d = d_stop
                if 0 < d <= DISTANCIA_PARADA:
                    velocidad_obj = NEUTRO
                elif DISTANCIA_PARADA < d < DISTANCIA_SEGURA:
                    max_permitida = 1500 + (d - DISTANCIA_PARADA) * ((2000 - 1500) / (DISTANCIA_SEGURA - DISTANCIA_PARADA))
                    if velocidad_obj > max_permitida:
                        velocidad_obj = int(max_permitida)

            # --- TIMER: cuando YA está en paro total por STOP, sostener 2s ---
            if stop_armed and stop_visible and (0 < d_stop <= DISTANCIA_PARADA):
                # disparar una sola vez hasta que se pierda el STOP
                stop_hold_until = now + STOP_HOLD_SECONDS
                stop_armed = False

            # si está en hold, forzar neutro sí o sí
            if now < stop_hold_until:
                velocidad_obj = NEUTRO
                pi.write(LED_ROJO, 1)
            else:
                pi.write(LED_ROJO, 0)

            # Aplicar PWM
            pi.set_servo_pulsewidth(ESC_PIN, int(velocidad_obj))
            pi.set_servo_pulsewidth(SERVO_PIN, int(direccion_obj))

            # =====================================================
            # 4) VISUAL (timer visible)
            # =====================================================
            img_draw = img.copy()

            # Dibujo STOP
            if stop_visible and memoria_stop["box"] is not None:
                x, y, w, h = memoria_stop["box"]
                c = (0, 255, 0) if hallado_stop else (0, 255, 255)
                cv2.drawContours(img_draw, [memoria_stop["puntos"]], 0, c, 3)
                cv2.putText(img_draw, f"STOP: {d_stop:.2f}m", (x, y - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, c, 2)

            # Centro imagen
            cv2.line(img_draw, (W // 2, mitad), (W // 2, H), (255, 255, 0), 2)

            # Centros carril
            if cx_left is not None:
                cv2.circle(img_draw, (cx_left, mitad + 40), 8, (0, 255, 255), -1)
            if cx_right is not None:
                cv2.circle(img_draw, (cx_right, mitad + 40), 8, (0, 255, 255), -1)
            if lane_center is not None:
                cv2.circle(img_draw, (lane_center, mitad + 80), 10, (0, 255, 0), -1)

            # Estado carril
            cv2.putText(img_draw, f"2 lineas: {ve_2_lineas}  L:{area_left} R:{area_right}",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2)

            # Timer visible
            if now < stop_hold_until:
                restante = stop_hold_until - now
                cv2.putText(img_draw, f"STOP HOLD: {restante:.1f}s",
                            (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.85, (0, 0, 255), 3)
            else:
                cv2.putText(img_draw, f"VEL: {int(velocidad_obj)}  DIR: {int(direccion_obj)}",
                            (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2)

            # Vista partida (arriba imagen / abajo máscara)
            top = img_draw[:mitad, :, :]
            bottom_mask_bgr = cv2.cvtColor(mask_azul, cv2.COLOR_GRAY2BGR)
            vista = np.vstack([top, bottom_mask_bgr])

            cv2.imshow("Autonomo Carril Azul + STOP", vista)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                ejecutando = False
                break

    finally:
        ejecutando = False
        pipeline.stop()
        for p in [LED_BLANCO, LED_ROJO, LED_GIRO_IZQ, LED_GIRO_DER]:
            pi.write(p, 0)
        pi.set_servo_pulsewidth(ESC_PIN, NEUTRO)
        pi.set_servo_pulsewidth(SERVO_PIN, NEUTRO)
        pi.stop()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()


