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
IZQ = 1200
DER = 1800

# Tope de velocidad
VEL_MAX = 1650

# RAMPAS
ACCEL_STEP = 4
DECEL_STEP = 12

# ==========================================
# STOP
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

STOP_HOLD_SECONDS = 2.0
stop_hold_until = 0.0
stop_armed = True

# ==========================================
# CARRIL AZUL
# ==========================================
AZUL_LO = np.array([90, 70, 40])
AZUL_HI = np.array([140, 255, 255])

KERNEL = np.ones((5, 5), np.uint8)
AREA_MIN_SIDE = 1200
KP_STEER = 1.2

# ==========================================
# GLOBAL
# ==========================================
pi = pigpio.pi()
ejecutando = True
intermitentes_emergencia = False

velocidad_actual = NEUTRO
direccion_actual = NEUTRO

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

def ramp_to(current, target):
    if target > current:
        return min(target, current + ACCEL_STEP)
    elif target < current:
        return max(target, current - DECEL_STEP)
    return current

def velocidad_limite_por_distancia(d):
    if d <= 0:
        return VEL_MAX
    if d <= DISTANCIA_PARADA:
        return NEUTRO
    if d < DISTANCIA_SEGURA:
        frac = (d - DISTANCIA_PARADA) / (DISTANCIA_SEGURA - DISTANCIA_PARADA)
        return int(NEUTRO + frac * (VEL_MAX - NEUTRO))
    return VEL_MAX

def make_debug_panel(H, W, roi_mask_bgr, halfW, area_left, area_right, ve_2_lineas,
                     vel_target, vel_now, dir_now, stop_visible, d_stop, hold_remaining):
    """
    Crea un panel derecho de tamaño HxW:
    - Arriba: máscara ROI (escalada para caber)
    - Abajo: texto + estado + "barras" L/R
    """
    panel = np.zeros((H, W, 3), dtype=np.uint8)

    # Zona superior (mask) ~ 60% de altura
    top_h = int(H * 0.60)
    bottom_h = H - top_h

    # Redimensionar máscara a W x top_h
    mask_resized = cv2.resize(roi_mask_bgr, (W, top_h), interpolation=cv2.INTER_NEAREST)
    cv2.line(mask_resized, (W // 2, 0), (W // 2, top_h), (255, 255, 255), 2)
    panel[:top_h, :, :] = mask_resized

    # Zona inferior info
    y0 = top_h + 30
    cv2.putText(panel, f"2 lineas: {ve_2_lineas}", (10, y0),
                cv2.FONT_HERSHEY_SIMPLEX, 0.75, (255, 255, 255), 2)
    y0 += 30
    cv2.putText(panel, f"L:{area_left}  R:{area_right}", (10, y0),
                cv2.FONT_HERSHEY_SIMPLEX, 0.75, (255, 255, 255), 2)
    y0 += 30
    cv2.putText(panel, f"vel_t:{int(vel_target)}  vel:{int(vel_now)}", (10, y0),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    y0 += 30
    cv2.putText(panel, f"dir:{int(dir_now)}", (10, y0),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    y0 += 40

    if stop_visible:
        cv2.putText(panel, f"STOP d={d_stop:.2f}m", (10, y0),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
        y0 += 35

    if hold_remaining > 0:
        cv2.putText(panel, f"HOLD: {hold_remaining:.1f}s", (10, y0),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 3)

    # “Barras” L/R (visual rápido)
    # Normalizamos a 0..W-40 (clamp)
    max_bar = W - 40
    lbar = int(clamp(area_left / 5000.0, 0.0, 1.0) * max_bar)
    rbar = int(clamp(area_right / 5000.0, 0.0, 1.0) * max_bar)

    bar_y = H - 60
    cv2.rectangle(panel, (20, bar_y), (20 + lbar, bar_y + 18), (255, 255, 255), -1)
    cv2.putText(panel, "L", (5, bar_y + 15), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

    cv2.rectangle(panel, (20, bar_y + 25), (20 + rbar, bar_y + 43), (255, 255, 255), -1)
    cv2.putText(panel, "R", (5, bar_y + 40), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

    return panel

def main():
    global ejecutando, intermitentes_emergencia
    global stop_hold_until, stop_armed
    global velocidad_actual, direccion_actual

    pi.set_servo_pulsewidth(ESC_PIN, NEUTRO)
    pi.set_servo_pulsewidth(SERVO_PIN, NEUTRO)
    time.sleep(2.0)

    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

    profile = pipeline.start(config)

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
            mitad_h = H // 2
            halfW = W // 2
            now = time.time()

            # ===================== STOP ROJO =====================
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

            if not stop_visible:
                stop_armed = True

            intermitentes_emergencia = bool(stop_visible and (0 < d_stop < DISTANCIA_INTERMITENTES))

            # ===================== CARRIL AZUL (ROI) =====================
            roi_hsv = hsv[mitad_h:, :]
            mask_azul = cv2.inRange(roi_hsv, AZUL_LO, AZUL_HI)
            mask_azul = cv2.medianBlur(mask_azul, 5)
            mask_azul = cv2.morphologyEx(mask_azul, cv2.MORPH_OPEN, KERNEL, iterations=1)
            mask_azul = cv2.morphologyEx(mask_azul, cv2.MORPH_CLOSE, KERNEL, iterations=2)

            mask_left = mask_azul[:, :halfW]
            mask_right = mask_azul[:, halfW:]

            area_left = int(cv2.countNonZero(mask_left))
            area_right = int(cv2.countNonZero(mask_right))
            ve_2_lineas = (area_left > AREA_MIN_SIDE) and (area_right > AREA_MIN_SIDE)

            cx_left = cx_right = lane_center = None
            if ve_2_lineas:
                M1 = cv2.moments(mask_left)
                M2 = cv2.moments(mask_right)
                if M1["m00"] > 0 and M2["m00"] > 0:
                    cx_left = int(M1["m10"] / M1["m00"])
                    cx_right = int(M2["m10"] / M2["m00"]) + halfW
                    lane_center = int((cx_left + cx_right) / 2)
                else:
                    ve_2_lineas = False

            direccion_obj = NEUTRO
            if ve_2_lineas and lane_center is not None:
                error = lane_center - (W // 2)
                direccion_obj = int(NEUTRO - KP_STEER * error)
                direccion_obj = clamp(direccion_obj, IZQ, DER)

            # ===================== VELOCIDAD OBJ =====================
            vel_lane = VEL_MAX if ve_2_lineas else NEUTRO
            vel_limit = VEL_MAX
            if stop_visible:
                vel_limit = velocidad_limite_por_distancia(d_stop)

            vel_target = min(vel_lane, vel_limit)

            # Hold al llegar a paro total por distancia
            if stop_armed and stop_visible and (0 < d_stop <= DISTANCIA_PARADA):
                stop_hold_until = now + STOP_HOLD_SECONDS
                stop_armed = False

            hold_remaining = 0.0
            if now < stop_hold_until:
                vel_target = NEUTRO
                hold_remaining = stop_hold_until - now
                pi.write(LED_ROJO, 1)
            else:
                pi.write(LED_ROJO, 0)

            # Rampa (freno progresivo + arranque lento)
            velocidad_actual = ramp_to(velocidad_actual, int(vel_target))
            direccion_actual = int(direccion_obj)

            pi.set_servo_pulsewidth(ESC_PIN, int(velocidad_actual))
            pi.set_servo_pulsewidth(SERVO_PIN, int(direccion_actual))

            # ===================== VISUAL =====================
            img_draw = img.copy()

            # STOP overlay
            if stop_visible and memoria_stop["box"] is not None:
                x, y, w, h = memoria_stop["box"]
                c = (0, 255, 0) if hallado_stop else (0, 255, 255)
                cv2.drawContours(img_draw, [memoria_stop["puntos"]], 0, c, 3)
                cv2.putText(img_draw, f"STOP: {d_stop:.2f}m", (x, y - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, c, 2)

            # Centro imagen
            cv2.line(img_draw, (W // 2, 0), (W // 2, H), (255, 255, 0), 2)

            # Centros carril
            if cx_left is not None:
                cv2.circle(img_draw, (cx_left, mitad_h + 50), 8, (0, 255, 255), -1)
            if cx_right is not None:
                cv2.circle(img_draw, (cx_right, mitad_h + 50), 8, (0, 255, 255), -1)
            if lane_center is not None:
                cv2.circle(img_draw, (lane_center, mitad_h + 90), 10, (0, 255, 0), -1)

            cv2.putText(img_draw, f"2 lineas:{ve_2_lineas}  L:{area_left} R:{area_right}",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2)
            cv2.putText(img_draw, f"vel_t:{int(vel_target)} vel:{int(velocidad_actual)} dir:{int(direccion_actual)}",
                        (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            if hold_remaining > 0:
                cv2.putText(img_draw, f"STOP HOLD: {hold_remaining:.1f}s",
                            (10, 95), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 3)

            # Panel derecho (full height)
            roi_vis = cv2.cvtColor(mask_azul, cv2.COLOR_GRAY2BGR)
            debug_panel = make_debug_panel(
                H=H, W=W,
                roi_mask_bgr=roi_vis,
                halfW=halfW,
                area_left=area_left,
                area_right=area_right,
                ve_2_lineas=ve_2_lineas,
                vel_target=vel_target,
                vel_now=velocidad_actual,
                dir_now=direccion_actual,
                stop_visible=stop_visible,
                d_stop=d_stop,
                hold_remaining=hold_remaining
            )

            # Split vertical real: izquierda cámara completa, derecha debug completo
            vista = np.hstack([img_draw, debug_panel])

            cv2.imshow("Autonomo (Camara | Debug L/R)", vista)
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
