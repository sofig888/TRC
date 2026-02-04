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

# Tope de velocidad solicitado
VEL_MAX = 1650

# RAMPAS (suavidad): ajusta según tu carro
ACCEL_STEP = 4    # +us por frame (~30fps => ~120us/seg)
DECEL_STEP = 12   # -us por frame (frena más rápido que acelera)

# ==========================================
# STOP (misma idea, pero escalado a VEL_MAX)
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

# Temporizador STOP
STOP_HOLD_SECONDS = 2.0
stop_hold_until = 0.0
stop_armed = True  # se rearma cuando se pierde el stop

# ==========================================
# CARRIL AZUL (HSV)
# ==========================================
AZUL_LO = np.array([90, 70, 40])     # AJUSTABLE
AZUL_HI = np.array([140, 255, 255])

KERNEL = np.ones((5, 5), np.uint8)

# “ver 2 líneas”: umbrales por lado
AREA_MIN_SIDE = 1200   # baja si no avanza aunque vea líneas

# Control dirección
KP_STEER = 1.2         # si zigzaguea baja 0.8; si gira poco sube 1.5

# ==========================================
# GLOBAL
# ==========================================
pi = pigpio.pi()
ejecutando = True
intermitentes_emergencia = False

# Estado de velocidad suave
velocidad_actual = NEUTRO
direccion_actual = NEUTRO

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
    """Rampa suave hacia target (acelera lento, frena rápido)."""
    if target > current:
        return min(target, current + ACCEL_STEP)
    elif target < current:
        return max(target, current - DECEL_STEP)
    return current

def velocidad_limite_por_distancia(d):
    """
    Límite de velocidad por STOP:
    d<=PARADA -> NEUTRO
    PARADA<d<SEGURA -> escala lineal NEUTRO..VEL_MAX
    d>=SEGURA -> VEL_MAX
    """
    if d <= 0:
        return VEL_MAX
    if d <= DISTANCIA_PARADA:
        return NEUTRO
    if d < DISTANCIA_SEGURA:
        # Escalado a VEL_MAX (no 2000)
        frac = (d - DISTANCIA_PARADA) / (DISTANCIA_SEGURA - DISTANCIA_PARADA)
        return int(NEUTRO + frac * (VEL_MAX - NEUTRO))
    return VEL_MAX

def main():
    global ejecutando, intermitentes_emergencia
    global stop_hold_until, stop_armed
    global velocidad_actual, direccion_actual

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

    # Ajustes cámara
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
            now = time.time()

            # =====================================================
            # 1) STOP (ROJO)
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

            # Intermitentes
            intermitentes_emergencia = bool(stop_visible and (0 < d_stop < DISTANCIA_INTERMITENTES))

            # =====================================================
            # 2) CARRIL AZUL en ROI (mitad inferior)
            # =====================================================
            roi_hsv = hsv[mitad_h:, :]
            mask_azul = cv2.inRange(roi_hsv, AZUL_LO, AZUL_HI)
            mask_azul = cv2.medianBlur(mask_azul, 5)
            mask_azul = cv2.morphologyEx(mask_azul, cv2.MORPH_OPEN, KERNEL, iterations=1)
            mask_azul = cv2.morphologyEx(mask_azul, cv2.MORPH_CLOSE, KERNEL, iterations=2)

            halfW = W // 2
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

            # Dirección
            direccion_obj = NEUTRO
            if ve_2_lineas and lane_center is not None:
                error = lane_center - (W // 2)
                # Giro corregido (inverso)
                direccion_obj = int(NEUTRO - KP_STEER * error)
                direccion_obj = clamp(direccion_obj, IZQ, DER)

            # =====================================================
            # 3) VELOCIDAD OBJETIVO:
            # - Solo avanza si ve 2 líneas
            # - Freno progresivo por STOP (limitador por distancia)
            # - Timer visible 2s cuando llega a paro total
            # - Arranque lento (rampa) después del timer
            # =====================================================
            vel_lane = VEL_MAX if ve_2_lineas else NEUTRO

            # limitador por STOP (usa VEL_MAX)
            vel_limit = VEL_MAX
            if stop_visible:
                vel_limit = velocidad_limite_por_distancia(d_stop)

            vel_target = min(vel_lane, vel_limit)

            # Disparar hold SOLO cuando ya tocó paro total por distancia
            if stop_armed and stop_visible and (0 < d_stop <= DISTANCIA_PARADA):
                stop_hold_until = now + STOP_HOLD_SECONDS
                stop_armed = False

            # Aplicar hold
            if now < stop_hold_until:
                vel_target = NEUTRO
                pi.write(LED_ROJO, 1)
            else:
                pi.write(LED_ROJO, 0)

            # Rampa real (aquí está la diferencia: ya no brinca)
            velocidad_actual = ramp_to(velocidad_actual, int(vel_target))
            direccion_actual = int(direccion_obj)

            # Salidas
            pi.set_servo_pulsewidth(ESC_PIN, int(velocidad_actual))
            pi.set_servo_pulsewidth(SERVO_PIN, int(direccion_actual))

            # =====================================================
            # 4) VISUAL: SPLIT VERTICAL
            # Izq: cámara + overlays
            # Der: máscara ROI con separación izq/der
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
            cv2.line(img_draw, (W // 2, mitad_h), (W // 2, H), (255, 255, 0), 2)

            # Centros carril
            if cx_left is not None:
                cv2.circle(img_draw, (cx_left, mitad_h + 50), 8, (0, 255, 255), -1)
            if cx_right is not None:
                cv2.circle(img_draw, (cx_right, mitad_h + 50), 8, (0, 255, 255), -1)
            if lane_center is not None:
                cv2.circle(img_draw, (lane_center, mitad_h + 90), 10, (0, 255, 0), -1)

            # Texto carril + velocidades
            cv2.putText(img_draw, f"2 lineas: {ve_2_lineas}  L:{area_left} R:{area_right}",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2)
            cv2.putText(img_draw, f"vel_target:{int(vel_target)} vel_now:{int(velocidad_actual)} dir:{int(direccion_actual)}",
                        (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

            # Timer visible
            if now < stop_hold_until:
                restante = stop_hold_until - now
                cv2.putText(img_draw, f"STOP HOLD: {restante:.1f}s",
                            (10, 95), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 3)

            # Panel derecho: máscara ROI en BGR y línea vertical divisoria
            roi_vis = cv2.cvtColor(mask_azul, cv2.COLOR_GRAY2BGR)
            cv2.line(roi_vis, (halfW, 0), (halfW, roi_vis.shape[0]), (255, 255, 255), 2)
            cv2.putText(roi_vis, f"L:{area_left}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
            cv2.putText(roi_vis, f"R:{area_right}", (halfW + 10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)

            # Igualar alturas para hstack (usamos solo la mitad inferior de la cámara para que sea mismo alto)
            left_panel = img_draw[mitad_h:, :, :]         # mitad inferior cámara
            right_panel = roi_vis                          # ROI ya es mitad inferior

            # Asegurar tamaños iguales
            if left_panel.shape[0] != right_panel.shape[0]:
                right_panel = cv2.resize(right_panel, (right_panel.shape[1], left_panel.shape[0]))

            vista = np.hstack([left_panel, right_panel])

            cv2.imshow("Autonomo (Izq Camara | Der ROI Mask L/R)", vista)
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

