import pyrealsense2 as rs
import numpy as np
import cv2
import pigpio
import sys
import tty
import termios
import time
import threading
import select

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
MAX_REVERSA = 1300
PASO_VELOCIDAD = 20
IZQ = 1200
DER = 1800
PASO_DIRECCION = 80

# ==========================================
# CONFIGURACIÓN VISIÓN Y PERSISTENCIA
# ==========================================
PERSISTENCIA_LIMITE = 15 
DISTANCIA_SEGURA = 1.8   
DISTANCIA_PARADA = 0.3   
DISTANCIA_INTERMITENTES = 0.6 # Umbral de 60 cm para luces

memoria_stop = {
    "contador": PERSISTENCIA_LIMITE,
    "distancia": 0.0,
    "box": None,
    "puntos": None
}

# ==========================================
# VARIABLES DE ESTADO GLOBAL
# ==========================================
pi = pigpio.pi()
velocidad_actual = NEUTRO
direccion_actual = NEUTRO
luces_blancas_on = False
intermitentes_emergencia = False # Controlado por distancia y tecla 'I'
ejecutando = True

# Inicializar Pines
for p in [LED_BLANCO, LED_ROJO, LED_GIRO_IZQ, LED_GIRO_DER]:
    pi.set_mode(p, pigpio.OUTPUT)
    pi.write(p, 0)

# ==========================================
# FUNCIONES DE APOYO
# ==========================================

def obtener_distancia_robusta(depth_frame, cx, cy):
    dist = depth_frame.get_distance(cx, cy)
    if dist > 0: return dist
    for dx in range(-3, 4, 3):
        for dy in range(-3, 4, 3):
            val = depth_frame.get_distance(cx + dx, cy + dy)
            if val > 0: return val
    return 0

def hilo_intermitentes():
    global ejecutando, direccion_actual, intermitentes_emergencia
    while ejecutando:
        # Prioridad 1: Emergencia (Activado por distancia < 60cm o manual)
        if intermitentes_emergencia:
            estado = not pi.read(LED_GIRO_IZQ)
            pi.write(LED_GIRO_IZQ, estado)
            pi.write(LED_GIRO_DER, estado)
        # Prioridad 2: Giro
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

def is_data():
    return select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], [])

# ==========================================
# HILO DE CONTROL DE USUARIO (TECLADO)
# ==========================================
def hilo_control_teclado():
    global velocidad_actual, direccion_actual, ejecutando, luces_blancas_on, intermitentes_emergencia
    
    old_settings = termios.tcgetattr(sys.stdin)
    last_s_press = 0
    
    try:
        tty.setcbreak(sys.stdin.fileno())
        while ejecutando:
            if time.time() - last_s_press > 0.7:
                pi.write(LED_ROJO, 0)

            if is_data():
                char = sys.stdin.read(1).lower()
                if char == 's':
                    pi.write(LED_ROJO, 1)
                    last_s_press = time.time()
                    if velocidad_actual > MAX_REVERSA: velocidad_actual -= PASO_VELOCIDAD
                elif char == 'w':
                    if velocidad_actual < MAX_AVANCE: velocidad_actual += PASO_VELOCIDAD
                elif char == 'a':
                    direccion_actual = max(IZQ, direccion_actual - PASO_DIRECCION)
                elif char == 'd':
                    direccion_actual = min(DER, direccion_actual + PASO_DIRECCION)
                elif char == 'l':
                    luces_blancas_on = not luces_blancas_on
                    pi.write(LED_BLANCO, 1 if luces_blancas_on else 0)
                elif char == 'i':
                    intermitentes_emergencia = not intermitentes_emergencia
                elif char == ' ':
                    velocidad_actual = NEUTRO; direccion_actual = NEUTRO
                elif char == 'q':
                    ejecutando = False
            
            # --- LÓGICA DE SEGURIDAD (VELOCIDAD POR DISTANCIA) ---
            if memoria_stop["contador"] < PERSISTENCIA_LIMITE:
                d = memoria_stop["distancia"]
                if 0 < d <= DISTANCIA_PARADA:
                    velocidad_actual = NEUTRO 
                elif DISTANCIA_PARADA < d < DISTANCIA_SEGURA:
                    max_permitida = 1500 + (d - DISTANCIA_PARADA) * ((2000 - 1500) / (DISTANCIA_SEGURA - DISTANCIA_PARADA))
                    if velocidad_actual > max_permitida:
                        velocidad_actual = int(max_permitida)

            pi.set_servo_pulsewidth(ESC_PIN, velocidad_actual)
            pi.set_servo_pulsewidth(SERVO_PIN, direccion_actual)
            time.sleep(0.01)
            
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

# ==========================================
# BUCLE PRINCIPAL (VISIÓN)
# ==========================================
def main():
    global ejecutando, intermitentes_emergencia
    
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
    threading.Thread(target=hilo_control_teclado, daemon=True).start()

    try:
        while ejecutando:
            frames = pipeline.wait_for_frames()
            aligned = align.process(frames)
            color_f = aligned.get_color_frame()
            depth_f = aligned.get_depth_frame()
            if not color_f or not depth_f: continue

            img = np.asanyarray(color_f.get_data())
            hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
            
            m1 = cv2.inRange(hsv, np.array([0, 130, 50]), np.array([10, 255, 255]))
            m2 = cv2.inRange(hsv, np.array([160, 130, 50]), np.array([180, 255, 255]))
            mask = cv2.dilate(cv2.addWeighted(m1, 1.0, m2, 1.0, 0), np.ones((5,5), np.uint8))
            
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            hallado = False

            for cnt in contours:
                if cv2.contourArea(cnt) < 600: continue
                approx = cv2.approxPolyDP(cnt, 0.02 * cv2.arcLength(cnt, True), True)
                
                if len(approx) == 8 and cv2.isContourConvex(approx):
                    x, y, w, h = cv2.boundingRect(approx)
                    if 0.7 <= float(w)/h <= 1.3:
                        dist = obtener_distancia_robusta(depth_f, x+w//2, y+h//2)
                        memoria_stop.update({"contador": 0, "distancia": dist, "box": (x,y,w,h), "puntos": approx})
                        hallado = True
                        break

            if not hallado:
                memoria_stop["contador"] += 1

            # --- LÓGICA DE INTERMITENTES AUTOMÁTICAS ---
            if memoria_stop["contador"] < PERSISTENCIA_LIMITE:
                # Si está a menos de 60cm, encender intermitentes
                if 0 < memoria_stop["distancia"] < DISTANCIA_INTERMITENTES:
                    intermitentes_emergencia = True
                else:
                    intermitentes_emergencia = False
            else:
                # Si se pierde el stop, apagar intermitentes
                intermitentes_emergencia = False

            # Dibujo
            if memoria_stop["contador"] < PERSISTENCIA_LIMITE:
                x, y, w, h = memoria_stop["box"]
                color = (0, 255, 0) if hallado else (0, 255, 255)
                cv2.drawContours(img, [memoria_stop["puntos"]], 0, color, 3)
                cv2.putText(img, f"STOP: {memoria_stop['distancia']:.2f}m", (x, y-10), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

            cv2.imshow("Control Inteligente Pi4B", img)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                ejecutando = False
                break
    finally:
        ejecutando = False
        pipeline.stop()
        for p in [LED_BLANCO, LED_ROJO, LED_GIRO_IZQ, LED_GIRO_DER]: pi.write(p, 0)
        pi.set_servo_pulsewidth(ESC_PIN, NEUTRO)
        pi.set_servo_pulsewidth(SERVO_PIN, NEUTRO)
        pi.stop()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
