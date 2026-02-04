import pyrealsense2 as rs
import numpy as np
import cv2
import pigpio
import time
import threading

# ==========================================
# PARÁMETROS DE CALIBRACIÓN (AJUSTA AQUÍ)
# ==========================================
KP = 2.5              # SENSIBILIDAD: Menor valor = giro más suave (Prueba con 0.2 a 0.5)
SENTIDO = 1           # INVERSIÓN: Cambia a 1 o -1 si el giro está al revés
ZONA_MUERTA = 1       # PÍXELES: Si el error es menor a esto, el carro va recto (evita vibración)

VEL_PRUEBA = 1550      # Velocidad constante muy baja para pruebas de carril
LIMIT_IZQ = 1200       # Límite físico de tu servo
LIMIT_DER = 1700       # Límite físico de tu servo

# Pines (Tus conexiones confirmadas)
ESC_PIN = 18; SERVO_PIN = 13; LED_ROJO = 27; LED_GIRO_IZQ = 22; LED_GIRO_DER = 23
NEUTRO = 1500

# ==========================================
# VARIABLES GLOBALES
# ==========================================
pi = pigpio.pi()
ejecutando = True
error_pista = 0
lineas_vistas = 0
intermitentes_emergencia = False

# ==========================================
# HILO DE VISIÓN
# ==========================================
def hilo_vision():
    global ejecutando, error_pista, lineas_vistas, intermitentes_emergencia
    
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    pipeline.start(config)
    
    # Valores de tu calibración exitosa
    AZUL_BAJO = np.array([82, 62, 110])
    AZUL_ALTO = np.array([135, 255, 255])

    try:
        while ejecutando:
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            if not color_frame: continue

            img = np.asanyarray(color_frame.get_data())
            h, w, _ = img.shape
            centro_camara = w // 2
            mitad_y = h // 2

            # Procesamiento ROI Inferior
            roi = img[mitad_y:, :]
            blur = cv2.GaussianBlur(roi, (5, 5), 0)
            hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv, AZUL_BAJO, AZUL_ALTO)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((5,5), np.uint8), iterations=2)

            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            centros = []
            for cnt in contours:
                if cv2.contourArea(cnt) > 500:
                    M = cv2.moments(cnt)
                    if M["m00"] != 0:
                        centros.append(int(M["m10"] / M["m00"]))

            lineas_vistas = len(centros)
            
            if lineas_vistas >= 2:
                centros.sort()
                punto_medio = (centros[0] + centros[-1]) // 2
                error_pista = punto_medio - centro_camara
                intermitentes_emergencia = False
            else:
                error_pista = 0
                intermitentes_emergencia = True

            # FEEDBACK VISUAL PARA CALIBRACIÓN
            # Dibujamos una barra que indica hacia dónde quiere girar el código
            feedback = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
            if not intermitentes_emergencia:
                cv2.line(feedback, (centro_camara, 0), (centro_camara + error_pista, 50), (0, 255, 0), 5)
            
            cv2.imshow("Calibracion Direccion", feedback)
            if cv2.waitKey(1) & 0xFF == ord('q'): break
    finally:
        pipeline.stop()

# ==========================================
# CONTROL DE MOTORES
# ==========================================
def control_autonomo():
    global ejecutando, error_pista, lineas_vistas
    
    # Armado
    pi.set_servo_pulsewidth(ESC_PIN, NEUTRO)
    pi.set_servo_pulsewidth(SERVO_PIN, NEUTRO)
    time.sleep(2)
    
    threading.Thread(target=hilo_vision, daemon=True).start()

    print("--- SISTEMA DE CALIBRACIÓN ACTIVO ---")
    print(f"Configuración: Kp={KP}, Sentido={SENTIDO}")

    try:
        while ejecutando:
            if lineas_vistas >= 2:
                # 1. APLICAR ZONA MUERTA
                error_ajustado = error_pista if abs(error_pista) > ZONA_MUERTA else 0
                
                # 2. CÁLCULO PROPORCIONAL
                ajuste = error_ajustado * KP * SENTIDO
                pwm_direccion = NEUTRO + ajuste
                
                # 3. LIMITAR RANGO (CLIPPING)
                pwm_direccion = max(LIMIT_IZQ, min(LIMIT_DER, pwm_direccion))
                
                # 4. ENVIAR SEÑAL
                pi.set_servo_pulsewidth(SERVO_PIN, pwm_direccion)
                
                # Velocidad si el camino está claro
                pi.set_servo_pulsewidth(ESC_PIN, VEL_PRUEBA if abs(error_pista) < 150 else NEUTRO)
                
            else:
                # Seguridad
                pi.set_servo_pulsewidth(ESC_PIN, NEUTRO)
                pi.set_servo_pulsewidth(SERVO_PIN, NEUTRO)
            
            time.sleep(0.02)

    except KeyboardInterrupt:
        ejecutando = False
    finally:
        pi.set_servo_pulsewidth(ESC_PIN, NEUTRO)
        pi.set_servo_pulsewidth(SERVO_PIN, NEUTRO)
        pi.stop()

if __name__ == "__main__":
    control_autonomo()
