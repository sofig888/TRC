import pyrealsense2 as rs
import numpy as np
import cv2
import pigpio
import time
import threading

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
VEL_CRUCERO = 1560 # Velocidad bajita para pruebas autónomas
IZQ_MAX = 1800
DER_MAX = 1200

# ==========================================
# CONFIGURACIÓN VISIÓN (Tus valores calibrados)
# ==========================================
AZUL_BAJO = np.array([82, 62, 110])
AZUL_ALTO = np.array([135, 255, 255])
AREA_MINIMA_LINEA = 500

# ==========================================
# VARIABLES GLOBALES DE ESTADO
# ==========================================
pi = pigpio.pi()
ejecutando = True
error_pista = 0
lineas_vistas = 0
intermitentes_emergencia = False

# ==========================================
# HILO DE VISIÓN (PROCESAMIENTO)
# ==========================================
def hilo_vision():
    global ejecutando, error_pista, lineas_vistas, intermitentes_emergencia
    
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    pipeline.start(config)
    
    try:
        while ejecutando:
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            if not color_frame: continue

            img = np.asanyarray(color_frame.get_data())
            h, w, _ = img.shape
            centro_camara = w // 2
            mitad_y = h // 2

            # ROI y Máscara
            roi = img[mitad_y:, :]
            blur = cv2.GaussianBlur(roi, (5, 5), 0)
            hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv, AZUL_BAJO, AZUL_ALTO)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((5,5), np.uint8), iterations=2)

            # Encontrar líneas
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            centros_encontrados = []

            for cnt in contours:
                if cv2.contourArea(cnt) > AREA_MINIMA_LINEA:
                    M = cv2.moments(cnt)
                    if M["m00"] != 0:
                        centros_encontrados.append(int(M["m10"] / M["m00"]))

            lineas_vistas = len(centros_encontrados)
            
            if lineas_vistas >= 2:
                centros_encontrados.sort()
                punto_medio = (centros_encontrados[0] + centros_encontrados[-1]) // 2
                error_pista = punto_medio - centro_camara
                intermitentes_emergencia = False
            else:
                error_pista = 0
                intermitentes_emergencia = True # Activa emergencia si pierde líneas

            # Opcional: Mostrar feedback visual (puedes quitarlo para ganar velocidad)
            cv2.imshow("Vision Autonoma", mask)
            if cv2.waitKey(1) & 0xFF == ord('q'): break
            
    finally:
        pipeline.stop()

# ==========================================
# HILO DE LUCES (Tu lógica original)
# ==========================================
def hilo_luces():
    global ejecutando, intermitentes_emergencia
    while ejecutando:
        if intermitentes_emergencia:
            pi.write(LED_GIRO_IZQ, not pi.read(LED_GIRO_IZQ))
            pi.write(LED_GIRO_DER, pi.read(LED_GIRO_IZQ))
            pi.write(LED_ROJO, 1) # Luz de freno si está en emergencia
        else:
            pi.write(LED_GIRO_IZQ, 0)
            pi.write(LED_GIRO_DER, 0)
            pi.write(LED_ROJO, 0)
        time.sleep(0.4)

# ==========================================
# BUCLE PRINCIPAL DE AUTONOMÍA
# ==========================================
def control_autonomo():
    global ejecutando, error_pista, lineas_vistas, intermitentes_emergencia
    
    # Armado del ESC
    pi.set_servo_pulsewidth(ESC_PIN, NEUTRO)
    pi.set_servo_pulsewidth(SERVO_PIN, NEUTRO)
    print("Armando ESC... Espera 2 seg")
    time.sleep(2)
    
    # Iniciar hilos
    threading.Thread(target=hilo_vision, daemon=True).start()
    threading.Thread(target=hilo_luces, daemon=True).start()

    print("¡Autonomía Iniciada! Presiona Ctrl+C para detener.")
    
    try:
        while ejecutando:
            if lineas_vistas >= 2:
                # 1. DIRECCIÓN: Control Proporcional Simple
                # Si el error es 50px a la derecha, giramos proporcionalmente
                kp = 0.8 # Sensibilidad de giro, ajusta este valor
                ajuste_direccion = NEUTRO + (error_pista * kp)
                
                # Limitar el giro a los valores físicos del servo
                direccion_final = max(IZQ_MAX, min(DER_MAX, ajuste_direccion))
                pi.set_servo_pulsewidth(SERVO_PIN, direccion_final)
                
                # 2. VELOCIDAD: Solo avanza si el error no es extremo (está centrado)
                if abs(error_pista) < 100: # Tolerancia de centrado
                    pi.set_servo_pulsewidth(ESC_PIN, VEL_CRUCERO)
                else:
                    pi.set_servo_pulsewidth(ESC_PIN, NEUTRO) # Detener si está muy chueco
            
            else:
                # Si pierde las líneas, STOP TOTAL
                pi.set_servo_pulsewidth(ESC_PIN, NEUTRO)
                pi.set_servo_pulsewidth(SERVO_PIN, NEUTRO)
            
            time.sleep(0.02) # Bucle de control a 50Hz (igual que el PWM)

    except KeyboardInterrupt:
        ejecutando = False
    finally:
        pi.set_servo_pulsewidth(ESC_PIN, NEUTRO)
        pi.set_servo_pulsewidth(SERVO_PIN, NEUTRO)
        pi.stop()

if __name__ == "__main__":
    control_autonomo()
