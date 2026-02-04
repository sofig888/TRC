import pigpio
import time
import cv2
import numpy as np
import pyrealsense2 as rs
import threading

# ==========================================
# CONFIGURACIÓN DE PINES (GPIO)
# ==========================================
ESC_PIN = 18
SERVO_PIN = 13
LED_BLANCO = 17
LED_ROJO = 27
LED_GIRO_IZQ = 22
LED_GIRO_DER = 23
BTN_AVANZAR = 5
BTN_RETROCEDER = 6
BTN_IZQ = 26
BTN_DER = 16

# Valores PWM
NEUTRO = 1500
MAX_AVANCE_BASE = 1700
MAX_REVERSA = 1300
IZQ = 1250
DER = 1750

# Variable global para detener hilos
running = True

# Instancia de pigpio
pi = pigpio.pi()

# Configuración de Salidas y Entradas
for p in [ESC_PIN, SERVO_PIN, LED_BLANCO, LED_ROJO, LED_GIRO_IZQ, LED_GIRO_DER]:
    pi.set_mode(p, pigpio.OUTPUT)
for p in [BTN_AVANZAR, BTN_RETROCEDER, BTN_IZQ, BTN_DER]:
    pi.set_mode(p, pigpio.INPUT)
    pi.set_pull_up_down(p, pigpio.PUD_UP)

pi.write(LED_BLANCO, 1)

# ==========================================
# FUNCIÓN DE CONTROL (HILO SECUNDARIO)
# ==========================================
def thread_control_fisico():
    """Maneja el movimiento y LEDs basándose en los botones físicos."""
    global running
    print("[INFO] Hilo de control físico iniciado.")
    
    try:
        while running:
            # 1. LÓGICA DE VELOCIDAD
            if pi.read(BTN_AVANZAR) == 0:
                velocidad = MAX_AVANCE_BASE
                pi.write(LED_ROJO, 0)
            elif pi.read(BTN_RETROCEDER) == 0:
                velocidad = MAX_REVERSA
                pi.write(LED_ROJO, 1)
            else:
                velocidad = NEUTRO
                pi.write(LED_ROJO, 0)

            # 2. LÓGICA DE DIRECCIÓN
            parpadeo = (int(time.time() * 2) % 2)
            if pi.read(BTN_IZQ) == 0:
                direccion = IZQ
                pi.write(LED_GIRO_IZQ, parpadeo)
                pi.write(LED_GIRO_DER, 0)
            elif pi.read(BTN_DER) == 0:
                direccion = DER
                pi.write(LED_GIRO_DER, parpadeo)
                pi.write(LED_GIRO_IZQ, 0)
            else:
                direccion = NEUTRO
                pi.write(LED_GIRO_IZQ, 0)
                pi.write(LED_GIRO_DER, 0)

            # 3. ACTUALIZAR HARDWARE
            pi.set_servo_pulsewidth(ESC_PIN, velocidad)
            pi.set_servo_pulsewidth(SERVO_PIN, direccion)
            
            time.sleep(0.01) # Alta frecuencia para respuesta rápida
            
    except Exception as e:
        print(f"[ERROR] En hilo de control: {e}")

# ==========================================
# CONFIGURACIÓN DE REALSENSE (HILO PRINCIPAL)
# ==========================================
def iniciar_realsense_index(index_deseado):
    """Busca y selecciona la cámara en el índice indicado."""
    ctx = rs.context()
    devices = ctx.query_devices()
    
    if len(devices) <= index_deseado:
        print(f"[WARN] No se encontró cámara en índice {index_deseado}. Usando primera disponible.")
        return rs.config()
    
    target_sn = devices[index_deseado].get_info(rs.camera_info.serial_number)
    print(f"[OK] Usando RealSense SN: {target_sn} (Índice {index_deseado})")
    
    cfg = rs.config()
    cfg.enable_device(target_sn)
    cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    cfg.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    return cfg

def main():
    global running
    
    # 1. Preparar Cámara
    config = iniciar_realsense_index(4) # Intentar índice 4
    pipeline = rs.pipeline()
    
    try:
        pipeline.start(config)
    except Exception as e:
        print(f"[ERROR] No se pudo iniciar el pipeline: {e}")
        return

    # 2. Iniciar Hilo de Control
    control_thread = threading.Thread(target=thread_control_fisico, daemon=True)
    control_thread.start()

    print("[INFO] Presiona 'q' en la ventana de video para salir.")

    try:
        while True:
            # Obtener frames de RealSense
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            
            if not color_frame:
                continue

            # Convertir a formato OpenCV
            img = np.asanyarray(color_frame.get_data())

            # --- Aquí puedes añadir tu lógica de detección de octágonos ---
            cv2.putText(img, "RealSense D435 - Control Activo", (20, 30), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

            # Mostrar en ventana independiente
            cv2.imshow("Vista RealSense D435", img)

            # Salida limpia
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    except KeyboardInterrupt:
        pass
    finally:
        # Limpieza total
        print("\nCerrando sistema...")
        running = False
        time.sleep(0.5)
        pipeline.stop()
        cv2.destroyAllWindows()
        
        # Detener motores
        pi.set_servo_pulsewidth(ESC_PIN, NEUTRO)
        pi.set_servo_pulsewidth(SERVO_PIN, NEUTRO)
        for p in [LED_BLANCO, LED_ROJO, LED_GIRO_IZQ, LED_GIRO_DER]:
            pi.write(p, 0)
        pi.stop()

if __name__ == "__main__":
    main()
