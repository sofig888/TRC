import pigpio
import sys
import tty
import termios
import time
import cv2
import threading

# --- Configuración de Hardware ---
ESC_PIN = 18
SERVO_PIN = 13
NEUTRO = 1500
MAX_AVANCE = 1800
MAX_REVERSA = 1200
PASO_VELOCIDAD = 1
IZQ = 1300
DER = 1700

pi = pigpio.pi()

# Variables de estado globales
velocidad_actual = NEUTRO
direccion_actual = NEUTRO
ejecutando = True

def getch():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

# --- Hilo de la Cámara ---
def hilo_camara():
    global ejecutando
    # Index 4 según tu configuración para la RealSense D435
    cap = cv2.VideoCapture(4)
    
    if not cap.isOpened():
        print("\n[Error] No se pudo abrir la RealSense en el índice 4")
        return

    # Opcional: Ajustar resolución para mayor fluidez en la Pi 4
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    print("\n[Cámara] Transmisión iniciada.")
    while ejecutando:
        ret, frame = cap.read()
        if not ret:
            break
        
        # Añadir texto informativo al video
        cv2.putText(frame, f"V: {velocidad_actual} | D: {direccion_actual}", 
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
        cv2.imshow("RealSense D435 - FPV Mode", frame)
        
        # Necesario para que OpenCV actualice la ventana
        if cv2.waitKey(1) & 0xFF == ord('q'):
            ejecutando = False
            break

    cap.release()
    cv2.destroyAllWindows()

# --- Lógica de Control ---
def mostrar_estado():
    sys.stdout.write(f"\rVelocidad: {velocidad_actual} | Dirección: {direccion_actual}   ")
    sys.stdout.flush()

def controlar_coche():
    global velocidad_actual, direccion_actual, ejecutando
    
    if not pi.connected:
        print("Error: pigpiod no iniciado.")
        return

    # Iniciar el hilo de la cámara
    cam_thread = threading.Thread(target=hilo_camara)
    cam_thread.start()

    print("--- CONTROL TRAXXAS + REALSENSE D435 ---")
    print("W/S: Velocidad | A/D: Giro | C: Centro | Espacio: STOP | Q: Salir")
    
    # Armado del ESC
    pi.set_servo_pulsewidth(ESC_PIN, NEUTRO)
    pi.set_servo_pulsewidth(SERVO_PIN, NEUTRO)
    print("Armando ESC (3s)...")
    time.sleep(3)

    try:
        while ejecutando:
            mostrar_estado()
            char = getch().lower()

            if char == 'w':
                if velocidad_actual < MAX_AVANCE:
                    velocidad_actual += PASO_VELOCIDAD
                pi.set_servo_pulsewidth(ESC_PIN, velocidad_actual)
            
            elif char == 's':
                if velocidad_actual > MAX_REVERSA:
                    velocidad_actual -= PASO_VELOCIDAD
                pi.set_servo_pulsewidth(ESC_PIN, velocidad_actual)

            elif char == 'a':
                direccion_actual = IZQ
                pi.set_servo_pulsewidth(SERVO_PIN, direccion_actual)
            
            elif char == 'd':
                direccion_actual = DER
                pi.set_servo_pulsewidth(SERVO_PIN, direccion_actual)
            
            elif char == 'c':
                direccion_actual = NEUTRO
                pi.set_servo_pulsewidth(SERVO_PIN, direccion_actual)

            elif char == ' ':
                velocidad_actual = NEUTRO
                direccion_actual = NEUTRO
                pi.set_servo_pulsewidth(ESC_PIN, NEUTRO)
                pi.set_servo_pulsewidth(SERVO_PIN, NEUTRO)

            elif char == 'q':
                ejecutando = False
                break
            
    except KeyboardInterrupt:
        ejecutando = False
    finally:
        pi.set_servo_pulsewidth(ESC_PIN, NEUTRO)
        pi.set_servo_pulsewidth(SERVO_PIN, NEUTRO)
        time.sleep(0.5)
        pi.set_servo_pulsewidth(ESC_PIN, 0)
        pi.set_servo_pulsewidth(SERVO_PIN, 0)
        pi.stop()
        print("\nSistema cerrado correctamente.")

if __name__ == "__main__":
    controlar_coche()
