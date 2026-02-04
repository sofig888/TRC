
import pigpio
import sys
import tty
import termios
import time
import threading
import select
import serial  # <-- NUEVO (UART)

# ==========================================
# CONFIGURACIÓN DE HARDWARE (GPIO)
# ==========================================
ESC_PIN = 18
SERVO_PIN = 13
LED_BLANCO = 17   # Faros (L)
LED_ROJO = 27     # Freno (S)
LED_GIRO_IZQ = 22 # Intermitente Izquierdo (A)
LED_GIRO_DER = 23 # Intermitente Derecho (D)

# Valores PWM
NEUTRO = 1500
MAX_AVANCE = 2000
MAX_REVERSA = 1400
PASO_VELOCIDAD = 10
IZQ = 1200
DER = 1800

# ==========================================
# UART (ESP32 -> Raspberry)
# ==========================================
SERIAL_PORT = "/dev/serial0"  # UART hardware de la Pi (GPIO14/15)
SERIAL_BAUD = 115200

# ==========================================
# VARIABLES DE ESTADO
# ==========================================
pi = pigpio.pi()
velocidad_actual = NEUTRO
direccion_actual = NEUTRO
luces_blancas_on = False
intermitentes_emergencia = False
ejecutando = True

# Inicializar Pines LED
for p in [LED_BLANCO, LED_ROJO, LED_GIRO_IZQ, LED_GIRO_DER]:
    pi.set_mode(p, pigpio.OUTPUT)
    pi.write(p, 0)

# ==========================================
# HILO DE INTERMITENTES
# ==========================================
def hilo_intermitentes():
    global ejecutando, direccion_actual, intermitentes_emergencia
    while ejecutando:
        # Prioridad 1: Intermitentes de emergencia (Ambas a la vez)
        if intermitentes_emergencia:
            estado_actual = not pi.read(LED_GIRO_IZQ)
            pi.write(LED_GIRO_IZQ, estado_actual)
            pi.write(LED_GIRO_DER, estado_actual)

        # Prioridad 2: Giro a la izquierda
        elif direccion_actual == IZQ:
            pi.write(LED_GIRO_IZQ, not pi.read(LED_GIRO_IZQ))
            pi.write(LED_GIRO_DER, 0)

        # Prioridad 3: Giro a la derecha
        elif direccion_actual == DER:
            pi.write(LED_GIRO_DER, not pi.read(LED_GIRO_DER))
            pi.write(LED_GIRO_IZQ, 0)

        # Apagado si no hay giro ni emergencia
        else:
            pi.write(LED_GIRO_IZQ, 0)
            pi.write(LED_GIRO_DER, 0)

        time.sleep(0.4)

# ==========================================
# UTILIDADES
# ==========================================
def is_data():
    return select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], [])

def procesar_comando(char):
    """
    Procesa 1 caracter (teclado o UART) usando EXACTAMENTE tu lógica.
    """
    global velocidad_actual, direccion_actual, ejecutando, luces_blancas_on, intermitentes_emergencia
    global last_s_press

    if char == 's':
        pi.write(LED_ROJO, 1)
        last_s_press = time.time()
        if velocidad_actual > MAX_REVERSA:
            velocidad_actual -= PASO_VELOCIDAD

    elif char == 'w':
        if velocidad_actual < MAX_AVANCE:
            velocidad_actual += PASO_VELOCIDAD

    elif char == 'a':
        direccion_actual = IZQ

    elif char == 'd':
        direccion_actual = DER

    elif char == 'c':
        direccion_actual = NEUTRO

    elif char == 'l':
        luces_blancas_on = not luces_blancas_on
        pi.write(LED_BLANCO, 1 if luces_blancas_on else 0)

    elif char == 'i':
        intermitentes_emergencia = not intermitentes_emergencia

    elif char == ' ':
        velocidad_actual = NEUTRO

    elif char == 'q':
        ejecutando = False

# ==========================================
# LÓGICA DE CONTROL
# ==========================================
def controlar_coche():
    global velocidad_actual, direccion_actual, ejecutando
    global last_s_press

    # Iniciar parpadeo en segundo plano
    threading.Thread(target=hilo_intermitentes, daemon=True).start()

    old_settings = termios.tcgetattr(sys.stdin)

    # UART: abrir puerto (no bloqueante por timeout=0)
    try:
        ser = serial.Serial(SERIAL_PORT, SERIAL_BAUD, timeout=0)
    except Exception as e:
        print(f"[ERROR] No pude abrir {SERIAL_PORT}: {e}")
        ser = None

    # Armado del ESC
    pi.set_servo_pulsewidth(ESC_PIN, NEUTRO)
    pi.set_servo_pulsewidth(SERVO_PIN, NEUTRO)
    time.sleep(2)

    last_s_press = 0

    try:
        tty.setcbreak(sys.stdin.fileno())

        while ejecutando:
            # Apagar freno rojo si ya pasó el tiempo
            if time.time() - last_s_press > 0.7:
                pi.write(LED_ROJO, 0)

            char = None

            # 1) Prioridad: UART (ESP32)
            if ser is not None and ser.in_waiting > 0:
                try:
                    char = ser.read(1).decode(errors='ignore').lower()
                except Exception:
                    char = None

            # 2) Teclado (opcional para debug)
            elif is_data():
                char = sys.stdin.read(1).lower()

            if char is not None and char != '':
                procesar_comando(char)

            # Aplicar PWM a ESC y Servo siempre
            pi.set_servo_pulsewidth(ESC_PIN, velocidad_actual)
            pi.set_servo_pulsewidth(SERVO_PIN, direccion_actual)

            time.sleep(0.01)

    finally:
        # Cerrar UART
        try:
            if ser is not None:
                ser.close()
        except Exception:
            pass

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        pi.set_servo_pulsewidth(ESC_PIN, NEUTRO)
        pi.set_servo_pulsewidth(SERVO_PIN, NEUTRO)
        for p in [LED_BLANCO, LED_ROJO, LED_GIRO_IZQ, LED_GIRO_DER]:
            pi.write(p, 0)
        pi.stop()

if __name__ == "__main__":
    controlar_coche()
