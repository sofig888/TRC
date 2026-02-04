import pigpio
import sys
import tty
import termios
import time

# Configuración de Pines
ESC_PIN = 18
SERVO_PIN = 13

# Constantes de Seguridad
NEUTRO = 1500
MAX_AVANCE = 1900   # No pasar de 2000
MAX_REVERSA = 1100  # No bajar de 1000
PASO_VELOCIDAD = 10 # Qué tanto aumenta la velocidad por cada pulsación

# Valores de Dirección
IZQ = 1250
DER = 1750

pi = pigpio.pi()

# Variables de estado inicial
velocidad_actual = NEUTRO
direccion_actual = NEUTRO

def getch():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

def mostrar_estado():
    # Limpia la línea y muestra los valores actuales
    sys.stdout.write(f"\rVelocidad: {velocidad_actual} | Dirección: {direccion_actual}   ")
    sys.stdout.flush()

def controlar_coche():
    global velocidad_actual, direccion_actual
    
    if not pi.connected:
        print("Error: pigpiod no iniciado.")
        return

    print("--- CONTROL AVANZADO TRAXXAS ---")
    print("W/S: +/- Velocidad | A/D: Giro | C: Centrar | Espacio: STOP | Q: Salir")
    
    # Armado inicial
    pi.set_servo_pulsewidth(ESC_PIN, NEUTRO)
    pi.set_servo_pulsewidth(SERVO_PIN, NEUTRO)
    print("Armando ESC...")
    time.sleep(3)
    print("¡LISTO!")

    try:
        while True:
            mostrar_estado()
            char = getch().lower()

            # Control de Velocidad (Incremental)
            if char == 'w':
                if velocidad_actual < MAX_AVANCE:
                    velocidad_actual += PASO_VELOCIDAD
                pi.set_servo_pulsewidth(ESC_PIN, velocidad_actual)
            
            elif char == 's':
                if velocidad_actual > MAX_REVERSA:
                    velocidad_actual -= PASO_VELOCIDAD
                pi.set_servo_pulsewidth(ESC_PIN, velocidad_actual)

            # Control de Dirección
            elif char == 'a':
                direccion_actual = IZQ
                pi.set_servo_pulsewidth(SERVO_PIN, direccion_actual)
            
            elif char == 'd':
                direccion_actual = DER
                pi.set_servo_pulsewidth(SERVO_PIN, direccion_actual)
            
            elif char == 'c': # NUEVO: Centrar dirección
                direccion_actual = NEUTRO
                pi.set_servo_pulsewidth(SERVO_PIN, direccion_actual)

            # Parada de Emergencia
            elif char == ' ':
                velocidad_actual = NEUTRO
                direccion_actual = NEUTRO
                pi.set_servo_pulsewidth(ESC_PIN, NEUTRO)
                pi.set_servo_pulsewidth(SERVO_PIN, NEUTRO)
                print("\n¡STOP!")

            elif char == 'q':
                print("\nApagando sistema...")
                break
            
    except KeyboardInterrupt:
        pass
    finally:
        pi.set_servo_pulsewidth(ESC_PIN, NEUTRO)
        pi.set_servo_pulsewidth(SERVO_PIN, NEUTRO)
        time.sleep(0.5)
        pi.set_servo_pulsewidth(ESC_PIN, 0)
        pi.set_servo_pulsewidth(SERVO_PIN, 0)
        pi.stop()

if __name__ == "__main__":
    controlar_coche()
