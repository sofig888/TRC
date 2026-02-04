import pigpio
import sys
import tty
import termios
import time

# Configuración de Pines
ESC_PIN = 18
SERVO_PIN = 13

# Valores PWM
NEUTRO = 1500
AVANCE = 1550
RETROCESO = 1450
IZQ = 1250
DER = 1750

pi = pigpio.pi()

def getch():
    """Lee una tecla sin necesidad de presionar Enter"""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

def controlar_coche():
    if not pi.connected:
        print("Error: pigpiod no iniciado.")
        return

    print("--- CONTROL POR TERMINAL ACTIVO ---")
    print("W: Adelante | S: Atrás | A: Izquierda | D: Derecha")
    print("Espacio: Frenar/Neutro | Q: Salir")
    
    # Armado inicial
    pi.set_servo_pulsewidth(ESC_PIN, NEUTRO)
    pi.set_servo_pulsewidth(SERVO_PIN, NEUTRO)
    print("Armando ESC (espera 3s)...")
    time.sleep(3)
    print("¡LISTO!")

    try:
        while True:
            char = getch().lower()

            if char == 'w':
                print("Acción: ADELANTE  ", end="\r")
                pi.set_servo_pulsewidth(ESC_PIN, AVANCE)
            elif char == 's':
                print("Acción: ATRÁS     ", end="\r")
                pi.set_servo_pulsewidth(ESC_PIN, RETROCESO)
            elif char == 'a':
                print("Acción: IZQUIERDA ", end="\r")
                pi.set_servo_pulsewidth(SERVO_PIN, IZQ)
            elif char == 'd':
                print("Acción: DERECHA   ", end="\r")
                pi.set_servo_pulsewidth(SERVO_PIN, DER)
            elif char == ' ':
                print("Acción: NEUTRO/STOP", end="\r")
                pi.set_servo_pulsewidth(ESC_PIN, NEUTRO)
                pi.set_servo_pulsewidth(SERVO_PIN, NEUTRO)
            elif char == 'q':
                print("\nSaliendo...")
                break
            
    except KeyboardInterrupt:
        pass
    finally:
        # Seguridad: Detener todo al salir
        pi.set_servo_pulsewidth(ESC_PIN, NEUTRO)
        pi.set_servo_pulsewidth(SERVO_PIN, NEUTRO)
        time.sleep(0.5)
        pi.set_servo_pulsewidth(ESC_PIN, 0)
        pi.set_servo_pulsewidth(SERVO_PIN, 0)
        pi.stop()

if __name__ == "__main__":
    controlar_coche()
