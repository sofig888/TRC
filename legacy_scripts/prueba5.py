import pigpio
import time
from pynput import keyboard

# Configuración de Pines
ESC_PIN = 18
SERVO_PIN = 13

# Configuración de Valores PWM
NEUTRO = 1500
AVANCE = 1620   # Ajusta este valor para más o menos velocidad
RETROCESO = 1380
IZQ = 1200
DER = 1800
CENTRO = 1500

pi = pigpio.pi()

# Estados del vehículo
velocidad_actual = NEUTRO
direccion_actual = CENTRO

print("--- CONTROL RC INICIADO ---")
print("Controles: W (Adelante), S (Atrás), A (Izquierda), D (Derecha)")
print("Presiona ESC para salir.")

def actualizar_coche():
    pi.set_servo_pulsewidth(ESC_PIN, velocidad_actual)
    pi.set_servo_pulsewidth(SERVO_PIN, direccion_actual)

def on_press(key):
    global velocidad_actual, direccion_actual
    try:
        if key.char == 'w':
            velocidad_actual = AVANCE
        elif key.char == 's':
            velocidad_actual = RETROCESO
        elif key.char == 'a':
            direccion_actual = IZQ
        elif key.char == 'd':
            direccion_actual = DER
    except AttributeError:
        pass
    actualizar_coche()

def on_release(key):
    global velocidad_actual, direccion_actual
    try:
        if key.char in ['w', 's']:
            velocidad_actual = NEUTRO
        elif key.char in ['a', 'd']:
            direccion_actual = CENTRO
    except AttributeError:
        if key == keyboard.Key.esc:
            # Detener el listener
            return False
    actualizar_coche()

# Iniciar el sistema
if not pi.connected:
    print("Error: pigpiod no detectado.")
else:
    # Armado inicial (muy importante para el ESC)
    pi.set_servo_pulsewidth(ESC_PIN, NEUTRO)
    pi.set_servo_pulsewidth(SERVO_PIN, CENTRO)
    print("Armando ESC... espera 3 segundos")
    time.sleep(3)
    print("¡Listo para conducir!")

    # Escuchar el teclado de forma no bloqueante
    with keyboard.Listener(on_press=on_press, on_release=on_release) as listener:
        listener.join()

    # Limpieza al salir
    pi.set_servo_pulsewidth(ESC_PIN, 0)
    pi.set_servo_pulsewidth(SERVO_PIN, 0)
    pi.stop()
    print("Programa finalizado.")
