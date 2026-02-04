import pigpio
import time
from pynput import keyboard

# Pines
ESC_PIN = 18
SERVO_PIN = 13

pi = pigpio.pi()

def test_hardware():
    print("--- INICIANDO TEST DE HARDWARE ---")
    if not pi.connected:
        print("Error: pigpiod no está corriendo.")
        return False

    # 1. ARMADO (Crucial para Traxxas)
    print("1. Armando ESC (Neutro)... Espera 5 segundos.")
    pi.set_servo_pulsewidth(ESC_PIN, 1500)
    pi.set_servo_pulsewidth(SERVO_PIN, 1500)
    time.sleep(5) # Dale tiempo al ESC para que pite o cambie de luz

    # 2. TEST DE DIRECCIÓN
    print("2. Test de Dirección (Derecha -> Izquierda)")
    pi.set_servo_pulsewidth(SERVO_PIN, 1700)
    time.sleep(0.5)
    pi.set_servo_pulsewidth(SERVO_PIN, 1300)
    time.sleep(0.5)
    pi.set_servo_pulsewidth(SERVO_PIN, 1500)

    # 3. TEST DE MOTOR
    print("3. Test de Motor (Giro leve)")
    pi.set_servo_pulsewidth(ESC_PIN, 1580) # Un poco de avance
    time.sleep(1)
    pi.set_servo_pulsewidth(ESC_PIN, 1500)
    print("--- TEST COMPLETADO. Si el coche se movió, presiona teclas ahora ---")
    return True

def on_press(key):
    try:
        if key.char == 'w':
            pi.set_servo_pulsewidth(ESC_PIN, 1620)
            print("Adelante")
        elif key.char == 's':
            pi.set_servo_pulsewidth(ESC_PIN, 1380)
            print("Atrás/Freno")
        elif key.char == 'a':
            pi.set_servo_pulsewidth(SERVO_PIN, 1200)
            print("Izquierda")
        elif key.char == 'd':
            pi.set_servo_pulsewidth(SERVO_PIN, 1800)
            print("Derecha")
    except AttributeError:
        pass

def on_release(key):
    if hasattr(key, 'char') and key.char in ['w', 's']:
        pi.set_servo_pulsewidth(ESC_PIN, 1500)
    if hasattr(key, 'char') and key.char in ['a', 'd']:
        pi.set_servo_pulsewidth(SERVO_PIN, 1500)
    if key == keyboard.Key.esc:
        return False

if test_hardware():
    with keyboard.Listener(on_press=on_press, on_release=on_release) as listener:
        listener.join()

# Limpieza final
pi.set_servo_pulsewidth(ESC_PIN, 0)
pi.set_servo_pulsewidth(SERVO_PIN, 0)
pi.stop()
