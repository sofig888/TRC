import pigpio
import time
from pynput import keyboard

# Configuración de Pines
ESC_PIN = 18
SERVO_PIN = 13

# Configuración de Valores (Ajusta según tu coche)
NEUTRO = 1500
AVANCE = 1630    # Velocidad de crucero segura
RETROCESO = 1370 # Para la reversa
IZQ = 1250       # Ajusta para no forzar el servo
DER = 1750       # Ajusta para no forzar el servo

pi = pigpio.pi()

# Estado actual
teclas_presionadas = set()

def actualizar_coche():
    # Lógica de Dirección
    if 'a' in teclas_presionadas:
        pi.set_servo_pulsewidth(SERVO_PIN, IZQ)
    elif 'd' in teclas_presionadas:
        pi.set_servo_pulsewidth(SERVO_PIN, DER)
    else:
        pi.set_servo_pulsewidth(SERVO_PIN, NEUTRO)

    # Lógica de Motor
    if 'w' in teclas_presionadas:
        pi.set_servo_pulsewidth(ESC_PIN, AVANCE)
    elif 's' in teclas_presionadas:
        pi.set_servo_pulsewidth(ESC_PIN, RETROCESO)
    else:
        pi.set_servo_pulsewidth(ESC_PIN, NEUTRO)

def on_press(key):
    try:
        if hasattr(key, 'char') and key.char in ['w', 's', 'a', 'd']:
            if key.char not in teclas_presionadas:
                teclas_presionadas.add(key.char)
                actualizar_coche()
    except Exception as e:
        print(f"Error: {e}")

def on_release(key):
    try:
        if hasattr(key, 'char') and key.char in ['w', 's', 'a', 'd']:
            if key.char in teclas_presionadas:
                teclas_presionadas.remove(key.char)
                actualizar_coche()
        if key == keyboard.Key.esc:
            return False
    except Exception as e:
        print(f"Error: {e}")

# --- FLUJO PRINCIPAL ---
if not pi.connected:
    print("Error: Inicia sudo pigpiod")
else:
    print("--- CONTROLADOR TRAXXAS PI 4 ---")
    print("1. ARMANDO ESC... Deja el coche quieto.")
    pi.set_servo_pulsewidth(ESC_PIN, NEUTRO)
    pi.set_servo_pulsewidth(SERVO_PIN, NEUTRO)
    time.sleep(3)
    
    print("2. ¡LISTO! Usa W, A, S, D para conducir.")
    print("   Presiona ESC para salir y detener el motor.")

    with keyboard.Listener(on_press=on_press, on_release=on_release) as listener:
        listener.join()

    # Seguridad final
    pi.set_servo_pulsewidth(ESC_PIN, NEUTRO)
    pi.set_servo_pulsewidth(SERVO_PIN, NEUTRO)
    time.sleep(0.5)
    pi.set_servo_pulsewidth(ESC_PIN, 0)
    pi.set_servo_pulsewidth(SERVO_PIN, 0)
    pi.stop()
    print("Programa cerrado.")
