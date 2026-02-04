import pigpio
import time

# Configuración del pin
ESC_PIN = 18 
pi = pigpio.pi()

if not pi.connected:
    print("Error: No se pudo conectar a pigpiod. Ejecuta 'sudo pigpiod' en la terminal.")
    exit()

def ejecutar_secuencia():
    try:
        # 1. INICIO / NEUTRO (5 segundos)
        # Esto sirve para armar el ESC de Traxxas
        print("Fase 1: Neutro - Armado (5s)")
        pi.set_servo_pulsewidth(ESC_PIN, 1500)
        time.sleep(5)

        # 2. ADELANTE (10 segundos)
        print("Fase 2: Adelante (10s)")
        # 1600 es velocidad baja, 2000 es máxima. 
        # Usamos 1650 para evitar arrancones violentos.
        pi.set_servo_pulsewidth(ESC_PIN, 1650) 
        time.sleep(10)

        # 3. NEUTRO (3 segundos)
        print("Fase 3: Neutro (3s)")
        pi.set_servo_pulsewidth(ESC_PIN, 1500)
        time.sleep(3)

        # 4. ATRÁS (10 segundos)
        # IMPORTANTE: Los ESC Traxxas suelen requerir "doble toque" para reversa
        print("Fase 4: Reversa (10s)")
        pi.set_servo_pulsewidth(ESC_PIN, 1350) # Primer toque (Freno)
        time.sleep(0.2)
        pi.set_servo_pulsewidth(ESC_PIN, 1500) # Regreso breve a neutro
        time.sleep(0.2)
        pi.set_servo_pulsewidth(ESC_PIN, 1350) # Segundo toque (Reversa real)
        time.sleep(10)

        # FINALIZAR
        print("Secuencia terminada. Deteniendo motor.")
        pi.set_servo_pulsewidth(ESC_PIN, 1500)
        time.sleep(1)

    except KeyboardInterrupt:
        print("\nInterrupción por usuario.")
    finally:
        # Limpieza: apaga la señal PWM
        pi.set_servo_pulsewidth(ESC_PIN, 0)
        pi.stop()

if __name__ == "__main__":
    ejecutar_secuencia()import pigpio
import time

pi = pigpio.pi()
PIN = 18

# Limpiar señal previa
pi.set_servo_pulsewidth(PIN, 0)
time.sleep(1)

print("Enviando Neutro... Enciende tu ESC ahora.")
pi.set_servo_pulsewidth(PIN, 1500)
time.sleep(10) # Dale tiempo de reconocer el neutro

print("Intento de giro suave...")
# En lugar de saltar a 1700, subimos gradualmente
for vel in range(1500, 1590, 5): 
    pi.set_servo_pulsewidth(PIN, vel)
    time.sleep(0.1)

time.sleep(10)
pi.set_servo_pulsewidth(PIN, 1500) # Stop
pi.stop()
import pigpio
import time

# Configuración del pin (GPIO 18 = Pin físico 12)
ESC_PIN = 18
pi = pigpio.pi()

# Verificación de conexión con el demonio pigpiod
if not pi.connected:
    print("Error: No se pudo conectar a pigpiod. Ejecuta 'sudo pigpiod' en la terminal.")
    exit()

try:
    print("--- Iniciando Secuencia del Motor ---")

    # 1. Neutro inicial (5 segundos)
    # Esto sirve para que el ESC de Traxxas se "arme" y reconozca el centro
    print("1. Neutro (5 segundos)...")
    pi.set_servo_pulsewidth(ESC_PIN, 1500)
    time.sleep(5)

    # 2. Adelante (10 segundos)
    print("2. Avanzando (10 segundos)...")
    pi.set_servo_pulsewidth(ESC_PIN, 1700) # Ajusta este valor si necesitas más o menos velocidad
    time.sleep(10)

    # 3. Neutro (3 segundos)
    print("3. Neutro (3 segundos)...")
    pi.set_servo_pulsewidth(ESC_PIN, 1500)
    time.sleep(3)

    # 4. Atrás (10 segundos)
    # NOTA: Los ESC de coche suelen necesitar un "doble toque" para entrar en reversa.
    # Primero detectan el freno (<1500), luego neutro, y luego reversa.
    # Si ves que solo frena, avísame para añadir el "doble pulso".
    print("4. Atrás (10 segundos)...")
    pi.set_servo_pulsewidth(ESC_PIN, 1300) 
    time.sleep(10)

    # Fin de la secuencia
    print("Secuencia completada. Deteniendo señal.")
    pi.set_servo_pulsewidth(ESC_PIN, 0) # Corta la señal PWM por seguridad

except KeyboardInterrupt:
    print("\nDetenido por el usuario.")
finally:
    # Aseguramos que el motor se detenga pase lo que pase
    pi.set_servo_pulsewidth(ESC_PIN, 0)
    pi.stop()
