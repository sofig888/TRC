import pigpio
import sys
import tty
import termios
import time
import threading
import select

# ==========================================
# CONFIGURACIÓN DE HARDWARE (GPIO)
# ==========================================
ESC_PIN = 18      # Motor (PWM)
SERVO_PIN = 13    # Dirección (PWM)
LED_BLANCO = 17   # Faros frontales (Tecla L)
LED_ROJO = 27     # Freno (Tecla S)
LED_GIRO_IZQ = 22 # Intermitente Izquierdo (Tecla A)
LED_GIRO_DER = 23 # Intermitente Derecho (Tecla D)

# Valores PWM estándar
NEUTRO = 1500
MAX_AVANCE = 1750
MAX_REVERSA = 1300
PASO_VELOCIDAD = 15
IZQ = 1250
DER = 1750

# ==========================================
# VARIABLES DE ESTADO GLOBALES
# ==========================================
pi = pigpio.pi()
velocidad_actual = NEUTRO
direccion_actual = NEUTRO
luces_blancas_on = False
ejecutando = True

# Inicializar Pines LED como salida
for p in [LED_BLANCO, LED_ROJO, LED_GIRO_IZQ, LED_GIRO_DER]:
    pi.set_mode(p, pigpio.OUTPUT)
    pi.write(p, 0)

# ==========================================
# HILO DE INTERMITENTES (DIRECCIONALES)
# ==========================================
def hilo_intermitentes():
    """Maneja el parpadeo de amarillos sin bloquear el control"""
    global ejecutando, direccion_actual
    while ejecutando:
        if direccion_actual == IZQ:
            pi.write(LED_GIRO_IZQ, not pi.read(LED_GIRO_IZQ))
            pi.write(LED_GIRO_DER, 0)
        elif direccion_actual == DER:
            pi.write(LED_GIRO_DER, not pi.read(LED_GIRO_DER))
            pi.write(LED_GIRO_IZQ, 0)
        else:
            pi.write(LED_GIRO_IZQ, 0)
            pi.write(LED_GIRO_DER, 0)
        time.sleep(0.4) # Velocidad del parpadeo

# ==========================================
# LÓGICA DE LECTURA DE TECLADO
# ==========================================
def is_data():
    """Revisa si hay una tecla en el buffer de entrada"""
    return select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], [])

def controlar_coche():
    global velocidad_actual, direccion_actual, ejecutando, luces_blancas_on
    
    # Iniciar hilo de intermitentes
    t_inter = threading.Thread(target=hilo_intermitentes, daemon=True)
    t_inter.start()

    # Configuración de terminal para lectura raw
    old_settings = termios.tcgetattr(sys.stdin)
    
    print("--- CONTROLADOR TRAXXAS PI 4 (MODO PURO) ---")
    print("W/S: Acelerar/Frenar | A/D: Giro | C: Centrar | L: Luces | Q: Salir")
    
    # Armado del ESC (2 segundos de Neutro)
    pi.set_servo_pulsewidth(ESC_PIN, NEUTRO)
    pi.set_servo_pulsewidth(SERVO_PIN, NEUTRO)
    print("Armando ESC... espera")
    time.sleep(2)
    print("¡Listo para conducir!")

    last_s_press = 0 # Cronómetro para la luz roja

    try:
        tty.setcbreak(sys.stdin.fileno())
        while ejecutando:
            # 1. Lógica de Luz Roja (Freno reactivo)
            # Si han pasado más de 0.12s desde la última vez que se detectó 'S', apagar.
            if time.time() - last_s_press > 0.12:
                pi.write(LED_ROJO, 0)

            # 2. Procesar entrada de teclado
            if is_data():
                char = sys.stdin.read(1).lower()
                
                if char == 's':
                    pi.write(LED_ROJO, 1) # Encender luz de freno
                    last_s_press = time.time() # Resetear cronómetro
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
                elif char == ' ':
                    velocidad_actual = NEUTRO
                elif char == 'q':
                    ejecutando = False
                    break
            
            # 3. Actualizar Hardware (Servo y Motor)
            pi.set_servo_pulsewidth(ESC_PIN, velocidad_actual)
            pi.set_servo_pulsewidth(SERVO_PIN, direccion_actual)
            
            # Mostrar estado en terminal de forma limpia
            sys.stdout.write(f"\rVelocidad: {velocidad_actual} | Dir: {direccion_actual} | Luces: {'ON' if luces_headlights else 'OFF'}   ")
            sys.stdout.flush()

            time.sleep(0.01) # 100Hz de refresco
            
    finally:
        # Restaurar terminal y apagar todo
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        print("\nDeteniendo sistema...")
        pi.set_servo_pulsewidth(ESC_PIN, NEUTRO)
        pi.set_servo_pulsewidth(SERVO_PIN, NEUTRO)
        for p in [LED_BLANCO, LED_ROJO, LED_GIRO_IZQ, LED_GIRO_DER]:
            pi.write(p, 0)
        pi.stop()

if __name__ == "__main__":
    controlar_coche()
