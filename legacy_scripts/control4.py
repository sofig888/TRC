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
ESC_PIN = 18      
SERVO_PIN = 13    
LED_BLANCO = 17   # Faros (L)
LED_ROJO = 27     # Freno (S)
LED_GIRO_IZQ = 22 # Intermitente Izquierdo (A)
LED_GIRO_DER = 23 # Intermitente Derecho (D)

# Valores PWM
NEUTRO = 1500
MAX_AVANCE = 1650
MAX_REVERSA = 1400
PASO_VELOCIDAD = 2
IZQ = 1250
DER = 1750

# ==========================================
# VARIABLES DE ESTADO
# ==========================================
pi = pigpio.pi()
velocidad_actual = NEUTRO
direccion_actual = NEUTRO
luces_blancas_on = False
intermitentes_emergencia = False # <-- Nueva variable
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
            
        time.sleep(0.5)

# ==========================================
# LÓGICA DE CONTROL
# ==========================================
def is_data():
    return select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], [])

def controlar_coche():
    global velocidad_actual, direccion_actual, ejecutando, luces_blancas_on, intermitentes_emergencia
    
    # Iniciar parpadeo en segundo plano
    threading.Thread(target=hilo_intermitentes, daemon=True).start()

    old_settings = termios.tcgetattr(sys.stdin)
    
    # Armado del ESC
    pi.set_servo_pulsewidth(ESC_PIN, NEUTRO)
    pi.set_servo_pulsewidth(SERVO_PIN, NEUTRO)
    time.sleep(2)

    last_s_press = 0 

    try:
        tty.setcbreak(sys.stdin.fileno())
        while ejecutando:
            if time.time() - last_s_press > 0.8:
                pi.write(LED_ROJO, 0)

            if is_data():
                char = sys.stdin.read(1).lower()
                
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
                
                # NUEVA LÓGICA: Tecla I para Emergencia
                elif char == 'i':
                    intermitentes_emergencia = not intermitentes_emergencia
                
                elif char == ' ':
                    velocidad_actual = NEUTRO
                elif char == 'q':
                    ejecutando = False
            
            pi.set_servo_pulsewidth(ESC_PIN, velocidad_actual)
            pi.set_servo_pulsewidth(SERVO_PIN, direccion_actual)
            
            time.sleep(0.01)
            
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        pi.set_servo_pulsewidth(ESC_PIN, NEUTRO)
        pi.set_servo_pulsewidth(SERVO_PIN, NEUTRO)
        for p in [LED_BLANCO, LED_ROJO, LED_GIRO_IZQ, LED_GIRO_DER]:
            pi.write(p, 0)
        pi.stop()

if __name__ == "__main__":
    controlar_coche()
