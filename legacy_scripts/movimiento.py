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
LED_BLANCO = 17   # Faros
LED_ROJO = 27     # Freno / Intermitentes Rojas
LED_GIRO_IZQ = 22 
LED_GIRO_DER = 23 

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
intermitentes_rojos_on = False
ejecutando = True

# Inicializar Pines LED
for p in [LED_BLANCO, LED_ROJO, LED_GIRO_IZQ, LED_GIRO_DER]:
    pi.set_mode(p, pigpio.OUTPUT)
    pi.write(p, 0)

# ==========================================
# HILO DE LUCES (INTERMITENTES)
# ==========================================
def hilo_luces():
    global ejecutando, direccion_actual, intermitentes_rojos_on
    while ejecutando:
        # Lógica Intermitentes de Giro (Amarillas)
        if direccion_actual == IZQ:
            pi.write(LED_GIRO_IZQ, not pi.read(LED_GIRO_IZQ))
            pi.write(LED_GIRO_DER, 0)
        elif direccion_actual == DER:
            pi.write(LED_GIRO_DER, not pi.read(LED_GIRO_DER))
            pi.write(LED_GIRO_IZQ, 0)
        else:
            pi.write(LED_GIRO_IZQ, 0)
            pi.write(LED_GIRO_DER, 0)

        # Lógica Intermitentes Rojas (Tecla I)
        if intermitentes_rojos_on:
            pi.write(LED_ROJO, not pi.read(LED_ROJO))
        
        time.sleep(0.5)

# ==========================================
# LÓGICA DE CONTROL
# ==========================================
def is_data():
    return select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], [])

def controlar_coche():
    global velocidad_actual, direccion_actual, ejecutando 
    global luces_blancas_on, intermitentes_rojos_on
    
    # Iniciar hilo de luces
    threading.Thread(target=hilo_luces, daemon=True).start()

    old_settings = termios.tcgetattr(sys.stdin)
    
    # Armado del ESC
    pi.set_servo_pulsewidth(ESC_PIN, NEUTRO)
    pi.set_servo_pulsewidth(SERVO_PIN, NEUTRO)
    time.sleep(2)

    last_s_press = 0 

    try:import pigpio
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
LED_BLANCO = 17   # Fimport pigpio
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
LED_BLANCO = 17   # Faros
LED_ROJO = 27     # Freno / Intermitentes Rojas
LED_GIRO_IZQ = 22 
LED_GIRO_DER = 23 

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
intermitentes_rojos_on = False
ejecutando = True

# Inicializar Pines LED
for p in [LED_BLANCO, LED_ROJO, LED_GIRO_IZQ, LED_GIRO_DER]:
    pi.set_mode(p, pigpio.OUTPUT)
    pi.write(p, 0)

# ==========================================
# HILO DE LUCES (INTERMITENTES)
# ==========================================
def hilo_luces():
    global ejecutando, direccion_actual, intermitentes_rojos_on
    while ejecutando:
        # Lógica Intermitentes de Giro (Amarillas)
        if direccion_actual == IZQ:
            pi.write(LED_GIRO_IZQ, not pi.read(LED_GIRO_IZQ))
            pi.write(LED_GIRO_DER, 0)
        elif direccion_actual == DER:
            pi.write(LED_GIRO_DER, not pi.read(LED_GIRO_DER))
            pi.write(LED_GIRO_IZQ, 0)
        else:
            pi.write(LED_GIRO_IZQ, 0)
            pi.write(LED_GIRO_DER, 0)

        # Lógica Intermitentes Rojas (Tecla I)
        if intermitentes_rojos_on:
            pi.write(LED_ROJO, not pi.read(LED_ROJO))
        
        time.sleep(0.5)

# ==========================================
# LÓGICA DE CONTROL
# ==========================================
def is_data():
    return select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], [])

def controlar_coche():
    global velocidad_actual, direccion_actual, ejecutando 
    global luces_blancas_on, intermitentes_rojos_on
    
    # Iniciar hilo de luces
    threading.Thread(target=hilo_luces, daemon=True).start()

    old_settings = termios.tcgetattr(sys.stdin)
    
    # Armado del ESC
    pi.set_servo_pulsewidth(ESC_PIN, NEUTRO)
    pi.set_servo_pulsewidth(SERVO_PIN, NEUTRO)
    time.sleep(2)

    last_s_press = 0 

    try:
        tty.setcbreak(sys.stdin.fileno())
        while ejecutando:
            # Apagar luz roja de freno si no se está presionando 'S' 
            # y si las intermitentes rojas están apagadas
            if time.time() - last_s_press > 0.7 and not intermitentes_rojos_on:
                pi.write(LED_ROJO, 0)

            if is_data():
                char = sys.stdin.read(1).lower()
                
                # MOVIMIENTO
                if char == 'w':
                    if velocidad_actual < MAX_AVANCE:
                        velocidad_actual += PASO_VELOCIDAD
                elif char == 's':
                    pi.write(LED_ROJO, 1)
                    last_s_press = time.time()
                    if velocidad_actual > MAX_REVERSA:
                        velocidad_actual -= PASO_VELOCIDAD
                elif char == 'a':
                    direccion_actual = IZQ
                elif char == 'd':
                    direccion_actual = DER
                elif char == 'c':
                    direccion_actual = NEUTRO
                elif char == ' ':
                    velocidad_actual = NEUTRO

                # LUCES
                elif char == 'l':
                    luces_blancas_on = not luces_blancas_on
                    pi.write(LED_BLANCO, 1 if luces_blancas_on else 0)
                elif char == 'i':
                    intermitentes_rojos_on = not intermitentes_rojos_on
                    if not intermitentes_rojos_on:
                        pi.write(LED_ROJO, 0) # Asegurar que apague al desactivar

                # SALIR
                elif char == 'q':
                    ejecutando = False
            
            # Actualización de hardware
            pi.set_servo_pulsewidth(ESC_PIN, velocidad_actual)
            pi.set_servo_pulsewidth(SERVO_PIN, direccion_actual)
            
            time.sleep(0.01)
            
    finally:
        # Restaurar sistema al salir
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        pi.set_servo_pulsewidth(ESC_PIN, NEUTRO)
        pi.set_servo_pulsewidth(SERVO_PIN, NEUTRO)
        for p in [LED_BLANCO, LED_ROJO, LED_GIRO_IZQ, LED_GIRO_DER]:
            pi.write(p, 0)
        pi.stop()

if __name__ == "__main__":
    controlar_coche()aros
LED_ROJO = 27     # Freno / Intermitentes Rojas
LED_GIRO_IZQ = 22 
LED_GIRO_DER = 23 

# Valores PWM
NEUTRO = 1500
MAX_AVANCE = 1750
MAX_REVERSA = 1300
PASO_VELOCIDAD = 15
IZQ = 1250
DER = 1750

# ==========================================
# VARIABLES DE ESTADO
# ==========================================
pi = pigpio.pi()
velocidad_actual = NEUTRO
direccion_actual = NEUTRO
luces_blancas_on = False
intermitentes_rojos_on = False
ejecutando = True

# Inicializar Pines LED
for p in [LED_BLANCO, LED_ROJO, LED_GIRO_IZQ, LED_GIRO_DER]:
    pi.set_mode(p, pigpio.OUTPUT)
    pi.write(p, 0)

# ==========================================
# HILO DE LUCES (INTERMITENTES)
# ==========================================
def hilo_luces():
    global ejecutando, direccion_actual, intermitentes_rojos_on
    while ejecutando:
        # Lógica Intermitentes de Giro (Amarillas)
        if direccion_actual == IZQ:
            pi.write(LED_GIRO_IZQ, not pi.read(LED_GIRO_IZQ))
            pi.write(LED_GIRO_DER, 0)
        elif direccion_actual == DER:
            pi.write(LED_GIRO_DER, not pi.read(LED_GIRO_DER))
            pi.write(LED_GIRO_IZQ, 0)
        else:
            pi.write(LED_GIRO_IZQ, 0)
            pi.write(LED_GIRO_DER, 0)

        # Lógica Intermitentes Rojas (Tecla I)
        if intermitentes_rojos_on:
            pi.write(LED_ROJO, not pi.read(LED_ROJO))
        
        time.sleep(0.4)

# ==========================================
# LÓGICA DE CONTROL
# ==========================================
def is_data():
    return select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], [])

def controlar_coche():
    global velocidad_actual, direccion_actual, ejecutando 
    global luces_blancas_on, intermitentes_rojos_on
    
    # Iniciar hilo de luces
    threading.Thread(target=hilo_luces, daemon=True).start()

    old_settings = termios.tcgetattr(sys.stdin)
    
    # Armado del ESC
    pi.set_servo_pulsewidth(ESC_PIN, NEUTRO)
    pi.set_servo_pulsewidth(SERVO_PIN, NEUTRO)
    time.sleep(2)

    last_s_press = 0 

    try:
        tty.setcbreak(sys.stdin.fileno())
        while ejecutando:
            # Apagar luz roja de freno si no se está presionando 'S' 
            # y si las intermitentes rojas están apagadas
            if time.time() - last_s_press > 0.3 and not intermitentes_rojos_on:
                pi.write(LED_ROJO, 0)

            if is_data():
                char = sys.stdin.read(1).lower()
                
                # MOVIMIENTO
                if char == 'w':
                    if velocidad_actual < MAX_AVANCE:
                        velocidad_actual += PASO_VELOCIDAD
                elif char == 's':
                    pi.write(LED_ROJO, 1)
                    last_s_press = time.time()
                    if velocidad_actual > MAX_REVERSA:
                        velocidad_actual -= PASO_VELOCIDAD
                elif char == 'a':
                    direccion_actual = IZQ
                elif char == 'd':
                    direccion_actual = DER
                elif char == 'c':
                    direccion_actual = NEUTRO
                elif char == ' ':
                    velocidad_actual = NEUTRO

                # LUCES
                elif char == 'l':
                    luces_blancas_on = not luces_blancas_on
                    pi.write(LED_BLANCO, 1 if luces_blancas_on else 0)
                elif char == 'i':
                    intermitentes_rojos_on = not intermitentes_rojos_on
                    if not intermitentes_rojos_on:
                        pi.write(LED_ROJO, 0) # Asegurar que apague al desactivar

                # SALIR
                elif char == 'q':
                    ejecutando = False
            
            # Actualización de hardware
            pi.set_servo_pulsewidth(ESC_PIN, velocidad_actual)
            pi.set_servo_pulsewidth(SERVO_PIN, direccion_actual)
            
            time.sleep(0.01)
            
    finally:
        # Restaurar sistema al salir
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        pi.set_servo_pulsewidth(ESC_PIN, NEUTRO)
        pi.set_servo_pulsewidth(SERVO_PIN, NEUTRO)
        for p in [LED_BLANCO, LED_ROJO, LED_GIRO_IZQ, LED_GIRO_DER]:
            pi.write(p, 0)
        pi.stop()

if __name__ == "__main__":
    controlar_coche()
        tty.setcbreak(sys.stdin.fileno())
        while ejecutando:
            # Apagar luz roja de freno si no se está presionando 'S' 
            # y si las intermitentes rojas están apagadas
            if time.time() - last_s_press > 0.3 and not intermitentes_rojos_on:
                pi.write(LED_ROJO, 0)

            if is_data():
                char = sys.stdin.read(1).lower()
                
                # MOVIMIENTO
                if char == 'w':
                    if velocidad_actual < MAX_AVANCE:
                        velocidad_actual += PASO_VELOCIDAD
                elif char == 's':
                    pi.write(LED_ROJO, 1)
                    last_s_press = time.time()
                    if velocidad_actual > MAX_REVERSA:
                        velocidad_actual -= PASO_VELOCIDAD
                elif char == 'a':
                    direccion_actual = IZQ
                elif char == 'd':
                    direccion_actual = DER
                elif char == 'c':
                    direccion_actual = NEUTRO
                elif char == ' ':
                    velocidad_actual = NEUTRO

                # LUCES
                elif char == 'l':
                    luces_blancas_on = not luces_blancas_on
                    pi.write(LED_BLANCO, 1 if luces_blancas_on else 0)
                elif char == 'i':
                    intermitentes_rojos_on = not intermitentes_rojos_on
                    if not intermitentes_rojos_on:
                        pi.write(LED_ROJO, 0) # Asegurar que apague al desactivar

                # SALIR
                elif char == 'q':
                    ejecutando = False
            
            # Actualización de hardware
            pi.set_servo_pulsewidth(ESC_PIN, velocidad_actual)
            pi.set_servo_pulsewidth(SERVO_PIN, direccion_actual)
            
            time.sleep(0.01)
            
    finally:
        # Restaurar sistema al salir
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        pi.set_servo_pulsewidth(ESC_PIN, NEUTRO)
        pi.set_servo_pulsewidth(SERVO_PIN, NEUTRO)
        for p in [LED_BLANCO, LED_ROJO, LED_GIRO_IZQ, LED_GIRO_DER]:
            pi.write(p, 0)
        pi.stop()

if __name__ == "__main__":
    controlar_coche()
