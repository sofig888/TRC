import pyrealsense2 as rs
import numpy as np
import cv2

# --- FUNCIÓN "DUMMY" PARA LOS TRACKBARS ---
def nada(x):
    pass

# ==========================================
# CONFIGURACIÓN INICIAL REALSENSE
# ==========================================
# Mantener la exposición manual es CRUCIAL para que el color no cambie solo
EXPOSICION_VAL = 150  
GANANCIA_VAL = 64

pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
# No necesitamos depth para este calibrador

profile = pipeline.start(config)
color_sensor = profile.get_device().query_sensors()[1]
color_sensor.set_option(rs.option.enable_auto_exposure, 0)
color_sensor.set_option(rs.option.exposure, EXPOSICION_VAL)
color_sensor.set_option(rs.option.gain, GANANCIA_VAL)

# ==========================================
# CONFIGURACIÓN DE INTERFAZ (TRACKBARS)
# ==========================================
cv2.namedWindow("Controles HSV", cv2.WINDOW_NORMAL)
cv2.resizeWindow("Controles HSV", 300, 350)

# Valores iniciales sugeridos para cinta azul (ajusta si es necesario)
cv2.createTrackbar("H Min", "Controles HSV", 90, 179, nada)
cv2.createTrackbar("H Max", "Controles HSV", 135, 179, nada)
cv2.createTrackbar("S Min", "Controles HSV", 80, 255, nada) # Bajo para detectar cinta lejana/palida
cv2.createTrackbar("S Max", "Controles HSV", 255, 255, nada)
cv2.createTrackbar("V Min", "Controles HSV", 40, 255, nada) # Bajo para detectar cinta en sombra
cv2.createTrackbar("V Max", "Controles HSV", 255, 255, nada)

# Kernel para operaciones morfológicas (un cuadrado de 5x5 píxeles)
# Si los huecos son muy grandes, aumenta a (7,7)
kernel_relleno = np.ones((5,5), np.uint8)

print("Ajusta los deslizadores hasta que SOLO la cinta azul sea visible.")
print("Presiona 'q' para salir.")

try:
    while True:
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame:
            continue

        img_original = np.asanyarray(color_frame.get_data())

        # --- PASO 1: SUAVIZADO (FLUIDEZ) ---
        # Un desenfoque ligero elimina el ruido granulado antes de procesar el color.
        # Esto hace que los bordes de la detección sean menos "nerviosos".
        img_blur = cv2.GaussianBlur(img_original, (5, 5), 0)

        # --- PASO 2: CONVERSIÓN A HSV ---
        hsv = cv2.cvtColor(img_blur, cv2.COLOR_BGR2HSV)

        # --- PASO 3: LECTURA DE TRACKBARS EN TIEMPO REAL ---
        h_min = cv2.getTrackbarPos("H Min", "Controles HSV")
        h_max = cv2.getTrackbarPos("H Max", "Controles HSV")
        s_min = cv2.getTrackbarPos("S Min", "Controles HSV")
        s_max = cv2.getTrackbarPos("S Max", "Controles HSV")
        v_min = cv2.getTrackbarPos("V Min", "Controles HSV")
        v_max = cv2.getTrackbarPos("V Max", "Controles HSV")

        lower_blue = np.array([h_min, s_min, v_min])
        upper_blue = np.array([h_max, s_max, v_max])

        # --- PASO 4: CREACIÓN DE MÁSCARA INICIAL ---
        mask_cruda = cv2.inRange(hsv, lower_blue, upper_blue)

        # --- PASO 5: MORFOLOGÍA (EL SECRETO DEL RELLENO) ---
        # Operación "CLOSE" (Cierre): Dilata y luego erosiona.
        # Cierra pequeños agujeros negros DENTRO de la cinta.
        mask_rellena = cv2.morphologyEx(mask_cruda, cv2.MORPH_CLOSE, kernel_relleno, iterations=2)
        
        # Operación "OPEN" (Apertura): Erosiona y luego dilata.
        # Elimina pequeños puntos blancos de ruido en el FONDO.
        mask_final = cv2.morphologyEx(mask_rellena, cv2.MORPH_OPEN, kernel_relleno)

        # --- VISUALIZACIÓN ---
        # Usamos la máscara para mostrar solo la parte azul de la imagen original
        resultado_visual = cv2.bitwise_and(img_original, img_original, mask=mask_final)

        # Opcional: Mostrar la máscara en blanco y negro al lado para comparar
        # mask_bgr = cv2.cvtColor(mask_final, cv2.COLOR_GRAY2BGR)
        # combinado = np.hstack((img_original, mask_bgr, resultado_visual))
        # cv2.imshow("Calibracion Cinta Azul", combinado)
        
        # Mostramos el resultado limpio
        cv2.imshow("Resultado Final (Ajusta Trackbars)", resultado_visual)

        key = cv2.waitKey(1)
        if key & 0xFF == ord('q'):
            # Imprimir los valores finales para que los puedas copiar
            print("\n--- VALORES FINALES PARA TU CÓDIGO UNIFICADO ---")
            print(f"AZUL_BAJO = np.array([{h_min}, {s_min}, {v_min}])")
            print(f"AZUL_ALTO = np.array([{h_max}, {s_max}, {v_max}])")
            print("------------------------------------------------\n")
            break

finally:
    pipeline.stop()
    cv2.destroyAllWindows()
