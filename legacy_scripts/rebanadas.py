import pyrealsense2 as rs
import numpy as np
import cv2

# ==========================================
# CONFIGURACIÓN CALIBRADA (AZUL)
# ==========================================
AZUL_BAJO = np.array([82, 62, 110])
AZUL_ALTO = np.array([135, 255, 255])

# Configuración de las "Rebanadas" (Scanlines)
Y_INFERIOR = 420  # Más cerca del coche
Y_SUPERIOR = 320  # Más lejos (anticipación)
ALTO_REBANADA = 10 # Grosor de la franja en píxeles

# Inicializar RealSense
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
pipeline.start(config)

def procesar_rebanada(imagen_hsv, y_coord, alto):
    """Extrae una franja, busca los dos puntos azules y calcula centro/ancho"""
    # Recortar la franja
    roi = imagen_hsv[y_coord : y_coord + alto, :]
    mask = cv2.inRange(roi, AZUL_BAJO, AZUL_ALTO)
    mask = cv2.dilate(mask, None, iterations=1)
    
    # Encontrar contornos en la franja
    cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    puntos_x = []
    for c in cnts:
        if cv2.contourArea(c) > 50: # Filtro de ruido
            M = cv2.moments(c)
            if M["m00"] != 0:
                puntos_x.append(int(M["m10"] / M["m00"]))
    
    puntos_x.sort()
    
    centro = None
    ancho = None
    
    if len(puntos_x) >= 2:
        # Suponemos que los dos extremos son las líneas de la pista
        x_izq = puntos_x[0]
        x_der = puntos_x[-1]
        ancho = x_der - x_izq
        centro = x_izq + (ancho // 2)
        return centro, ancho, x_izq, x_der, mask
    
    return None, None, None, None, mask

try:
    while True:
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame: continue

        img = np.asanyarray(color_frame.get_data())
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        img_visual = img.copy()

        # --- PROCESAR REBANADA INFERIOR ---
        c_inf, w_inf, x_izq_i, x_der_i, mask_i = procesar_rebanada(hsv, Y_INFERIOR, ALTO_REBANADA)
        
        # --- PROCESAR REBANADA SUPERIOR ---
        c_sup, w_sup, x_izq_s, x_der_s, mask_s = procesar_rebanada(hsv, Y_SUPERIOR, ALTO_REBANADA)

        # --- DIBUJAR INTERFAZ (SOLO EN LAS REBANADAS) ---
        for y, c, w, xi, xd, mask in [(Y_INFERIOR, c_inf, w_inf, x_izq_i, x_der_i, mask_i), 
                                      (Y_SUPERIOR, c_sup, w_sup, x_izq_s, x_der_s, mask_s)]:
            
            # Pintar la máscara solo en la franja para efecto "Cyber"
            mask_bgr = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
            img_visual[y : y + ALTO_REBANADA, :] = cv2.addWeighted(
                img_visual[y : y + ALTO_REBANADA, :], 0.3, mask_bgr, 0.7, 0
            )

            if c is not None:
                # Dibujar puntos de las líneas
                cv2.circle(img_visual, (xi, y + 5), 7, (255, 255, 0), -1)
                cv2.circle(img_visual, (xd, y + 5), 7, (255, 255, 0), -1)
                # Dibujar centro de pista
                cv2.circle(img_visual, (c, y + 5), 5, (0, 255, 0), -1)
                # Texto informativo
                info = f"W: {w}px | C: {c}"
                cv2.putText(img_visual, info, (xd + 10, y + 10), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)

        # --- CÁLCULO DE TRAYECTORIA (TEORÍA) ---
        if c_inf and c_sup:
            # Dibujar línea de trayectoria entre centros
            cv2.line(img_visual, (c_inf, Y_INFERIOR), (c_sup, Y_SUPERIOR), (0, 255, 255), 2)
            angulo_pista = c_sup - c_inf
            cv2.putText(img_visual, f"ANGULO PISTA: {angulo_pista}", (10, 30), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

        cv2.imshow("Scanline Vision System", img_visual)
        if cv2.waitKey(1) & 0xFF == ord('q'): break

finally:
    pipeline.stop()
    cv2.destroyAllWindows()
