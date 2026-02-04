
import cv2
import numpy as np
import math
import time

# ======================
# CAM fija
# ======================
CAM_INDEX = 4

# ======================
# Umbrales HSV (más estrictos para rojo)
# Ajusta con M para ver máscara
# ======================
S_MIN = 120
V_MIN = 70

# ======================
# Filtros geométricos (anti-falsos)
# ======================
MIN_AREA_FRAC = 0.002     # tamaño mínimo relativo
EPS_FRAC = 0.02           # approxPolyDP: 2% del perímetro

# Octágono esperado: 8 lados (con perspectiva a veces 7 o 9)
ALLOW_SIDES = {7, 8, 9}

# Debe ser convexo y "relleno"
MIN_SOLIDITY = 0.92       # más alto = menos falsos
MIN_CIRCULARITY = 0.72    # 1.0 círculo, octágono suele estar alto (ajustable)
ASPECT_MIN = 0.80
ASPECT_MAX = 1.20

# Match contra octágono ideal (Hu moments). Menor = más parecido.
MAX_SHAPE_DIST = 0.25     # baja si sigue confundiendo (ej 0.18)

def make_ideal_octagon(size=200):
    """Crea un contorno de octágono regular ideal para matchShapes."""
    # Octágono regular centrado
    cx, cy = size // 2, size // 2
    r = size * 0.38
    pts = []
    for k in range(8):
        ang = (math.pi / 8.0) + k * (math.pi / 4.0)  # rotado 22.5°
        x = cx + r * math.cos(ang)
        y = cy + r * math.sin(ang)
        pts.append([int(x), int(y)])
    return np.array(pts, dtype=np.int32).reshape((-1, 1, 2))

IDEAL_OCT = make_ideal_octagon(240)

def red_mask(frame_bgr):
    hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)

    lower_red1 = np.array([0,   S_MIN, V_MIN])
    upper_red1 = np.array([10,  255,   255])
    lower_red2 = np.array([170, S_MIN, V_MIN])
    upper_red2 = np.array([180, 255,   255])

    mask = cv2.bitwise_or(
        cv2.inRange(hsv, lower_red1, upper_red1),
        cv2.inRange(hsv, lower_red2, upper_red2)
    )

    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
    return mask

def circularity(contour):
    a = cv2.contourArea(contour)
    p = cv2.arcLength(contour, True) + 1e-6
    return 4.0 * math.pi * a / (p * p)

def detect_octagon_stop(frame):
    h, w = frame.shape[:2]
    frame_area = h * w

    mask = red_mask(frame)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    best = None
    best_score = 1e9  # menor es mejor (shape distance)

    for c in contours:
        area = cv2.contourArea(c)
        if area < MIN_AREA_FRAC * frame_area:
            continue

        peri = cv2.arcLength(c, True)
        approx = cv2.approxPolyDP(c, EPS_FRAC * peri, True)

        sides = len(approx)
        if sides not in ALLOW_SIDES:
            continue

        if not cv2.isContourConvex(approx):
            continue

        x, y, bw, bh = cv2.boundingRect(approx)
        aspect = bw / float(bh)
        if not (ASPECT_MIN <= aspect <= ASPECT_MAX):
            continue

        hull = cv2.convexHull(approx)
        hull_area = cv2.contourArea(hull) + 1e-6
        solidity = area / hull_area
        if solidity < MIN_SOLIDITY:
            continue

        circ = circularity(approx)
        if circ < MIN_CIRCULARITY:
            continue

        # MatchShapes: compara contra octágono ideal
        dist = cv2.matchShapes(approx, IDEAL_OCT, cv2.CONTOURS_MATCH_I1, 0.0)
        if dist > MAX_SHAPE_DIST:
            continue

        # Elige el más parecido (dist mínimo). Si empatan, el más grande.
        score = dist - 0.000001 * area
        if score < best_score:
            best_score = score
            best = (x, y, bw, bh, sides, solidity, circ, dist, approx)

    return best, mask

def main():
    cap = cv2.VideoCapture(CAM_INDEX, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1024)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
    try:
        cap.set(cv2.CAP_PROP_CONVERT_RGB, 1)
    except Exception:
        pass

    if not cap.isOpened():
        print(f"No se pudo abrir la camara index {CAM_INDEX}")
        return

    show_mask = False
    print("Cam index 4. Solo octagonos rojos. Teclas: ESC salir | M mascara")

    while True:
        ret, frame = cap.read()
        if not ret or frame is None:
            continue

        # Asegura BGR
        if len(frame.shape) == 2:
            frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)

        best, mask = detect_octagon_stop(frame)

        vis = frame.copy()

        if best is not None:
            x, y, bw, bh, sides, solidity, circ, dist, approx = best
            cv2.rectangle(vis, (x, y), (x + bw, y + bh), (0, 255, 0), 2)
            cv2.drawContours(vis, [approx], -1, (0, 255, 0), 2)
            cv2.putText(vis, "STOP (octagono)", (20, 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)
            cv2.putText(vis, f"sides={sides} sol={solidity:.2f} circ={circ:.2f} dist={dist:.3f}",
                        (20, 75), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 0), 2)
        else:
            cv2.putText(vis, "Buscando octagono rojo...", (20, 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2)

        cv2.putText(vis, "M: mask | ESC: salir", (20, vis.shape[0] - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

        cv2.imshow("STOP Octagon - cam4", vis)
        if show_mask:
            cv2.imshow("mask rojo", mask)
        else:
            cv2.destroyWindow("mask rojo")

        k = cv2.waitKey(1) & 0xFF
        if k == 27:
            break
        if k in (ord('m'), ord('M')):
            show_mask = not show_mask

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()

