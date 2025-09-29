import cv2 #librería OpenCV, para visión por computadora.
import numpy as np #para trabajar con arreglos numéricos.
import glob #sirve para buscar archivos que coincidan con un patrón (como chess_*.png).

# -------------- Tamaño del patrón (intersecciones internas del tablero de aljedres) --------------
pattern_size = (8, 6)

# -------------- Medida real entre puntos, tamaño de cuadro del tablero --------------
square_size = 50.0  # en milímetros

# -------------- Coordenadas 3D del patrón (en mm) --------------
objp = np.zeros((pattern_size[0]*pattern_size[1], 3), np.float32) #Crea un arreglo 3D lleno de ceros (x, y, z) para todos los puntos del patrón.48 (8*6)
objp[:, :2] = np.mgrid[0:pattern_size[0], 0:pattern_size[1]].T.reshape(-1, 2) #Genera una malla 2D de coordenadas (x, y), Representa la posición de cada esquina del patrón en el mundo real.
objp *= square_size  # << Aquí aplicamos los 25 mm

# -------------- Listas para puntos --------------
objpoints = []  # puntos 3D reales
imgpoints = []  # puntos 2D en la imagen

# -------------- Cargar imágenes capturadas del patrón --------------
images = glob.glob('chess_*.png')

for fname in images: #Recorre cada imagen. Lee la imagen y la convierte a escala de grises (necesario para detectar el patrón).
    img = cv2.imread(fname)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    ret, corners = cv2.findChessboardCorners(gray, pattern_size, None)

    if ret:
        objpoints.append(objp)
        imgpoints.append(corners)

        # -------------- Dibujar esquinas (opcional) --------------
        cv2.drawChessboardCorners(img, pattern_size, corners, ret)
        cv2.imshow('Esquinas detectadas', img)
        cv2.waitKey(1000)

cv2.destroyAllWindows()

# -------------- Calibración --------------
ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
    objpoints, imgpoints, gray.shape[::-1], None, None
)

# -------------- Guardar parámetros --------------
np.savez("calibracionn", mtx=mtx, dist=dist)

# -------------- Mostrar resultados --------------
print("Calibración completa")
print("Matriz de cámara (intrínseca):\n", mtx)
print("\nCoeficientes de distorsión:\n", dist)
