# Detección de Señal de Alto con Intel RealSense D435

Este proyecto permite detectar la **señal de alto** en tiempo real usando la cámara **Intel RealSense D435** y Python. Además, muestra la **distancia** desde la cámara hasta la señal detectada.

---

## Descripción general

El código realiza los siguientes pasos:

1. Captura **video y profundidad** desde la cámara RealSense.
2. Convierte la imagen de color a **HSV** y genera una máscara para detectar **rojo**.
3. Encuentra **contornos** en la máscara roja y filtra por:
   - Área mínima
   - Número de vértices (para aproximar óctagono)
   - Relación ancho/alto
4. (Opcional) Verifica el contorno con **template matching** usando una plantilla de la señal de alto (`stop_template.png`).
5. Calcula la **distancia** al objeto usando el frame de profundidad.
6. Dibuja el contorno y la distancia en la ventana de video en tiempo real.

---

## Requisitos

- Python 3.x
- Librerías:
  ```bash
  pip install opencv-python numpy pyrealsense2

## Nota
Cambiar rutas de acceso 
