import cv2

cap = cv2.VideoCapture(0)  #0 para la cámara de la compu, 1 para camara usb

contador = 0

while True:
    ret, frame = cap.read()
    if not ret:
        print("No se pudo acceder a la cámara.")
        break

    cv2.imshow('Vista previa - presiona "s" para guardar', frame)

    key = cv2.waitKey(1)
    if key == ord('s'):
        filename = f'chess_{contador}.png'
        cv2.imwrite(filename, frame)
        print(f"Imagen guardada: {filename}")
        contador += 1
    elif key == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
