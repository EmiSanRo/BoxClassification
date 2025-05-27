import cv2
import time
from datetime import datetime
import os

# 1. Abre la cámara (0 = cámara web por defecto).
cap = cv2.VideoCapture(1)

# Verifica que la cámara se abrió bien
if not cap.isOpened():
    raise RuntimeError("No pude acceder a la cámara. Revisa la conexión o el índice (0, 1, 2…).")

# 2. Crea una carpeta para las fotos (solo la primera vez).
output_dir = "fotos"
os.makedirs(output_dir, exist_ok=True)

print("Presiona la tecla 'q' en la ventana de vista previa para terminar.")

try:
    foto_num = 1
    while True:
        # Captura un frame
        ret, frame = cap.read()
        if not ret:
            print("⚠️  No se pudo leer el frame. Deteniendo…")
            break

        # 3. Muestra una vista previa (opcional)
        cv2.imshow("Vista previa (presiona q para salir)", frame)

        # 4. Guarda la imagen con marca de tiempo
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"{output_dir}/foto_{timestamp}.jpg"
        cv2.imwrite(filename, frame)
        print(f"📸  Guardada: {filename} (foto #{foto_num})")
        foto_num += 1

        # 5. Espera 3 segundos o sale si el usuario presiona 'q'
        start = time.time()
        while time.time() - start < 3:
            # Revisa si se presionó la tecla 'q'
            if cv2.waitKey(1) & 0xFF == ord('q'):
                raise KeyboardInterrupt
            # Pequeña espera para no saturar la CPU
            time.sleep(0.01)

except KeyboardInterrupt:
    print("\n⏹️  Captura detenida por el usuario.")

finally:
    cap.release()
    cv2.destroyAllWindows()