import cv2
import os

# Parámetros de recorte
new_w, new_h = 320, 200 

# Carpeta donde se guardarán las imágenes
output_dir = 'capturas'
os.makedirs(output_dir, exist_ok=True)

# Inicializa la cámara (0 = dispositivo por defecto)
cap = cv2.VideoCapture(1)
if not cap.isOpened():
    print("No se pudo abrir la cámara.")
    exit()

print("Presiona ENTER para capturar, ESC para salir.")

count = 0
while True:
    ret, frame = cap.read()
    if not ret:
        print("Error al leer frame de la cámara.")
        break

    # Muestra la vista previa
    h, w = frame.shape[:2]
    x0 = (w - new_w) // 2
    y0 = new_h  # punto de partida vertical
    crop = frame[0:200, x0:x0 + new_w]

    cv2.imshow('Vista previa', crop)
    key = cv2.waitKey(1) & 0xFF

    # ENTER → captura
    if key in (10, 13):
        
        # recorta a new_h alto y new_w ancho
        
        
        filename = os.path.join(output_dir, f"img_{count:03d}.png")
        cv2.imwrite(filename, crop)
        print(f"[{count}] Guardada → {filename}")
        count += 1
    
    # ESC → sale
    elif key == 27:
        break
    
# Limpieza
cap.release()
cv2.destroyAllWindows()
