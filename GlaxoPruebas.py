from ultralytics import YOLO
import cv2
import numpy as np
import csv
import signal
import sys
import time
from collections import Counter


model = YOLO("best_12v_93.pt").to("cuda")  # or .to("cpu")
#cap = cv2.VideoCapture(1)
cap = cv2.VideoCapture("Prueba1.mp4")
#cap = cv2.VideoCapture("Prueba2.mp4")

# ————— Parámetros —————
START_DELAY_FRAMES = 5   # frames tras primera detección hasta empezar a contar
END_DELAY_FRAMES   = 40  # frames sin detección tras última para cerrar la caja
#END_DELAY_FRAMES   = 25 # cam2frames sin detección tras última para cerrar la caja
FRAME_PERIOD       = 1.0 / 60.0  # ≈0.01667 s
# ————— Variables de inventario —————
collecting            = False  # si ya estamos dentro de la ventana de conteo
start_delay_count     = 0      # cuenta inicial de frames tras detectar por primera vez
no_detection_count    = 0      # cuenta de frames sin detección tras collecting=True
box_detections        = []     # todos los class_id recogidos durante collecting
inventory             = []     # etiqueta final (más frecuente) por cada caja

#counts         = {}    # { label_id: total_count }
#box_detections = []    # detecciones acumuladas mientras hay caja
#in_box         = False # indicador de “estoy viendo una caja”
#no_detection_count = 0     # frames consecutivos SIN detección
CSV_OUTPUT = "counts.csv"

while True:
    t0 = time.time()

    ret, frame = cap.read()
    if not ret:
        break

    # 1) Resize to speed up processing
    frame = cv2.resize(frame, (640, 480))

    # 2) CAmara  uno recorte de imagen
    h, w = frame.shape[:2]

    new_w, new_h = 320,80
    x0 = (w - new_w) // 2
    y0 =  new_h
    frame = frame[0:200 , x0:x0 + new_w]#"""

    # 2) CAmara  uno recorte de imagen
    """h, w = frame.shape[:2]
    new_w, new_h = 350,270
    x0 = (w - new_w) // 2
    y0 = (h - new_h) // 2
    frame = frame[y0:y0 + new_h, x0:x0 + new_w] #"""
    # 3) Run YOLO on the cropped region
    res = model(frame, conf=0.65, imgsz=512, verbose=False)[0]
    # ————— Filtrar detecciones que toquen top o bottom —————
    # frame is size new_w×new_h after your crop
    frame_height = new_h
    filtered_ids = []
    for coords, cls in zip(res.boxes.xyxy, res.boxes.cls):
    # coords es tensor [x1, y1, x2, y2]
        x1, y1, x2, y2 = coords.cpu().numpy().astype(int)
        # si toca el top (y1==0) o el bottom (y2==frame_height) lo descartas
        if y1 == 0 or y2 >= 180:
            continue
        # si pasa el filtro, guardas el ID de clase
        filtered_ids.append(int(cls.item()))
    # 4) Display
    frame_ids = filtered_ids

    # ——— Lógica “10 frames tras la última detección” ———
    if frame_ids:
        if not collecting:
            # estamos en el periodo de delay inicial
            start_delay_count += 1
            if start_delay_count >= START_DELAY_FRAMES:
                collecting = True
                no_detection_count = 0
        else:
            # ya estamos colectando
            box_detections.extend(frame_ids)
            no_detection_count = 0
            print(box_detections)
    else:
        # no hay detección
        if not collecting:
            # resetear si la detección desaparece antes de completar el delay
            start_delay_count = 0
        else:
            # dentro de collecting, contamos frames vacíos para cerrar la caja
            no_detection_count += 1
            if no_detection_count >= END_DELAY_FRAMES:
                
                # cerramos la caja: calculamos la etiqueta más común
                most_common = int(np.bincount(box_detections).argmax())
                print(most_common)
                inventory.append(most_common)
                # reseteamos todo para la siguiente caja
                collecting = False
                start_delay_count = 0
                no_detection_count = 0
                box_detections.clear()

    cv2.imshow("YOLO Fast (center crop)", res.plot())

    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

     # 6) Control de FPS a 60
    elapsed = time.time() - t0
    to_wait = FRAME_PERIOD - elapsed
    if to_wait > 0:
        time.sleep(to_wait)


cap.release()
cv2.destroyAllWindows()



# ——— Imprimir estadísitcas ———
from collections import Counter
counts = Counter(inventory)
print("Etiqueta más frecuente por caja procesada:")
for label_id, cnt in counts.items():
    print(f"  ID {label_id}: {cnt} cajas")
