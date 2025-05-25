from ultralytics import YOLO
import cv2

model = YOLO("best_12v_85_ft.pt").to("cuda")  # o .to("cpu")

cap = cv2.VideoCapture("Prueba1.mp4")

while True:
    ret, frame = cap.read()
    if not ret:
        break

    frame = cv2.resize(frame, (640, 480))  # Acelera procesamiento
    res = model(frame, conf=0.75, verbose=False)[0]
    cv2.imshow("YOLO Fast", res.plot())

    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

cap.release()
cv2.destroyAllWindows()
