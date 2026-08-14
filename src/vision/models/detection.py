from ultralytics import YOLO
import cv2

model = YOLO("/home/bryan/detection_pt/chair.pt")

cap = cv2.VideoCapture(0)

if not cap.isOpened():
    print("Cannot open webcam")
    exit()

while True:
    ret, frame = cap.read()
    if not ret:
        break

    results = model(frame, conf=0.6)

    annotated_frame = results[0].plot()

    cv2.imshow("YOLO Webcam Test", annotated_frame)

    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

cap.release()
cv2.destroyAllWindows()