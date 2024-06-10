from ultralytics import YOLO
import cv2

def main():
    model = YOLO("yolov8n.pt")

    model.train(data="../data-base/data.yaml", epochs=70, imgsz=640)
    matrics = model.val()
    print(matrics)

if __name__ == "__main__":
    main()
