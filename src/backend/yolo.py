from ultralytics import YOLO

model = YOLO("yolov8s.pt")

results = model.train(data="../data-base/data.yaml", epochs=10, imgsz=640)
