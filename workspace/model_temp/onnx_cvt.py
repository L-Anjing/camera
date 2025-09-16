from ultralytics import YOLO
import onnx
model = YOLO("/home/li/models/yolov8s.pt")
success = model.export(format="onnx", dynamic=True, simplify=True)