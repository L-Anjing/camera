from ultralytics import YOLO

# 1. 加载训练好的模型
model = YOLO("/home/li/models/best.pt")

# 2. 导出为 ONNX
model.export(
    format="onnx",
    opset=16,            # ✅ 指定 opset ≤ 17
    dynamic=False,        # ❌ Jetson 不建议 dynamic（TensorRT 8.5 对动态 shape 兼容不好）
    simplify=True,        # ✅ 简化网络结构
    imgsz=640             # 你的输入大小，如 YOLO 默认 640
)
