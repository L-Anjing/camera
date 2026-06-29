# yolo_train_dete.py
import sys
import os
# 删除本地源码路径（你说的是 /home/li/third-pkg/ultralytics-main）
sys.path = [p for p in sys.path if "third-pkg/ultralytics-main" not in p]
os.environ["PYTHONPATH"] = ""  # 清空可能的环境变量影响

import ultralytics
print("🔥 当前使用的 ultralytics 路径：", ultralytics.__file__)

from ultralytics import YOLO
# 配置
                                
workspace = "/home/li/camera_ws/src/camera_bridge/workspace"
dataset_dir = os.path.join(workspace, "dataset_yolo_dete")      # 已划分好的 train/val
model_out_dir = os.path.join(workspace, "model_generate")
os.makedirs(model_out_dir, exist_ok=True)

yaml_path = os.path.join(dataset_dir, "data.yaml")    # YOLOv8 数据配置文件


# 初始化模型
model = YOLO("yolov8m.pt")  # 使用预训练的 YOLOv8m 模型

# 开始训练
model.train(
    data=yaml_path,
    epochs=200,          # 可根据需求调整
    imgsz=640,           # 图片尺寸
    batch=8,             # 根据显存调整
    project=model_out_dir,
    name="yolo_dete_1_20",  # 保存文件夹名
    workers=4,
    device=0,             # 如果想用CPU改为 'cpu'
    resume=False,
)


