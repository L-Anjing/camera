# train_yolo_seg.py
import sys
import os
# 删除本地源码路径（你说的是 /home/li/third-pkg/ultralytics-main）
sys.path = [p for p in sys.path if "third-pkg/ultralytics-main" not in p]
os.environ["PYTHONPATH"] = ""  # 清空可能的环境变量影响

import ultralytics
print("🔥 当前使用的 ultralytics 路径：", ultralytics.__file__)

from ultralytics import YOLO

# 配置
workspace = "/home/li/camera_cxx/workspace"
dataset_dir = os.path.join(workspace, "dataset_yolo_seg")      # 已划分好的 train/val
model_out_dir = os.path.join(workspace, "model_generate")
os.makedirs(model_out_dir, exist_ok=True)

yaml_path = os.path.join(dataset_dir, "dataset.yaml")    # YOLOv8 数据配置文件


model = YOLO("yolov8s-seg.pt")


# 开始训练
model.train(
    data=yaml_path,
    epochs=100,          # 可根据需求调整
    imgsz=640,           # 图片尺寸
    batch=8,             # 根据显存调整
    project=model_out_dir,
    name="yolo_seg_r2_real",  # 保存文件夹名
    workers=4,
    device=0             # 如果想用CPU改为 'cpu'
)

print(f"\n🎉 训练完成！模型保存在: {model_out_dir}/yolo_seg_r2_real/")
