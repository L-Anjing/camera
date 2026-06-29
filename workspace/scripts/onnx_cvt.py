#使用源码修改后的ultralytics 
#注：用到了torch，在conda环境内运行此脚本
import sys
# 将本地源码路径插到最前面
sys.path.insert(0, "/home/li/third-pkg/ultralytics-main")

import ultralytics
from ultralytics import YOLO

print("🔥 使用的 ultralytics:", ultralytics.__file__)


model = YOLO("/home/li/camera_ws/src/camera_bridge/workspace/model_generate/yolo_dete_1_20/weights/best.pt")

success = model.export(format="onnx", dynamic=True, simplify=True)
