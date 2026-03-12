import cv2
import numpy as np

mask = cv2.imread("/home/li/camera_cxx/workspace/masks/R2_KFS_14_19.png", cv2.IMREAD_UNCHANGED)
visible = (mask * 80).astype(np.uint8)  # 放大亮度
cv2.imshow("mask visible", visible)
cv2.waitKey(0)
