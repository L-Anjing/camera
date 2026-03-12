import sys
import numpy as np
import cv2

npy_path = sys.argv[1]
png_path = sys.argv[2]

mask = np.load(npy_path)
cv2.imwrite(png_path, mask)
print(f"✅ 保存成功: {png_path}")
