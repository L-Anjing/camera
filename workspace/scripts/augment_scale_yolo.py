import os
import cv2
import numpy as np

# ================= 配置 =================
dataset_dir = "/home/li/camera_cxx/workspace/dataset_yolo_dete"
target_h, target_w = 720, 1280        # canvas 尺寸
scale_factors = [0.25, 0.5, 0.75, 1.0] # 缩放比例
# ========================================

subsets = ["train", "valid", "test"]

for subset in subsets:
    image_dir = os.path.join(dataset_dir, subset, "images")
    label_dir = os.path.join(dataset_dir, subset, "labels")

    image_files = [f for f in os.listdir(image_dir) if f.lower().endswith((".jpg", ".png"))]
    print(f"Processing {subset}, total {len(image_files)} images...")

    for img_name in image_files:
        img_path = os.path.join(image_dir, img_name)
        img = cv2.imread(img_path)
        h0, w0 = img.shape[:2]

        # 读取 YOLO 标签
        label_path = os.path.join(label_dir, os.path.splitext(img_name)[0] + ".txt")
        labels = []
        if os.path.exists(label_path):
            with open(label_path) as f:
                for line in f:
                    labels.append([float(x) for x in line.strip().split()])

        for scale in scale_factors:
            # 跳过 scale=1，避免重复原图
            if scale == 1.0:
                continue

            new_w = int(w0 * scale)
            new_h = int(h0 * scale)
            scaled_img = cv2.resize(img, (new_w, new_h))

            # 创建黑色 canvas
            canvas = np.zeros((target_h, target_w, 3), dtype=np.uint8)

            # 计算居中偏移
            y_offset = (target_h - new_h) // 2
            x_offset = (target_w - new_w) // 2

            # 计算填充区域
            y1 = max(y_offset, 0)
            x1 = max(x_offset, 0)
            y2 = min(y_offset + new_h, target_h)
            x2 = min(x_offset + new_w, target_w)

            sy1 = 0 if y_offset >= 0 else -y_offset
            sx1 = 0 if x_offset >= 0 else -x_offset
            sy2 = sy1 + (y2 - y1)
            sx2 = sx1 + (x2 - x1)

            # 放入 canvas
            canvas[y1:y2, x1:x2] = scaled_img[sy1:sy2, sx1:sx2]

            # 保存增强图像（加入原文件夹）
            out_img_name = os.path.splitext(img_name)[0] + f"_s{int(scale*100)}.jpg"
            cv2.imwrite(os.path.join(image_dir, out_img_name), canvas)

            # 调整 YOLO 标签
            out_labels = []
            for lbl in labels:
                cls, x_c, y_c, w_box, h_box = lbl
                x_c = x_c * w0 * scale + x_offset
                y_c = y_c * h0 * scale + y_offset
                w_box = w_box * w0 * scale
                h_box = h_box * h0 * scale

                x_c /= target_w
                y_c /= target_h
                w_box /= target_w
                h_box /= target_h

                if 0 < x_c < 1 and 0 < y_c < 1:
                    out_labels.append([cls, x_c, y_c, w_box, h_box])

            out_label_name = os.path.splitext(img_name)[0] + f"_s{int(scale*100)}.txt"
            with open(os.path.join(label_dir, out_label_name), "w") as f:
                for lbl in out_labels:
                    f.write(" ".join([str(x) for x in lbl]) + "\n")

print("Done! Augmented images added to original folders.")
