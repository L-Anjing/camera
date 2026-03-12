import os
import glob
import json
import cv2


input_dir = "/home/li/camera_cxx/workspace/labels_json"  
image_root = "/home/li/camera_cxx/workspace/images"       

json_list = glob.glob(os.path.join(input_dir, "*.json"))

# 收集所有图片的路径（包含子文件夹）
all_images = glob.glob(os.path.join(image_root, "**", "*.*"), recursive=True)
all_images_map = {os.path.basename(p): p for p in all_images}

print(f"共找到 {len(json_list)} 个标注文件，{len(all_images)} 张图片。\n")

for json_file in json_list:
    try:
        with open(json_file, "r") as f:
            data = json.load(f)

        img_name = os.path.basename(data.get("imagePath", ""))
        if not img_name:
            raise ValueError("JSON 文件缺少 imagePath 字段")

        if img_name not in all_images_map:
            raise FileNotFoundError(f"未找到匹配图片文件：{img_name}")

        img_full_path = all_images_map[img_name]
        img = cv2.imread(img_full_path)

        if img is None:
            raise ValueError(f"无法读取图片内容：{img_full_path}")

        h, w, c = img.shape
        print(f"{os.path.basename(json_file)}  -->  宽: {w}, 高: {h}, 通道数: {c}")

    except Exception as e:
        print(f"❌ 读取失败：{os.path.basename(json_file)}，错误：{e}")
