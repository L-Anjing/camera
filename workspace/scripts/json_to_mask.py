import os
import glob
import numpy as np
import labelme
import subprocess

input_dir = "/home/li/camera_cxx/workspace/labels_json"
temp_dir = "/home/li/camera_cxx/workspace/temp_masks"
output_dir = "/home/li/camera_cxx/workspace/masks"

os.makedirs(temp_dir, exist_ok=True)
os.makedirs(output_dir, exist_ok=True)

label_name_to_value = {
    "_background_": 0,
    "1": 1,
    "2": 2,
    "3": 3
}

json_list = glob.glob(os.path.join(input_dir, "*.json"))

for json_file in json_list:
    base = os.path.basename(json_file).replace(".json", "")
    print(f"➡️ 处理: {base}.json ...")

    lf = labelme.LabelFile(filename=json_file)
    mask, _ = labelme.utils.shapes_to_label(
        img_shape=(720, 1280, 3),
        shapes=lf.shapes,
        label_name_to_value=label_name_to_value
    )

    npy_path = os.path.join(temp_dir, base + ".npy")
    np.save(npy_path, mask)

    # ✅ 调用系统 Python 保存为 PNG（使用 cv2）
    subprocess.run([
        "/usr/bin/python3",
        "/home/li/camera_cxx/workspace/scripts/json_to_mask_save_png.py",
        npy_path,
        os.path.join(output_dir, base + ".png")
    ], check=True)

print("\n🎉 所有 mask 已保存到:", output_dir)
