import os
import cv2
import numpy as np
import shutil
from sklearn.model_selection import train_test_split
import yaml

# 路径设置
dataset_aug_dir = "/home/li/camera_cxx/workspace/dataset_aug"
images_dir = os.path.join(dataset_aug_dir, "images_aug")
masks_dir = os.path.join(dataset_aug_dir, "masks_aug")

# YOLO输出目录
yolo_dir = "/home/li/camera_cxx/workspace/dataset_yolo"
os.makedirs(yolo_dir, exist_ok=True)

# 获取所有图像文件
image_files = [f for f in os.listdir(images_dir) if f.endswith(('.jpg', '.png'))]
image_files.sort()

print(f"找到 {len(image_files)} 个图像文件")

# 划分训练集和验证集
train_files, val_files = train_test_split(image_files, test_size=0.2, random_state=42)

print(f"训练集: {len(train_files)} 张")
print(f"验证集: {len(val_files)} 张")

# 创建YOLO目录结构
for split in ['train', 'val']:
    os.makedirs(os.path.join(yolo_dir, 'images', split), exist_ok=True)
    os.makedirs(os.path.join(yolo_dir, 'labels', split), exist_ok=True)

def mask_to_yolo_format(mask_path, img_width, img_height):
    """将分割mask转换为YOLO分割格式"""
    mask = cv2.imread(mask_path, cv2.IMREAD_GRAYSCALE)
    if mask is None:
        print(f"无法读取mask: {mask_path}")
        return []

    # 检查mask中的像素值分布
    unique_vals, counts = np.unique(mask, return_counts=True)
    print(f"Mask {os.path.basename(mask_path)} 的像素值分布: {dict(zip(unique_vals, counts))}")
    
    # 映射关系：mask像素值 -> YOLO类别ID
    pixel_to_class = {
        2: 1,  # 您的R2_Real（像素值2）-> YOLO类别1
        1: 0,# 将来R1数据（像素值1）-> YOLO类别0
        3: 2   # 将来R2_Fake数据（像素值3）-> YOLO类别2
    }
    
    yolo_segments = []
    
    for pixel_value, class_id in pixel_to_class.items():
        target_mask = (mask == pixel_value).astype(np.uint8)
        
        # 检查是否有目标像素
        if np.sum(target_mask) == 0:
            print(f"警告: 在 {mask_path} 中未找到像素值 {pixel_value} 的目标")
            continue
        contours, _ = cv2.findContours(
            (mask == pixel_value).astype(np.uint8), 
            cv2.RETR_EXTERNAL, 
            cv2.CHAIN_APPROX_SIMPLE
        )
        print(f"在 {mask_path} 中找到 {len(contours)} 个轮廓")
        for contour in contours:
            if cv2.contourArea(contour) > 10:  # 过滤小区域
                # 归一化坐标
                segment = contour.reshape(-1, 2).astype(float)
                segment[:, 0] /= img_width   # x 归一化
                segment[:, 1] /= img_height  # y 归一化
                
                # 转换为YOLO格式: class x1 y1 x2 y2 ...
                segment_str = f"{class_id} " + " ".join([f"{x:.6f} {y:.6f}" for x, y in segment])
                yolo_segments.append(segment_str)
    
    return yolo_segments

def process_split(file_list, split_name):
    """处理训练集或验证集"""
    for img_file in file_list:
        # 图像路径
        img_path = os.path.join(images_dir, img_file)
        
        # 读取图像获取尺寸
        img = cv2.imread(img_path)
        if img is None:
            continue
            
        img_height, img_width = img.shape[:2]
        
        # mask路径
        mask_file = img_file.replace('.jpg', '.png').replace('.jpeg', '.png')
        mask_path = os.path.join(masks_dir, mask_file)
        
        # 转换mask为YOLO格式
        yolo_segments = mask_to_yolo_format(mask_path, img_width, img_height)
        
        # 复制图像到YOLO目录
        dst_img_path = os.path.join(yolo_dir, 'images', split_name, img_file)
        shutil.copy2(img_path, dst_img_path)
        
        # 保存标签文件
        if yolo_segments:
            label_file = img_file.replace('.jpg', '.txt').replace('.jpeg', '.txt')
            label_path = os.path.join(yolo_dir, 'labels', split_name, label_file)
            with open(label_path, 'w') as f:
                f.write('\n'.join(yolo_segments))

# 处理训练集和验证集
process_split(train_files, 'train')
process_split(val_files, 'val')

# 创建YOLO数据集配置文件
data_yaml = {
    'path': yolo_dir,
    'train': 'images/train',
    'val': 'images/val',
    'nc' : 3, #总共有三个类别
    'names': {
        0: 'R1',  
        1: 'R2_Real', # 目前
        2: 'R2_Fake'
    }
}

yaml_path = os.path.join(yolo_dir, 'dataset.yaml')
with open(yaml_path, 'w') as f:
    yaml.dump(data_yaml, f, default_flow_style=False)

print(f"YOLO格式转换完成！")
print(f"数据集配置: {yaml_path}")
print(f"目录结构:")
print(f"{yolo_dir}/")
print(f"├── images/train/")
print(f"├── images/val/")
print(f"├── labels/train/")
print(f"├── labels/val/")
print(f"└── dataset.yaml")