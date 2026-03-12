#! conda 环境有 opencv 下运行
"""
preprocess_and_augment.py

说明：
- 遍历 images 目录下的 R2_KFS_0 到 R2_KFS_14 共15个文件夹
- 读取彩色图片，转为灰度图（3通道）进行训练
- 每张原图生成10个增强版本
- 输出到 dataset_aug/images_aug 和 dataset_aug/masks_aug
"""

import os, glob, random, cv2
import numpy as np

random.seed(42)
np.random.seed(42)

# 修改输入目录结构
images_base_dir = "/home/li/camera_cxx/workspace/images"  # 包含 R2_KFS_0 到 R2_KFS_14 文件夹
masks_dir = "/home/li/camera_cxx/workspace/masks"  # 原始 mask，像素值 2 表示目标

# 输出目录
dataset_aug_dir = "/home/li/camera_cxx/workspace/dataset_aug"
images_out_dir = os.path.join(dataset_aug_dir, "images_aug")
masks_out_dir = os.path.join(dataset_aug_dir, "masks_aug")

# 创建输出目录
os.makedirs(dataset_aug_dir, exist_ok=True)
os.makedirs(images_out_dir, exist_ok=True)
os.makedirs(masks_out_dir, exist_ok=True)

# 参数：每张原图生成多少增强样本（不包含原图）
AUG_PER_IMAGE = 10

def to_gray_3ch(img):
    """将彩色图转为3通道灰度图"""
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    gray3 = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
    return gray3

def random_brightness_contrast(img, brightness_range=0.3, contrast_range=0.3):
    """随机调整亮度和对比度"""
    b = (np.random.rand() * 2 - 1) * brightness_range * 255
    c = 1.0 + (np.random.rand() * 2 - 1) * contrast_range
    out = img.astype(np.float32) * c + b
    out = np.clip(out, 0, 255).astype(np.uint8)
    return out

def gaussian_blur(img, ksize=(5,5)):
    """高斯模糊"""
    return cv2.GaussianBlur(img, ksize, 0)

def random_flip(img, mask):
    """随机翻转"""
    r = random.choice([0,1,2,3])  # 0 no, 1 horizontal, 2 vertical, 3 both
    if r == 1:
        return cv2.flip(img, 1), cv2.flip(mask, 1)
    elif r == 2:
        return cv2.flip(img, 0), cv2.flip(mask, 0)
    elif r == 3:
        return cv2.flip(img, -1), cv2.flip(mask, -1)
    return img, mask

def random_rotate(img, mask, max_angle=20):
    """随机旋转"""
    angle = (np.random.rand() * 2 - 1) * max_angle
    h, w = img.shape[:2]
    M = cv2.getRotationMatrix2D((w/2, h/2), angle, 1.0)
    img_r = cv2.warpAffine(img, M, (w,h), flags=cv2.INTER_LINEAR, borderMode=cv2.BORDER_REFLECT)
    mask_r = cv2.warpAffine(mask, M, (w,h), flags=cv2.INTER_NEAREST, borderMode=cv2.BORDER_CONSTANT, borderValue=0)
    return img_r, mask_r

def random_crop(img, mask, min_area_ratio=0.6):
    """随机裁剪并缩放回原尺寸"""
    h, w = img.shape[:2]
    for _ in range(5):
        scale = np.random.uniform(min_area_ratio, 1.0)
        new_h = int(h * scale)
        new_w = int(w * scale)
        if new_h < 2 or new_w < 2:
            continue
        x = np.random.randint(0, w - new_w + 1)
        y = np.random.randint(0, h - new_h + 1)
        img_c = img[y:y+new_h, x:x+new_w]
        mask_c = mask[y:y+new_h, x:x+new_w]
        # resize back to original
        img_r = cv2.resize(img_c, (w,h), interpolation=cv2.INTER_LINEAR)
        mask_r = cv2.resize(mask_c, (w,h), interpolation=cv2.INTER_NEAREST)
        return img_r, mask_r
    return img, mask

def add_gaussian_noise(img, mean=0, sigma=25):
    """添加高斯噪声"""
    noise = np.random.normal(mean, sigma, img.shape).astype(np.float32)
    noisy_img = img.astype(np.float32) + noise
    return np.clip(noisy_img, 0, 255).astype(np.uint8)

def save_pair(img, mask, dst_img_path, dst_mask_path):
    """保存图像和mask对"""
    # save image as 3-channel jpg
    cv2.imwrite(dst_img_path, img)
    # ensure mask is single-channel uint8
    if len(mask.shape) == 3:
        mask = cv2.cvtColor(mask, cv2.COLOR_BGR2GRAY)
    cv2.imwrite(dst_mask_path, mask)

# 主流程
# 查找所有 R2_KFS_ 开头的文件夹
sub_folders = []
for i in range(15):  # R2_KFS_0 到 R2_KFS_14
    folder_name = f"R2_KFS_{i}"
    folder_path = os.path.join(images_base_dir, folder_name)
    if os.path.exists(folder_path):
        sub_folders.append(folder_path)

print(f"找到 {len(sub_folders)} 个图片文件夹")

all_img_paths = []

# 遍历所有子文件夹收集图片路径
for folder in sub_folders:
    folder_name = os.path.basename(folder)
    img_paths = glob.glob(os.path.join(folder, "*.*"))
    img_paths = [p for p in img_paths if os.path.basename(p).lower().endswith((".jpg",".jpeg",".png"))]
    
    print(f" {folder_name}: {len(img_paths)} 张图片")
    all_img_paths.extend(img_paths)

all_img_paths.sort()
print(f"\n 总共找到 {len(all_img_paths)} 张原始图片")

total_augmented = 0

#保存文件名设置
for p in all_img_paths:
    # 修改这部分
    file_name = os.path.splitext(os.path.basename(p))[0]  # 直接就是 R2_KFS_0_00
    name = file_name  # 直接用文件名


    
    # 查找对应的mask文件
    mask_path = os.path.join(masks_dir, f"{file_name}.png")  # mask文件名与图片文件名相同
    

    if not os.path.exists(mask_path):
        print(f"[WARN] mask not found for {p}")
        continue

    # 读取彩色图片
    img = cv2.imread(p)
    mask = cv2.imread(mask_path, cv2.IMREAD_UNCHANGED)
    if mask is None:
        print(f"[WARN] failed to load mask: {mask_path}")
        continue

    # 确保mask为uint8类型
    if mask.dtype != np.uint8:
        mask = mask.astype(np.uint8)

    # 将原图转为灰度图并保存
    base_img = to_gray_3ch(img)  # 灰度化并扩回3通道
    base_mask = mask.copy()  # mask保持不变
    
    # 保存原图（灰度版本）
    save_pair(base_img, base_mask,
              os.path.join(images_out_dir, f"{name}.jpg"),
              os.path.join(masks_out_dir, f"{name}.png"))

    # 生成增强样本
    for i in range(AUG_PER_IMAGE):
        aug_img = base_img.copy()
        aug_mask = base_mask.copy()

        # 应用多种增强技术（按概率）
        augmentations_applied = []
        
        # 1. 随机翻转 (50%概率)
        if random.random() < 0.5:
            aug_img, aug_mask = random_flip(aug_img, aug_mask)
            augmentations_applied.append("flip")

        # 2. 随机旋转 (50%概率)
        if random.random() < 0.5:
            aug_img, aug_mask = random_rotate(aug_img, aug_mask, max_angle=15)
            augmentations_applied.append("rotate")

        # 3. 随机裁剪 (40%概率)
        if random.random() < 0.4:
            aug_img, aug_mask = random_crop(aug_img, aug_mask, min_area_ratio=0.7)
            augmentations_applied.append("crop")

        # 4. 亮度和对比度调整 (80%概率)
        if random.random() < 0.8:
            aug_img = random_brightness_contrast(aug_img, brightness_range=0.2, contrast_range=0.2)
            augmentations_applied.append("brightness")

        # 5. 高斯模糊 (30%概率)
        if random.random() < 0.3:
            k = random.choice([3,5,7])
            aug_img = gaussian_blur(aug_img, (k,k))
            augmentations_applied.append("blur")

        # 6. 高斯噪声 (20%概率)
        if random.random() < 0.2:
            aug_img = add_gaussian_noise(aug_img, sigma=15)
            augmentations_applied.append("noise")

        # 保存增强后的图像对
        out_name = f"{name}_aug{i}"
        save_pair(aug_img, aug_mask,
                  os.path.join(images_out_dir, f"{out_name}.jpg"),
                  os.path.join(masks_out_dir, f"{out_name}.png"))
        
        total_augmented += 1

    print(f"完成: {name} -> 原图 + {AUG_PER_IMAGE}个增强样本")

print(f"\n🎉 数据增强完成!")
print(f"原始图片: {len(all_img_paths)} 张")
print(f"增强样本: {total_augmented} 张") 
print(f"总计: {len(all_img_paths) + total_augmented} 张")
print(f"数据集目录: {dataset_aug_dir}")
print(f"  ├── images_aug/")
print(f"  └── masks_aug/")