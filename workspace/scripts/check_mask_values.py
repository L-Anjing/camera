# check_mask_values_v2.py
import glob, os
import cv2
import numpy as np
from collections import Counter, defaultdict

mask_dir = "/home/li/camera_cxx/workspace/dataset_aug/masks_aug"  # 改成你的目录
mask_paths = glob.glob(os.path.join(mask_dir, "*.png"))
mask_paths.sort()

if not mask_paths:
    print(f"❌ 没找到任何 mask 文件，请检查路径: {mask_dir}")
    exit()

# 全局统计
global_counter = Counter()
missing_2 = []

for p in mask_paths:
    m = cv2.imread(p, cv2.IMREAD_UNCHANGED)
    if m is None:
        print(f"❌ 读取失败: {p}")
        continue

    # 若为三通道，则取第一个通道或转灰度
    if m.ndim == 3:
        if (m[:,:,0]==m[:,:,1]).all() and (m[:,:,0]==m[:,:,2]).all():
            arr = m[:,:,0]
        else:
            arr = cv2.cvtColor(m, cv2.COLOR_BGR2GRAY)
    else:
        arr = m

    vals, cnts = np.unique(arr, return_counts=True)
    d = dict(zip(vals.tolist(), cnts.tolist()))

    # 记录全局统计
    global_counter.update(d)

    # 输出每张图的像素分布
    val_str = ", ".join(f"{k}:{v}" for k,v in sorted(d.items()))
    print(f"📄 {os.path.basename(p)} -> {val_str}")

    # 检查是否包含像素值2
    if 2 not in d:
        missing_2.append(os.path.basename(p))

# 全局汇总结果
print("\n================= 总结 =================")
print("全局像素值分布：")
for k,v in sorted(global_counter.items()):
    print(f"  类别 {k}: {v} 像素")

if missing_2:
    print("\n⚠️ 下列文件未包含像素值 2：")
    for f in missing_2:
        print("  -", f)
else:
    print("\n✅ 所有 mask 均包含像素值 2！")
print("========================================\n")
