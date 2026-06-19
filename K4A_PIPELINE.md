# K4A 检测流水线 — 当前计算逻辑

> 基于 `ros2` 分支 | 2026-06

---

## 一、完整数据流

```
┌─────────────────────────────────────────────────────────────┐
│  K4A 相机 (RGB 1280×720 + Depth NFOV_UNBINNED 640×576)     │
└────────────────────┬────────────────────────────────────────┘
                     ↓
┌─────────────────────────────────────────────────────────────┐
│  ① Image_to_Cv  (src/k4a/camera_k4a.cpp)                   │
│    帧丢弃 → get_capture → depth_to_color 对齐              │
│    输出: color_image(BGR), depth_image(16U,mm) 均为1280×720 │
└────────────────────┬────────────────────────────────────────┘
                     ↓
┌─────────────────────────────────────────────────────────────┐
│  ② YOLOv8 推理 (src/utils/myinfer.cpp)                      │
│    Single_Inference_Letterbox(gray3, detections, 640)       │
│    ┌─ 内部做 letterbox resize: 1280×720 → 640×640          │
│    ├─ 推理 (置信度 0.5, NMS 0.5)                            │
│    ├─ 检测框从 640×640 映射回 1280×720                      │
│    └─ 输出: detections (yolo::BoxArray)                     │
│      class_label=0: block                                   │
│      class_label=1: R1 face                                 │
│      class_label=2~16: R2r faces                            │
│      class_label=17~31: R2f faces                           │
└────────────────────┬────────────────────────────────────────┘
                     ↓
┌─────────────────────────────────────────────────────────────┐
│  ③ BlockRecognizer::recognize  (src/utils/block_recognizer) │
│    ┌─ 按 class_label 拆分为 blocks(0) 和 faces(≠0)         │
│    ├─ 对每个 block:                                         │
│    │   遍历 faces, IoF > 0.7 关联                           │
│    │   收集 face_classes → 分类:                            │
│    │     all label=1          → R1                         │
│    │     has label=2~16       → R2r                        │
│    │     has label=17~31      → R2f                        │
│    │   几何打分 score = area_ratio - 1.5×center_offset     │
│    │   best_pattern = 最高分 face                           │
│    │   所有关联 face → candidates[] 按分降序                │
│    └─ 输出: FinalBlockResults                               │
└────────────────────┬────────────────────────────────────────┘
                     ↓
┌─────────────────────────────────────────────────────────────┐
│  ④ compute_precise_center  (src/k4a/camera_k4a.cpp) ★ 核心  │
│    输入: depth_image, block_box, face_box(best_pattern)     │
│    ┌─ RANSAC 平面拟合 (block 框 ROI, 步长 2 采样)           │
│    │   pcl::SACSegmentation, 2cm 容差, 200 次迭代           │
│    │   输出: 平面方程 A·X + B·Y + C·Z + D = 0              │
│    │         法向量 n_cam = (A,B,C) 归一化                   │
│    ├─ 射线求交 (face 中心像素 → 相机射线 → 平面交点)       │
│    │   u = face_center.x, v = face_center.y                 │
│    │   dx = (u-cx)/fx, dy = (v-cy)/fy, dz = 1              │
│    │   t = -D / (A·dx + B·dy + C·dz)                       │
│    │   P_cam = (t·dx, t·dy, t)                             │
│    └─ 输出: 相机坐标系下的精确 3D 中心 (射线-平面交点)     │
└────────────────────┬────────────────────────────────────────┘
                     ↓
┌─────────────────────────────────────────────────────────────┐
│  ⑤ 法向量校验 (主循环, k4a_detect.cpp)                      │
│    ┌─ P_robot = R × P_cam + T                               │
│    ├─ n_robot = R × n_cam                                   │
│    ├─ dir_to_robot = -P_robot.normalized()                  │
│    ├─ dot = |n_robot · dir_to_robot|                        │
│    ├─ if dot ≥ 0.5 → 接受, 作为卡尔曼观测值                 │
│    └─ if dot < 0.5 → 零值丢弃（面不正对机器人）             │
└────────────────────┬────────────────────────────────────────┘
                     ↓
┌─────────────────────────────────────────────────────────────┐
│  ⑥ 卡尔曼滤波 + 航迹管理  (src/utils/kalman_tracker)        │
│    状态: [cx, cy, cz, vx, vy, vz]                          │
│    ┌─ Predict (恒速度 + IMU 角速度旋转速度向量)             │
│    ├─ 有效观测 → Update (马氏距离门限 3.0)                 │
│    ├─ 无效观测 → 只 Predict                                 │
│    ├─ 连续 3 帧无 Update → 删除 track                       │
│    ├─ 多个 track → 选距离机器人最近且本帧有 update 的        │
│    └─ 输出: 平滑后的 (x, y, z)                             │
└────────────────────┬────────────────────────────────────────┘
                     ↓
┌─────────────────────────────────────────────────────────────┐
│  ⑦ ROS 消息发布                                             │
│    Topic: /k4a/target_info                                  │
│    格式: "cls_ID, x, y, z, yaw"                            │
│    cls_ID ∈ {0=UNKNOWN, 1=R1, 2=R2r, 3=R2f}                │
│    无检测 / 法向量校验失败 → "0,0,0,0,0"                    │
└─────────────────────────────────────────────────────────────┘
```

---

## 二、坐标系关系

### 2.1 旋转平移 (来自 K4AConfig.yaml)

```
P_robot = R × P_camera + T

R = [-0.085757, -0.104509,  0.990819
      0.0,       0.994483,  0.104895
     -0.996316,  0.008995, -0.085284]

T = [0.32521, -0.31991, 0.62596]  单位: 米
```

### 2.2 相机内参 (来自 K4A 标定, 1280×720)

```
u = fx × X/Z + cx
v = fy × Y/Z + cy
```

---

## 三、核心算法详解

### 3.1 `compute_precise_center` — RANSAC + 射线求交

```
输入: depth_image(16U,mm), block_box, face_box
           ↓
    从 block_box ROI 采集点云 (步长 2)
           ↓
    RANSAC 平面拟合 (pcl::SACMODEL_PLANE)
     - 距离阈值: 2cm
     - 最大迭代: 200
     - 最少内点: 10
           ↓
    输出: 平面系数 A,B,C,D    法向量 n_cam = (A,B,C)
           ↓
    从 face_box 中心像素发出一条射线:
      dx = (u - cx) / fx
      dy = (v - cy) / fy
      dz = 1.0
           ↓
    射线与平面求交:
      t = -D / (A·dx + B·dy + C·dz)
      P_cam = (t·dx, t·dy, t)
           ↓
    输出: 相机坐标系下的精确 3D 中心
```

**核心优点**：3D 中心由 **Block 框拟合的平面** 和 **Face 框中心的射线** 联合决定，不依赖 Face 框内的深度值。只要 Block 框住了物体（即便稍大），Face 框中心在物体表面上，结果就稳定。

### 3.2 `Single_Inference_Letterbox` — 降采样推理

```
输入: 1280×720 gray3
       ↓
    计算 letterbox 参数:
      scale = min(640/1280, 640/720) = 0.5
      resize 到 640×360
      pad 上 + 下 = 140px → 640×640
       ↓
    YOLO 推理 (640×640)
       ↓
    检测框坐标逆映射回 1280×720:
      x_orig = (x_lb - pad_left) / scale
      y_orig = (y_lb - pad_top) / scale
       ↓
    输出: 原始 1280×720 坐标下的 detections
```

### 3.3 `KalmanFilter3D`

| 参数 | 值 | 说明 |
|------|-----|------|
| 状态 | 6维 | (x,y,z,vx,vy,vz) 机器人系 |
| 观测噪声 R | diag(0.01, 0.01, 0.04) | z(深度)噪声更大 |
| 过程噪声 Q | pos:0.01, vel:0.05 | 允许速度变化 |
| 马氏距离门限 | 3.0 | 观测关联匹配 |
| 最大丢失帧数 | 3 | 3帧无观测即删 |
| IMU | 陀螺仪角速度 | predict 中旋转速度向量 |

---

## 四、输出格式

### ROS Topic: `/k4a/target_info`

```  
每帧: "cls_ID, center.x, center.y, center.z, yaw"

cls_ID:
  0 = UNKNOWN (无检测 / 法向量校验失败)
  1 = R1
  2 = R2r (R2_Real)
  3 = R2f (R2_Fake)

无检测 / 被拒绝: "0,0,0,0,0"
```

### 终端日志 (每 5 帧):

```
Block Class: R2_Fake Confidence: 0.934
Center: [0.821, -0.061, 0.025, -0.074]
Time: 0.312s
```

### 辅助日志:

```
[NORMAL] dot=0.32 < 0.5, reject    ← 法向量校验未通过
```

---

## 五、文件职责

| 文件 | 职责 |
|------|------|
| `src/k4a/k4a_detect.cpp` | 主循环: 编排各模块调用, 卡尔曼滤波, ROS发布 |
| `src/k4a/camera_k4a.cpp` | K4a 设备管理, `compute_precise_center` (RANSAC+射线) |
| `src/utils/myinfer.cpp` | YOLO 推理封装, `Single_Inference_Letterbox` (降采样+映射) |
| `src/utils/block_recognizer.cpp` | block + face 融合分类 |
| `src/utils/kalman_tracker.cpp` | 卡尔曼滤波 + 航迹管理 |
