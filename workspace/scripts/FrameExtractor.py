import cv2
import os

# 视频文件所在目录
video_dir = "/home/li/camera_cxx/workspace/videos"
# 图片保存目录
img_dir = "/home/li/camera_cxx/workspace/images_2F"

# 确保保存目录存在
os.makedirs(img_dir, exist_ok=True)

# 视频列表
video_names = [f"R2_Fake_{i}.mp4" for i in range(15)]  # 0~14

for video_name in video_names:
    video_path = os.path.join(video_dir, video_name)
    if not os.path.isfile(video_path):
        print(f"[WARN] 视频不存在: {video_path}")
        continue

    # 为每个视频创建单独子文件夹
    video_base = os.path.splitext(video_name)[0]
    video_img_dir = os.path.join(img_dir, video_base)
    os.makedirs(video_img_dir, exist_ok=True)

    cap = cv2.VideoCapture(video_path)
    total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    if total_frames == 0:
        print(f"[WARN] 视频无帧数: {video_name}")
        cap.release()
        continue

    step = 20  # 每20帧抽一张
    frame_idx = 0
    saved_count = 0

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        if frame_idx % step == 0:
            frame_name = f"{video_base}_{saved_count:04d}.png"
            frame_path = os.path.join(video_img_dir, frame_name)
            cv2.imwrite(frame_path, frame)
            saved_count += 1

        frame_idx += 1

    cap.release()
    print(f"[INFO] 完成视频 {video_name} 的抽帧，共保存 {saved_count} 张图片")
