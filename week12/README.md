# Week 12 作业记录：手机摄像头、ArUco 识别与距离测量

## 1. 学习目标

- 了解手机摄像头接入电脑或 WSL 的基本思路。
- 使用 OpenCV 识别 ArUco 标记。
- 理解相机标定、像素坐标和实际距离估计之间的关系。

## 2. 课程重点

### 2.1 手机摄像头接入

手机摄像头可以作为临时实验相机，用于采集实时图像。接入后，OpenCV 可以通过视频流或摄像头编号读取画面。

```python
cap = cv2.VideoCapture(0)
ret, frame = cap.read()
```

### 2.2 ArUco 标记识别

ArUco 是一种带 ID 的方形视觉标记。OpenCV 可以检测标记角点和编号：

```python
dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
parameters = cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(dictionary, parameters)
corners, ids, rejected = detector.detectMarkers(frame)
```

### 2.3 距离测量思路

如果已知 ArUco 标记的真实边长，并完成相机标定，就可以利用标记在图像中的大小估计相机到标记的距离。

关键数据：

- 标记真实尺寸
- 相机内参矩阵
- 畸变参数
- 图像中检测到的四个角点

## 3. 实验流程

1. 准备 ArUco 标记并打印或在屏幕显示。
2. 使用手机摄像头采集画面。
3. OpenCV 读取视频帧。
4. 检测 ArUco 角点和 ID。
5. 根据标记尺寸和相机参数估算距离。
6. 在画面上绘制检测框、ID 和距离信息。

## 4. 可复现实验脚本

本周新增脚本：

- [`aruco_distance_estimator.py`](aruco_distance_estimator.py)：读取图片、检测 ArUco、根据标记像素宽度估算距离。

运行命令：

```bash
cd week12
python3 aruco_distance_estimator.py ../img/week12/scan.jpg
```

脚本中保留了距离估算公式：

```text
distance = marker_real_length * focal_length / marker_pixel_width
```

这条公式说明：同一个真实尺寸的标记，在画面中越大，说明距离相机越近；在画面中越小，说明距离越远。

![scan.jpg](../img/week12/scan.jpg)

## 5. 核心理解

- 图像检测得到的是像素坐标，不是直接的真实世界距离。
- 相机标定的作用是建立像素坐标与相机坐标之间的关系。
- ArUco 标记适合作为机器人视觉实验中的定位参考物，因为形状明确、ID 可识别、角点容易检测。
- 如果没有准确相机内参，也可以先用近似焦距做工程估算，再在报告中说明误差来源。

## 6. 问题与解决

- 检测不到标记：检查光照、清晰度、标记是否完整出现在画面中。
- 距离抖动：可以连续多帧取平均，减少噪声。
- 摄像头打不开：确认摄像头权限、设备编号和 WSL/Docker 是否能访问摄像头。
- 估算距离不准：需要用棋盘格或标定板做相机标定，替换脚本中的近似焦距。

## 7. 本周总结

本周把 OpenCV 图像处理推进到视觉测量任务，理解了从“看到标记”到“估计距离”的完整链路。这部分内容可以与后续机器人导航、视觉抓取和目标跟踪结合。
