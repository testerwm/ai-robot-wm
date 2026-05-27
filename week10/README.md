# Week 10 作业记录：Docker 概念与 OpenCV 实验

## 1. 学习目标

- 在 Docker/WSL 环境中配置 Python OpenCV。
- 使用 OpenCV 读取图片并理解 BGR、RGB、GRAY 色彩空间。
- 解决依赖安装和图像显示过程中的常见问题。

## 2. 实验环境

- Python 3
- OpenCV：`opencv-python`
- Matplotlib：用于显示图片
- 运行环境：WSL Ubuntu 或 Docker ROS2 容器

## 3. 操作过程

### 3.1 安装依赖

```bash
pip3 install opencv-python matplotlib
```

如果出现系统环境限制，可以根据课程环境使用：

```bash
pip3 install opencv-python matplotlib --break-system-packages
```

### 3.2 读取图片

OpenCV 使用 `cv2.imread()` 读取图片，默认通道顺序是 BGR。

```python
import cv2
import matplotlib.pyplot as plt

img = cv2.imread("image.png")
rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
plt.imshow(rgb)
plt.show()
```

### 3.3 灰度化处理

```python
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
plt.imshow(gray, cmap="gray")
plt.show()
```

## 4. 核心理解

- OpenCV 默认使用 BGR，而 Matplotlib 默认按 RGB 显示，所以直接显示会出现颜色异常。
- 灰度化会把三通道彩色图转换成单通道亮度图，适合边缘检测、阈值分割等任务。
- Docker 环境可以固定依赖版本，让视觉实验更容易复现。

## 5. 问题与解决

- `cv2.imread()` 返回 `None`：检查图片路径和文件名。
- 显示颜色不正确：使用 `cv2.cvtColor(img, cv2.COLOR_BGR2RGB)`。
- 依赖冲突：记录安装命令和版本，必要时使用虚拟环境或容器隔离。

## 6. 本周总结

本周完成了 OpenCV 基础图像处理流程，理解了图像读取、颜色通道转换和灰度化处理，为后续 ArUco 识别实验做准备。
