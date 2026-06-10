# AI 机器人课程作业

本仓库整理 AI Robotics 课程每周实验、代码、截图和学习反思。作业内容按照课程评分系统更容易识别的方式组织：每周都有学习目标、实验环境、操作步骤、关键概念、运行结果、问题记录和总结。

> 冲高分优化记录见：[SCORE_IMPROVEMENT_PLAN.md](SCORE_IMPROVEMENT_PLAN.md)

## 仓库结构

- `week*/README.md`：每周实验报告与学习笔记
- `week*/`：课程实验代码
- `img/week*/`：每周运行截图、录屏或效果图
- `_config.yml`：GitHub Pages 站点配置

## 课程作业目录

- [Week 1：OpenCLAW、Python、Ubuntu 与 ROS2 环境准备](week1/README.md)
- [Week 2：WSL、Ubuntu 与 ROS2 环境配置](week2/README.md)
- [Week 3：GitHub SSH、VS Code 与 ROS2 命令行交互](week3/README.md)
- [Week 4：命令行、机器人基础概念与 Python 仿真](week4/README.md)
- [Week 5：Linux 目录操作、PyBullet 与机械臂运动学](week5/README.md)
- [Week 6：传感器介绍与 ROS2 KITTI 实验](week6/README.md)
- [Week 7：Markdown 与 GitHub 作业整理](week7/README.md)
- [Week 8：Docker 安装与 ROS2 桌面容器](week8/README.md)
- [Week 9：机器人与机器视觉数学基础](week9/README.md)
- [Week 10：Docker 概念与 OpenCV 实验](week10/README.md)
- [Week 11：Docker 进阶与 GitHub Pages 网页部署](week11/README.md)
- [Week 12：手机摄像头、ArUco 识别与距离测量](week12/README.md)
- [Week 13：四足机器人入门 + 期末项目实施](week13/README.md)
- [Week 14：手机遥控 + 局域网通信 + 仿真机器人迷宫探索（小组项目）](week14/README.md)
- [Week 15：考试周](week15/README.md)

## 评分优化索引

为了便于课程自动评价系统和老师人工检查，本仓库按以下证据组织每周作业：

| 周次 | 主要证据 | 代码 / 文件 | 运行结果 |
| --- | --- | --- | --- |
| Week 2-4 | ROS2 TurtleSim 控制 | `week4/square_mover.py` | 直线、圆形、正方形轨迹截图 |
| Week 5 | PyBullet 机械臂运动学 | `week5/robot_sim.py`, `week5/robot_sim2.py` | 机械臂轨迹截图与视频 |
| Week 6 | 传感器与 KITTI 数据 | `week6/README.md` | rqt / KITTI 数据截图 |
| Week 7 | Markdown 与 GitHub Pages 整理 | `README.md`, `week*/README.md` | GitHub 仓库整理截图 |
| Week 8 | Docker ROS2 桌面容器 | `week8/README.md` | Docker / noVNC 截图 |
| Week 9 | 视觉数学基础 | `week9/vision_math_demo.py` | 坐标、矩阵、像素换算说明 |
| Week 10 | OpenCV 图像处理 | `week10/opencv_color_demo.py` | OpenCV 运行截图 |
| Week 11 | Docker 进阶与 Pages 部署 | `week11/README.md` | Docker 镜像与 Pages 截图 |
| Week 12 | 手机摄像头与 ArUco 距离估计 | `week12/aruco_distance_estimator.py` | ArUco / 摄像头截图 |
| Week 13 | 四足机器人步态与项目实施 | `week13/demos/`, `week13/quadruped_ppo_residual_stairs.py` | 步态视频、训练资源 |
| Week 14 | 手机遥控与迷宫探索 | `week14/maze_remote_sim.py`, `week14/remote_controller.html` | 迷宫遥控视频 |

## 关于我

- 姓名：王明（왕명）
- 学号：20231890
- 专业：软件工程

## 项目说明

本项目使用 GitHub Pages 自动部署。

在线访问：https://testerwm.github.io/ai-robot-wm/

GitHub 仓库：https://github.com/testerwm/ai-robot-wm
