# Week 1 作业记录：OpenCLAW、Python、Ubuntu 与 ROS2 环境准备

## 1. 学习目标

- 了解 AI 机器人课程需要的基础开发环境。
- 完成 OpenCLAW、Python、Ubuntu/WSL 与 ROS2 的安装验证。
- 通过 TurtleSim 确认 ROS2 图形化程序可以正常启动。

## 2. 实验环境

- 操作系统：Windows + Ubuntu/WSL
- 编程环境：Python
- 机器人中间件：ROS2
- 辅助工具：OpenCLAW、终端、浏览器

## 3. 操作过程

### 3.1 安装并启动 OpenCLAW

OpenCLAW 是课程中用于 AI 助手与机器人系统连接的工具。安装完成后启动程序，确认界面可以正常运行。

![OpenCLAW 运行截图](../img/week1/openclaw-running.png)

### 3.2 验证 Python 环境

在终端中检查 Python 版本，确认后续 ROS2 脚本和机器人控制程序可以运行。

```bash
python --version
python3 --version
```

![Python 版本截图](../img/week1/python-version.png)

### 3.3 启动 ROS2 TurtleSim

安装 Ubuntu 与 ROS2 后，使用 TurtleSim 做第一次图形化验证。TurtleSim 能启动，说明 ROS2 节点、图形界面和运行环境基本可用。

```bash
ros2 run turtlesim turtlesim_node
```

![ROS2 TurtleSim 启动截图](../img/week1/ros-turtlesim.png)

## 4. 核心理解

- ROS2 是机器人软件系统的通信框架，后续会用节点、话题和消息来控制机器人。
- Python 是本课程主要的控制脚本语言，适合快速编写实验程序。
- 环境搭建是后续实验的基础，如果 Python、Ubuntu 或 ROS2 版本不一致，后面运行节点时容易出错。

## 5. 问题与解决

- 图形程序启动前需要确认 Ubuntu/WSL 或 Docker 图形环境可用。
- 运行 ROS2 命令前需要先确认终端环境已经加载 ROS2 配置。

## 6. 本周总结

本周完成了机器人课程的基础环境准备，并通过 OpenCLAW、Python 版本检查和 TurtleSim 启动截图证明环境可用。下一步可以开始学习 ROS2 节点通信与机器人运动控制。
