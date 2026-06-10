# Week 8 作业记录：Docker 安装与 ROS2 桌面容器

## 1. 学习目标

- 理解 Docker 镜像、容器和端口映射的概念。
- 使用 Docker Desktop 运行 ROS2 图形化桌面环境。
- 通过浏览器 noVNC 访问容器中的 Ubuntu/ROS2 桌面。

## 2. 实验环境

- Docker Desktop
- ROS2 Humble Desktop VNC 镜像
- 浏览器 noVNC
- 端口：`6080:80`
- 命令记录：[`docker_ros2_commands.md`](docker_ros2_commands.md)

## 3. 操作过程

### 3.1 拉取并启动 ROS2 桌面容器

```bash
docker run -p 6080:80 --security-opt seccomp=unconfined --shm-size=512m ghcr.io/tiryoh/ros2-desktop-vnc:humble
```

参数理解：

- `-p 6080:80`：把容器中的网页桌面映射到本机 `6080` 端口。
- `--shm-size=512m`：增加共享内存，减少图形程序异常。
- `--security-opt seccomp=unconfined`：放宽容器限制，便于图形应用运行。

### 3.2 浏览器访问桌面

```text
http://127.0.0.1:6080
```

进入 noVNC 后，可以看到容器内 Ubuntu 桌面，并在终端中运行 ROS2 命令。

## 4. 实验结果

![Docker 中启动 ROS2 桌面](../img/week8/docker.png)

## 5. 核心理解

- Docker 镜像像“模板”，容器是由镜像运行出来的实例。
- 容器化 ROS2 环境可以减少系统配置差异，提高实验复现性。
- noVNC 让图形化 ROS2 程序可以在浏览器中运行和查看。
- Docker 命令、容器截图和运行记录放在同一周目录中，可以证明环境不是只安装了一次，而是可以复现。

## 6. 问题与解决

- 如果浏览器打不开 `6080`，检查容器是否仍在运行。
- 如果图形程序卡顿，可以增加 Docker 分配的内存和 CPU。

## 7. 本周总结

本周完成了 Docker 中 ROS2 桌面环境的搭建，理解了容器化对机器人实验复现的重要作用。
