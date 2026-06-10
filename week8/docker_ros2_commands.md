# Week 8 Docker + ROS2 常用命令记录

## 1. 启动 ROS2 桌面容器

```bash
docker run -p 6080:80 \
  --security-opt seccomp=unconfined \
  --shm-size=512m \
  ghcr.io/tiryoh/ros2-desktop-vnc:humble
```

## 2. 检查容器

```bash
docker ps
docker images
docker stats
```

## 3. 进入容器

```bash
docker exec -it <container_id> bash
```

## 4. 运行 ROS2 验证

```bash
ros2 run turtlesim turtlesim_node
ros2 topic list
ros2 node list
```

## 5. 保存环境

```bash
docker commit <container_id> ai-robot-ros2:week8
```

## 6. 记录理解

- 镜像是环境模板。
- 容器是镜像运行起来的实例。
- 端口映射让浏览器能访问容器里的 noVNC 桌面。
- 共享内存参数可以减少图形程序异常。
