# Week 2 作业记录：WSL、Ubuntu 与 ROS2 环境配置

## 1. 学习目标

- 配置 GitHub SSH，理解本地仓库与远程仓库的关系。
- 使用 VS Code 连接 WSL Ubuntu，提高代码编辑与命令行操作效率。
- 运行 ROS2 TurtleSim，并通过速度话题控制小乌龟直线运动。

## 2. 实验环境

- Windows + WSL Ubuntu
- VS Code Remote WSL
- ROS2 TurtleSim
- Git 与 GitHub

## 3. 操作过程

### 3.1 GitHub SSH 配置

在 Ubuntu 终端中生成 SSH key，并将公钥添加到 GitHub。这样以后可以用 SSH 方式 `git pull`、`git push`，不需要每次输入账号密码。

```bash
ssh-keygen -t ed25519 -C "github"
ssh -T git@github.com
```

### 3.2 VS Code 与 WSL 连接

使用 VS Code 打开 WSL 中的项目目录，代码编辑在 VS Code 中完成，ROS2 命令在 Ubuntu 终端中执行。这样可以把图形编辑和 Linux 命令行结合起来。

### 3.3 TurtleSim 直线运动

先启动小乌龟仿真节点：

```bash
ros2 run turtlesim turtlesim_node
```

再通过键盘控制或速度命令让小乌龟向前运动。核心控制消息是 `geometry_msgs/Twist`，其中：

- `linear.x` 控制前进速度
- `angular.z` 控制旋转速度

运行结果如下：

![TurtleSim 直线运动](../img/week2/draw_line.png)

## 4. 核心理解

- ROS2 的节点之间通过 Topic 通信，控制小乌龟时本质上是在向 `/turtle1/cmd_vel` 发布速度消息。
- VS Code + WSL 的组合适合机器人课程实验：代码管理清楚，命令运行环境也接近真实 Linux。
- GitHub SSH 配置成功后，课程作业可以稳定同步到远程仓库。

## 5. 问题与解决

- 如果 `ssh -T git@github.com` 失败，需要检查公钥是否复制完整。
- 如果 TurtleSim 没有反应，需要确认控制命令发布到正确话题 `/turtle1/cmd_vel`。

## 6. 本周总结

本周完成了 GitHub、VS Code、WSL 与 ROS2 的基础联动，并通过 TurtleSim 直线运动验证了 ROS2 控制链路。
