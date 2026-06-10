# Week 4 作业记录：命令行、机器人基础概念与 Python 仿真

## 1. 学习目标

- 在程序中理解二维平面坐标系与机器人姿态。
- 编写 ROS2 Python 节点，向 `/turtle1/cmd_vel` 发布速度命令。
- 控制 TurtleSim 按“直行 + 旋转”的方式走出正方形。

## 2. 实验文件

- [`square_mover.py`](square_mover.py)：TurtleSim 正方形运动控制脚本

## 3. 程序设计

程序将正方形拆成四组动作：

1. 直行一段时间
2. 停止
3. 原地旋转 90 度
4. 重复 4 次

关键参数：

```python
self.SPEED = 1.0
self.TURN_SPEED = 1.0
self.SIDE_LENGTH = 2.0
self.MOVE_TIME = self.SIDE_LENGTH / self.SPEED
self.TURN_TIME = 1.5708 / self.TURN_SPEED
```

其中 `1.5708` 约等于 `pi / 2`，表示 90 度。

## 4. 运行步骤

先启动 TurtleSim：

```bash
ros2 run turtlesim turtlesim_node
```

再运行控制脚本：

```bash
python3 square_mover.py
```

## 5. 实验结果

程序运行后，小乌龟按正方形轨迹运动。截图记录了机器人姿态变化和轨迹结果。

![TurtleSim 正方形控制](../img/week4/bullet.png)

## 6. 核心理解

- 坐标系帮助我们描述机器人“在哪里”和“朝向哪里”。
- 正方形轨迹可以由简单动作组合出来，不一定要一次性计算完整路径。
- 时间控制方法简单直观，但误差会累积；更精确的方式应结合里程计反馈。

## 7. 本周总结

本周通过代码把坐标系、角速度和机器人轨迹联系起来，完成了一个可复现的 ROS2 控制节点。
