#!/usr/bin/env python3
"""Week 2 ROS2 TurtleSim straight-line publisher.

Run after starting turtlesim:
    ros2 run turtlesim turtlesim_node
    python3 turtle_line_publisher.py
"""

import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class TurtleLinePublisher(Node):
    def __init__(self):
        super().__init__("week2_turtle_line_publisher")
        self.publisher = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)

    def move_forward(self, speed=1.5, seconds=3.0):
        msg = Twist()
        msg.linear.x = speed
        msg.angular.z = 0.0
        start = time.time()
        while time.time() - start < seconds:
            self.publisher.publish(msg)
            time.sleep(0.05)
        self.publisher.publish(Twist())


def main():
    rclpy.init()
    node = TurtleLinePublisher()
    time.sleep(0.5)
    node.move_forward()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
