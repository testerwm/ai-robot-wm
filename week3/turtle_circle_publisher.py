#!/usr/bin/env python3
"""Week 3 ROS2 TurtleSim circle publisher."""

import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class TurtleCirclePublisher(Node):
    def __init__(self):
        super().__init__("week3_turtle_circle_publisher")
        self.publisher = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)

    def draw_circle(self, linear=1.6, angular=1.2, seconds=8.0):
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        start = time.time()
        while time.time() - start < seconds:
            self.publisher.publish(msg)
            time.sleep(0.05)
        self.publisher.publish(Twist())


def main():
    rclpy.init()
    node = TurtleCirclePublisher()
    time.sleep(0.5)
    node.draw_circle()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
