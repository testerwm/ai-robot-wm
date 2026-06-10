#!/usr/bin/env python3
"""Week 9 robot vision math mini demo.

This script keeps the Week 9 theory executable: it shows how image pixels,
camera intrinsics, and a simple 2D robot pose transform relate to each other.
It uses only the Python standard library so it can run in a minimal WSL setup.
"""

from __future__ import annotations

import math


def pixel_to_normalized(u: float, v: float, fx: float, fy: float, cx: float, cy: float) -> tuple[float, float]:
    """Convert image pixel coordinates to normalized camera coordinates."""
    x = (u - cx) / fx
    y = (v - cy) / fy
    return x, y


def camera_ray_to_point(x: float, y: float, depth_m: float) -> tuple[float, float, float]:
    """Project a normalized camera ray to a 3D point at a known depth."""
    return x * depth_m, y * depth_m, depth_m


def robot_to_world(px: float, py: float, robot_x: float, robot_y: float, yaw_rad: float) -> tuple[float, float]:
    """Transform a 2D point from robot frame to world frame."""
    cos_yaw = math.cos(yaw_rad)
    sin_yaw = math.sin(yaw_rad)
    wx = robot_x + cos_yaw * px - sin_yaw * py
    wy = robot_y + sin_yaw * px + cos_yaw * py
    return wx, wy


def main() -> None:
    # Example camera parameters: 640x480 image, focal length about 600 pixels.
    fx = fy = 600.0
    cx, cy = 320.0, 240.0
    u, v = 380.0, 210.0
    depth_m = 1.5

    norm_x, norm_y = pixel_to_normalized(u, v, fx, fy, cx, cy)
    cam_x, cam_y, cam_z = camera_ray_to_point(norm_x, norm_y, depth_m)
    world_x, world_y = robot_to_world(cam_x, cam_y, robot_x=2.0, robot_y=1.0, yaw_rad=math.radians(30))

    print("Week 9 vision math demo")
    print(f"pixel: ({u:.0f}, {v:.0f})")
    print(f"normalized camera coordinate: ({norm_x:.3f}, {norm_y:.3f})")
    print(f"camera point at {depth_m:.1f}m: ({cam_x:.3f}, {cam_y:.3f}, {cam_z:.3f})")
    print(f"world point with robot pose (2.0, 1.0, 30deg): ({world_x:.3f}, {world_y:.3f})")


if __name__ == "__main__":
    main()
