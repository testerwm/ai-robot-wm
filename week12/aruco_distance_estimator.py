#!/usr/bin/env python3
"""Week 12 ArUco marker detection and distance estimation.

Usage:
    python3 aruco_distance_estimator.py ../img/week12/scan.jpg

If OpenCV can detect an ArUco marker, the script estimates distance from marker
size. If the image has no marker, it still documents the exact processing path
and exits cleanly so the experiment is reproducible.
"""

from __future__ import annotations

import sys
from pathlib import Path


MARKER_LENGTH_M = 0.05
FOCAL_LENGTH_PX = 700.0


def estimate_distance(marker_pixel_width: float) -> float:
    return (MARKER_LENGTH_M * FOCAL_LENGTH_PX) / marker_pixel_width


def main() -> None:
    try:
        import cv2
    except ImportError as exc:
        raise SystemExit("Install OpenCV first: pip3 install opencv-contrib-python") from exc

    image_path = Path(sys.argv[1]) if len(sys.argv) > 1 else Path("../img/week12/scan.jpg")
    image = cv2.imread(str(image_path))
    if image is None:
        raise SystemExit(f"Could not read image: {image_path}")

    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    parameters = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(dictionary, parameters)
    corners, ids, _ = detector.detectMarkers(gray)

    if ids is None:
        print("No ArUco marker detected. Check marker visibility, lighting, and focus.")
        return

    for marker_corners, marker_id in zip(corners, ids.flatten()):
        pts = marker_corners.reshape(4, 2)
        width_px = ((pts[0][0] - pts[1][0]) ** 2 + (pts[0][1] - pts[1][1]) ** 2) ** 0.5
        distance_m = estimate_distance(width_px)
        print(f"marker id={marker_id}, width={width_px:.1f}px, estimated distance={distance_m:.2f}m")


if __name__ == "__main__":
    main()
