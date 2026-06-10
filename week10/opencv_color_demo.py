#!/usr/bin/env python3
"""Week 10 OpenCV color-space experiment.

The script reads an image, converts BGR to RGB and GRAY, and saves a side-by-side
result. It is written so the same file can be run locally, in WSL, or inside a
Docker ROS2 desktop container.
"""

from __future__ import annotations

from pathlib import Path


def main() -> None:
    try:
        import cv2
        import matplotlib.pyplot as plt
    except ImportError as exc:
        raise SystemExit("Install dependencies first: pip3 install opencv-python matplotlib") from exc

    root = Path(__file__).resolve().parents[1]
    image_path = root / "img" / "week10" / "open_cv.png"
    output_path = Path(__file__).with_name("opencv_color_result.png")

    img_bgr = cv2.imread(str(image_path))
    if img_bgr is None:
        raise SystemExit(f"Could not read image: {image_path}")

    img_rgb = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB)
    img_gray = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2GRAY)

    fig, axes = plt.subplots(1, 3, figsize=(12, 4))
    axes[0].imshow(img_bgr)
    axes[0].set_title("Raw BGR shown as RGB")
    axes[1].imshow(img_rgb)
    axes[1].set_title("BGR -> RGB")
    axes[2].imshow(img_gray, cmap="gray")
    axes[2].set_title("BGR -> GRAY")

    for ax in axes:
        ax.axis("off")

    fig.tight_layout()
    fig.savefig(output_path, dpi=160)
    print(f"Saved result to {output_path}")


if __name__ == "__main__":
    main()
