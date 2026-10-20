"""Detect ArUco markers in one local image without ROS topics."""

import argparse
import json
from pathlib import Path

import cv2

from .aruco_core import SUPPORTED_DICTIONARIES, annotate_image, detect_markers


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Detect ArUco markers in a static image.")
    parser.add_argument("image")
    parser.add_argument(
        "--dictionary",
        default="DICT_4X4_50",
        choices=sorted(SUPPORTED_DICTIONARIES),
    )
    parser.add_argument("--target-id", type=int, default=-1)
    parser.add_argument("--output", default="output/detected.png")
    return parser


def main(args=None) -> None:
    parsed = build_parser().parse_args(args)
    image_path = Path(parsed.image).expanduser().resolve()
    image = cv2.imread(str(image_path), cv2.IMREAD_COLOR)
    if image is None:
        raise FileNotFoundError(f"Could not read image: {image_path}")
    observations, _ = detect_markers(image, parsed.dictionary, parsed.target_id)
    output = Path(parsed.output).expanduser().resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    if not cv2.imwrite(str(output), annotate_image(image, observations)):
        raise RuntimeError(f"Failed to write annotated image to {output}")
    payload = {
        "detected": bool(observations),
        "count": len(observations),
        "ids": [observation.marker_id for observation in observations],
        "centers_px": [list(observation.center_px) for observation in observations],
        "output": str(output),
    }
    print(json.dumps(payload, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
