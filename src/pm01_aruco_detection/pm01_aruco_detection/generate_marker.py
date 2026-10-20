"""Command-line ArUco marker generator."""

import argparse
from pathlib import Path

import cv2
import numpy as np

from .aruco_core import SUPPORTED_DICTIONARIES, generate_marker_image


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Generate a non-rescaled ArUco marker PNG.")
    parser.add_argument(
        "--dictionary",
        default="DICT_4X4_50",
        choices=sorted(SUPPORTED_DICTIONARIES),
    )
    parser.add_argument("--id", type=int, default=23, dest="marker_id")
    parser.add_argument("--pixels", type=int, default=1000)
    parser.add_argument("--margin-pixels", type=int, default=100)
    parser.add_argument("--output", default="output/aruco_4x4_50_id23.png")
    return parser


def main(args=None) -> None:
    parsed = build_parser().parse_args(args)
    output = Path(parsed.output).expanduser().resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    marker = generate_marker_image(parsed.dictionary, parsed.marker_id, parsed.pixels)
    if parsed.margin_pixels < 0:
        raise ValueError("margin-pixels must be non-negative")
    canvas_size = parsed.pixels + (2 * parsed.margin_pixels)
    canvas = np.full((canvas_size, canvas_size), 255, dtype=np.uint8)
    start = parsed.margin_pixels
    canvas[start:start + parsed.pixels, start:start + parsed.pixels] = marker
    if not cv2.imwrite(str(output), canvas):
        raise RuntimeError(f"Failed to write marker image to {output}")
    print(f"marker={parsed.dictionary}:{parsed.marker_id}")
    print(f"marker_pixels={parsed.pixels}x{parsed.pixels}")
    print(f"image_pixels={canvas_size}x{canvas_size}")
    print(f"output={output}")
    print("Print at 100% scale and measure the black marker edge before pose testing.")


if __name__ == "__main__":
    main()
