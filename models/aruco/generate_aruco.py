#!/usr/bin/env python3
"""
Generate ArUco marker PNGs for Gazebo model textures.

Example:
  python3 models/aruco/generate_aruco_texture.py \
    --dict DICT_4X4_50 --id 0 \
    --size 512 \
    --out models/aruco/DICT_4X4_50/marker_0/materials/textures/marker_0.png
"""

import argparse
import os
import cv2
import sys

DICT_MAP = {
    "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
    "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
    "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
    "DICT_6X6_250": cv2.aruco.DICT_6X6_250,
    # add more mappings as needed
}

def generate_marker(dict_name: str, marker_id: int, size: int, out_path: str):
    if dict_name not in DICT_MAP:
        raise ValueError(f"Unsupported dictionary: {dict_name}")
    aruco_dict = cv2.aruco.getPredefinedDictionary(DICT_MAP[dict_name])
    marker = cv2.aruco.drawMarker(aruco_dict, marker_id, size)
    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    if not cv2.imwrite(out_path, marker):
        raise RuntimeError(f"Failed to write image to {out_path}")
    print(f"Saved {out_path} (dict={dict_name} id={marker_id} size={size})")

def main():
    p = argparse.ArgumentParser()
    p.add_argument("--dict", required=True, help="ArUco dictionary name, e.g. DICT_4X4_50")
    p.add_argument("--id", type=int, required=True, help="Marker id (integer)")
    p.add_argument("--size", type=int, default=512, help="Output image size in pixels (square)")
    p.add_argument("--out", required=True, help="Output PNG path")
    args = p.parse_args()

    if not hasattr(cv2.aruco, "getPredefinedDictionary"):
        print("Your OpenCV build does not have aruco module with getPredefinedDictionary.", file=sys.stderr)
        sys.exit(1)

    generate_marker(args.dict, args.id, args.size, args.out)

if __name__ == "__main__":
    main()