#!/usr/bin/env python3

import argparse
import os
import cv2
import cv2.aruco as aruco


ARUCO_FAMILY_MAP = {
    "DICT_4X4_50": aruco.DICT_4X4_50,
    "DICT_4X4_100": aruco.DICT_4X4_100,
    "DICT_4X4_250": aruco.DICT_4X4_250,
    "DICT_4X4_1000": aruco.DICT_4X4_1000,
    "DICT_5X5_50": aruco.DICT_5X5_50,
    "DICT_5X5_100": aruco.DICT_5X5_100,
    "DICT_5X5_250": aruco.DICT_5X5_250,
    "DICT_5X5_1000": aruco.DICT_5X5_1000,
    "DICT_6X6_50": aruco.DICT_6X6_50,
    "DICT_6X6_100": aruco.DICT_6X6_100,
    "DICT_6X6_250": aruco.DICT_6X6_250,
    "DICT_6X6_1000": aruco.DICT_6X6_1000,
    "DICT_7X7_50": aruco.DICT_7X7_50,
    "DICT_7X7_100": aruco.DICT_7X7_100,
    "DICT_7X7_250": aruco.DICT_7X7_250,
    "DICT_7X7_1000": aruco.DICT_7X7_1000,
}


def generate_marker(family: str, marker_id: int, size_px: int):
    if family not in ARUCO_FAMILY_MAP:
        raise ValueError(
            f"Unsupported ArUco family '{family}'. "
            f"Valid options: {list(ARUCO_FAMILY_MAP.keys())}"
        )

    aruco_dict = aruco.getPredefinedDictionary(ARUCO_FAMILY_MAP[family])

    output_dir = os.path.join(
        "/home/tejas/dev_ws/src/Drone_IBVS/models", "aruco_markers", family, f"marker_{marker_id}"
    )
    os.makedirs(output_dir, exist_ok=True)

    output_path = os.path.join(
        output_dir, f"marker.png"
    )

    marker_img = aruco.drawMarker(
        aruco_dict,
        marker_id,
        size_px
    )

    cv2.imwrite(output_path, marker_img)

    print(f"[INFO] ArUco marker generated:")
    print(f"       Family     : {family}")
    print(f"       ID         : {marker_id}")
    print(f"       Size (px)  : {size_px}")
    print(f"       Output     : {output_path}")


def main():
    parser = argparse.ArgumentParser(
        description="Generate ArUco marker images for Gazebo / IBVS simulation"
    )

    parser.add_argument(
        "--family",
        required=True,
        type=str,
        help="ArUco dictionary (e.g. DICT_4X4_50)"
    )

    parser.add_argument(
        "--id",
        required=True,
        type=int,
        help="Marker ID"
    )

    parser.add_argument(
        "--size",
        default=800,
        type=int,
        help="Marker size in pixels (default: 800)"
    )

    args = parser.parse_args()

    generate_marker(
        family=args.family,
        marker_id=args.id,
        size_px=args.size
    )


if __name__ == "__main__":
    main()
