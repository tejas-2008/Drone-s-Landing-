import cv2, os, argparse
import numpy as np
import itertools
import pandas as pd

def hamming_distance(a, b):
    return np.count_nonzero(a != b)

def get_marker_bits(dictionary, marker_id):
    """
    Extract flattened binary codeword of an ArUco marker
    """
    marker_size = dictionary.markerSize
    bits = dictionary.bytesList[marker_id][0]
    bits = np.unpackbits(bits)[:marker_size * marker_size]
    return bits

def compute_hamming_matrix(marker_bits, ids):
    """
    Compute NxN Hamming distance matrix
    """
    n = len(ids)
    mat = np.zeros((n, n), dtype=int)

    for i in range(n):
        for j in range(n):
            mat[i, j] = hamming_distance(
                marker_bits[ids[i]],
                marker_bits[ids[j]]
            )
    return mat

def select_best_marker_ids(
    aruco_dict_type=cv2.aruco.DICT_5X5_50,
    num_markers=5
):
    dictionary = cv2.aruco.getPredefinedDictionary(aruco_dict_type)
    total_markers = dictionary.bytesList.shape[0]

    # Pre-extract marker bits
    marker_bits = {
        i: get_marker_bits(dictionary, i)
        for i in range(total_markers)
    }

    best_subset = None
    best_min_dist = -1

    for subset in itertools.combinations(range(total_markers), num_markers):
        dists = [
            hamming_distance(marker_bits[i], marker_bits[j])
            for i, j in itertools.combinations(subset, 2)
        ]

        min_dist = min(dists)

        if min_dist > best_min_dist:
            best_min_dist = min_dist
            best_subset = subset

    return best_subset, best_min_dist, marker_bits

def generate_platform(
    platform_size_m=2.0,
    px_per_meter=500,
    center_marker_size_m=0.40,
    corner_marker_size_m=0.20,
    padding_m=0.15,
    min_distance_m=0.30,
    output_path="aruco_platform.png",
    aruco_dict=cv2.aruco.DICT_4X4_50, 
    marker_ids=[0, 1, 2, 3, 4]
):
    """
    Generate a square platform with 5 unique ArUco markers.
    """

    # Convert to pixels
    platform_px = int(platform_size_m * px_per_meter)
    padding_px = int(padding_m * px_per_meter)
    center_marker_px = int(center_marker_size_m * px_per_meter)
    corner_marker_px = int(corner_marker_size_m * px_per_meter)
    min_dist_px = int(min_distance_m * px_per_meter)

    # Sanity checks
    if padding_px * 2 + corner_marker_px > platform_px:
        raise ValueError("Padding + marker size exceeds platform size.")

    # Create white canvas
    canvas = np.ones((platform_px, platform_px), dtype=np.uint8) * 255

    # ArUco dictionary
    aruco_dictionary = cv2.aruco.getPredefinedDictionary(aruco_dict)

    # Marker IDs (unique)
    marker_ids = list(marker_ids)

    # Marker positions (top-left coordinates)
    positions = {
        "center": (
            platform_px // 2 - center_marker_px // 2,
            platform_px // 2 - center_marker_px // 2,
        ),
        "top_left": (padding_px, padding_px),
        "top_right": (
            platform_px - padding_px - corner_marker_px,
            padding_px,
        ),
        "bottom_left": (
            padding_px,
            platform_px - padding_px - corner_marker_px,
        ),
        "bottom_right": (
            platform_px - padding_px - corner_marker_px,
            platform_px - padding_px - corner_marker_px,
        ),
    }

    # Distance validation (center ↔ corners)
    cx = platform_px // 2
    cy = platform_px // 2
    for name, (x, y) in positions.items():
        if name == "center":
            continue
        mx = x + corner_marker_px // 2
        my = y + corner_marker_px // 2
        dist = np.hypot(mx - cx, my - cy)
        if dist < min_dist_px:
            raise ValueError(
                f"Marker '{name}' too close to center marker ({dist}px < {min_dist_px}px)"
            )
        print(f"Positions of marker '{name}' (wrt top-left): {mx/px_per_meter}m, {my/px_per_meter}m")

    # Draw center marker
    if hasattr(cv2.aruco, "generateImageMarker"):
        center_marker = cv2.aruco.generateImageMarker(
            aruco_dictionary, marker_ids[0], center_marker_px
        )
    else:
        center_marker = np.zeros((center_marker_px, center_marker_px), dtype=np.uint8)
        cv2.aruco.drawMarker(aruco_dictionary, marker_ids[0], center_marker_px, center_marker, 1)
    x, y = positions["center"]
    canvas[y:y + center_marker_px, x:x + center_marker_px] = center_marker

    # Draw small marker ID labels
    font = cv2.FONT_HERSHEY_SIMPLEX
    font_scale = max(0.3, corner_marker_px / 200.0)
    thickness = 1
    text_color = (0,)  # black for grayscale
    text_offset = 5

    def draw_id_text(marker_id, top_left, marker_px):
        text = str(marker_id)
        text_size, _ = cv2.getTextSize(text, font, font_scale, thickness)
        text_x = top_left[0] + text_offset
        text_y = top_left[1] + marker_px + text_size[1] + text_offset
        cv2.putText(canvas, text, (text_x, text_y), font, font_scale, text_color, thickness, cv2.LINE_AA)

    draw_id_text(marker_ids[0], positions["center"], center_marker_px)

    # Draw corner markers
    corner_names = ["top_left", "top_right", "bottom_left", "bottom_right"]
    for idx, name in enumerate(corner_names, start=1):
        if hasattr(cv2.aruco, "generateImageMarker"):
            marker = cv2.aruco.generateImageMarker(
                aruco_dictionary, marker_ids[idx], corner_marker_px
            )
        else:
            marker = np.zeros((corner_marker_px, corner_marker_px), dtype=np.uint8)
            cv2.aruco.drawMarker(aruco_dictionary, marker_ids[idx], corner_marker_px, marker, 1)
        x, y = positions[name]
        canvas[y:y + corner_marker_px, x:x + corner_marker_px] = marker
        draw_id_text(marker_ids[idx], positions[name], corner_marker_px)

    # Save
    cv2.imwrite(output_path, canvas)
    print(f"[✔] Platform generated: {output_path}")
    print(f"    Platform size : {platform_size_m}m x {platform_size_m}m")
    print(f"    Center marker : {center_marker_size_m}m")
    print(f"    Corner marker : {corner_marker_size_m}m")
    print(f"    Resolution    : {px_per_meter} px/m")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Generate square ArUco landing platform")

    parser.add_argument("--platform_size", type=float, default=2.0,
                        help="Platform size in meters (square)")
    parser.add_argument("--px_per_meter", type=int, default=500,
                        help="Resolution in pixels per meter")
    parser.add_argument("--center_marker_size", type=float, default=0.40,
                        help="Center marker size in meters (visible from ~5m)")
    parser.add_argument("--corner_marker_size", type=float, default=0.20,
                        help="Corner marker size in meters")
    parser.add_argument("--padding", type=float, default=0.15,
                        help="Padding from platform edge in meters")
    parser.add_argument("--min_distance", type=float, default=0.30,
                        help="Minimum distance between markers in meters")
    parser.add_argument("--output", type=str, default="aruco_platform.png",
                        help="Output image path")
    args = parser.parse_args()

    ids, min_dist, marker_bits = select_best_marker_ids(
        aruco_dict_type=cv2.aruco.DICT_5X5_50,
        num_markers=5
    )

    hamming_mat = compute_hamming_matrix(marker_bits, ids)

    # Pretty print as table
    df = pd.DataFrame(
        hamming_mat,
        index=[f"ID {i}" for i in ids],
        columns=[f"ID {i}" for i in ids]
    )

    print("\nSelected Marker IDs:")
    print(ids)

    print(f"\nMinimum pairwise Hamming distance: {min_dist}")

    print("\n5x5 Hamming Distance Matrix:")
    print(df)

    generate_platform(
        platform_size_m=args.platform_size,
        px_per_meter=args.px_per_meter,
        center_marker_size_m=args.center_marker_size,
        corner_marker_size_m=args.corner_marker_size,
        padding_m=args.padding,
        min_distance_m=args.min_distance,
        output_path=args.output,
        aruco_dict=cv2.aruco.DICT_5X5_50, 
        marker_ids=list(ids)
    )
