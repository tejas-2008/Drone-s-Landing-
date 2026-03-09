#!/usr/bin/env python3

import cv2
import cv2.aruco as aruco
import argparse
import sys
import numpy as np

def get_dictionary(dict_name):
    dict_map = {
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
        "DICT_ARUCO_ORIGINAL": aruco.DICT_ARUCO_ORIGINAL
    }
    return aruco.getPredefinedDictionary(dict_map.get(dict_name, aruco.DICT_4X4_50))

def main():
    parser = argparse.ArgumentParser(description='Generate an ArUco Fractal Marker.')
    parser.add_argument('--output', type=str, required=True, help='Output filename (e.g., fractal.png)')
    parser.add_argument('--dictionary', type=str, default='DICT_5X5_50', help='ArUco dictionary to use')
    parser.add_argument('--levels', type=int, default=2, help='Number of fractal levels')
    parser.add_argument('--scale', type=float, default=0.5, help='Fractal scale factor (0.0 to 1.0)')
    parser.add_argument('--pixel_size', type=int, default=1000, help='Size of the output image in pixels')
    parser.add_argument('--margin', type=int, default=10, help='Margin size in pixels')
    parser.add_argument('--border_bits', type=int, default=1, help='Number of bits in marker border')

    args = parser.parse_args()

    # Check if FractalMarkerSet is available (requires opencv-contrib-python)
    if not hasattr(aruco, "FractalMarkerSet_create"):
        print("Error: cv2.aruco.FractalMarkerSet_create not found.")
        print("Please ensure you have 'opencv-contrib-python' installed.")
        sys.exit(1)

    dictionary = get_dictionary(args.dictionary)
    
    print(f"-----Generating Fractal Marker-----")
    print(f"  Dictionary: {args.dictionary}")
    print(f"  Levels: {args.levels}")
    print(f"  Scale: {args.scale}")
    print(f"  Output Size: {args.pixel_size}x{args.pixel_size}")
    
    # Create the fractal marker set
    # Parameters: dictionary, levels, scale, size (1.0 for normalized), margin (1.0 for normalized)
    # We use 1.0 for size to keep it normalized, we'll scale during drawing
    fractal_set = aruco.FractalMarkerSet_create(dictionary, args.levels, args.scale, 1.0, 1)

    # Get the board for drawing
    # Note: different OpenCV versions might have slightly different APIs for extracting the image
    # For a FractalMarkerSet, we can often treat it similarly to a Board or use specific draw methods.
    
    try:
        # Drawing the fractal marker
        # image_size is (width, height)
        out_size = (args.pixel_size, args.pixel_size)
        
        # In newer OpenCV versions, FractalMarkerSet might not inherit directly from Board in a way 
        # that allow drawPlanarBoard to work seamlessly without the Board object.
        # However, usually there's a method to get the image.
        
        # Attempt 1: Using draw() method of FractalMarkerSet if it exists (common in some wrappers)
        if hasattr(fractal_set, 'draw'):
             img = fractal_set.draw(out_size, args.margin, args.border_bits)
        # Attempt 2: Using aruco.drawPlanarBoard if we can get a board
        elif hasattr(fractal_set, 'getBoard'):
             board = fractal_set.getBoard()
             img = board.generateImage(out_size, args.margin, args.border_bits)
        else:
             # Fallback/Generic attempt (API dependent)
             # Some versions use a static draw function taking the fractal set
             img = aruco.drawPlanarBoard(fractal_set, out_size, args.margin, args.border_bits)

        cv2.imwrite(args.output, img)
        print(f"Successfully saved fractal marker to {args.output}")

    except Exception as e:
        print(f"Error generating image: {e}")
        # Print available attributes to help debug if it fails
        print("Available attributes in fractal_set:", dir(fractal_set))

if __name__ == "__main__":
    main()
