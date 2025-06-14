import serial
import time
from contextlib import contextmanager
from robot import *
import requests
import numpy as np
import cv2


COLOR_POSITIONS = {
    "green": {"x": 250, "y": -250},    # Right side
    "blue": {"x": -250, "y": 250},    # Left side
    "red": {"x": 250, "y": 250}       # Top side
}

# === Load Homography ===
try:
    H = np.load("homography.npy")
except FileNotFoundError:
    print("❌ homography.npy not found. Run the calibration script first.")
    exit(1)

# === Convert camera (pixel) to robot (mm) coordinates ===
def camera_to_robot(x, y):
    pt = np.array([[[x, y]]], dtype=np.float32)
    result = cv2.perspectiveTransform(pt, H)
    return result[0][0]

# === Main Logic ===
with connect_serial(port='COM6', baud=115200, timeout=2, graceful_exit_flag=True) as ser:
    if ser is None:
        exit(1)
        
    send_gcode(ser, "G28")  # Home
    move_to_position(ser, z=-800)  # Move to center

    while True:
        response = requests.get("http://127.0.0.1:5000/once")
        if response.status_code != 200:
            print("Error:", response.status_code)
            break

        data = response.json()
        for color, found in data.items():
            if found:
                for item in found:
                    print(f"Found {color} object")
                    x, y, width, height, area = item["x"], item["y"], item["width"], item["height"], item["area"]
                    print(f"Image coords: x={x}, y={y}, w={width}, h={height}, area={area}")

                    # Center of bounding box in image
                    cx = x + width / 2
                    cy = y + height / 2

                    # Convert to robot coordinates
                    rx, ry = camera_to_robot(cx, cy)
                    print(f"→ Robot coords: X={rx:.2f}, Y={ry:.2f}")

                    # Send movement command
                    move_to_position(ser, x=rx, y=ry)
                    pick(ser)
                    
                    
                    drop_pos = COLOR_POSITIONS.get(color, {"x": 0, "y": 0})  # Default to center if color not found
                    move_to_position(ser, x=drop_pos["x"], y=drop_pos["y"])
                    place(ser)
    

    # move_to_position(ser, x=100)
    # move_to_position(ser, y=100)
    # move_to_position(ser, x=0)
    # move_to_position(ser, y=0)
    
    # move_to_position(ser, z=-900)
    # enable_effector(ser)
    # move_to_position(ser, x=0, y=0)
    # move_to_position(ser, x=50, y=0)
    # move_to_position(ser, x=50, y=-50)
    # move_to_position(ser, x=-50, y=-50)
    # pick(ser)
    # move_to_position(ser, x=100, y=100)
    # place(ser)
    
    
    # disable_effector(ser)



