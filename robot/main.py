import serial
import time
from contextlib import contextmanager
from robot import * # Assuming robot.py contains connect_serial, send_gcode, move_to_position, pick, place, enable_effector, disable_effector
import requests
import numpy as np
import cv2
from flask import Flask, request, jsonify
from flask_cors import CORS # To handle Cross-Origin Resource Sharing

# --- Flask App Setup ---
app = Flask(__name__)
CORS(app) # Enable CORS for all routes

# --- Robot Global State (for simplicity, in a real app you might use a more robust state management) ---
current_robot_coordinates = {"x": 0, "y": 0, "z": 0}
current_robot_command = {"name": "idle", "status": "completed"}
robot_connected = False
robot_serial_connection = None # To hold the serial connection

COLOR_POSITIONS = {
    "green": {"x": 250, "y": -250},    # Right side
    "blue": {"x": -250, "y": 250},    # Left side
    "red": {"x": 250, "y": 250}        # Top side
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

# --- Helper function to update robot coordinates (simulate or read from robot) ---
def update_robot_coordinates(x, y, z):
    global current_robot_coordinates
    current_robot_coordinates = {"x": x, "y": y, "z": z}

# --- API Endpoints ---

@app.route("/api/position", methods=["GET"])
def get_position():
    return jsonify(current_robot_coordinates)

@app.route("/api/command/current", methods=["GET"])
def get_current_command():
    return jsonify(current_robot_command)

@app.route("/api/connect", methods=["POST"])
def connect_robot():
    global robot_connected, robot_serial_connection
    if not robot_connected:
        try:
            # Using your existing connect_serial context manager
            # Note: This is a simplified way. In a production app, you'd manage the serial connection more robustly.
            ser_temp = serial.Serial(port='COM3', baudrate=115200, timeout=2)
            robot_serial_connection = ser_temp
            robot_connected = True
            print("✅ Robot connected.")
            send_gcode(robot_serial_connection, "G28") # Home on connect
            update_robot_coordinates(0, 0, 0) # Assuming homing resets to 0,0,0
            return jsonify(True), 200
        except serial.SerialException as e:
            print(f"❌ Failed to connect to robot: {e}")
            robot_connected = False
            return jsonify(False), 500
    return jsonify(True), 200 # Already connected

@app.route("/api/disconnect", methods=["POST"])
def disconnect_robot():
    global robot_connected, robot_serial_connection
    if robot_connected and robot_serial_connection:
        try:
            robot_serial_connection.close()
            robot_serial_connection = None
            robot_connected = False
            print("🔌 Robot disconnected.")
            return jsonify(True), 200
        except Exception as e:
            print(f"❌ Error disconnecting robot: {e}")
            return jsonify(False), 500
    return jsonify(False), 200 # Already disconnected or not connected

@app.route("/api/move", methods=["POST"])
def move_to_position_api():
    global current_robot_command
    if not robot_connected:
        return jsonify({"message": "Robot not connected"}), 400

    data = request.get_json()
    x = data.get("x")
    y = data.get("y")
    z = data.get("z")

    if x is None and y is None and z is None:
        return jsonify({"message": "At least one coordinate (x, y, or z) is required"}), 400

    # Execute robot movement
    try:
        current_robot_command = {"name": "moveToPosition", "status": "in_progress", "target": {"x": x, "y": y, "z": z}}
        # You'll need to adapt your `move_to_position` function to accept individual coordinates
        # For now, we'll assume it can handle None for unchanged axes
        move_to_position(robot_serial_connection, x=x, y=y, z=z)
        update_robot_coordinates(x, y, z) # Update global state
        current_robot_command = {"name": "moveToPosition", "status": "completed"}
        return jsonify(current_robot_command), 200
    except Exception as e:
        current_robot_command = {"name": "moveToPosition", "status": "failed", "error": str(e)}
        return jsonify(current_robot_command), 500

@app.route("/api/move/relative", methods=["POST"])
def move_relative_api():
    global current_robot_command
    if not robot_connected:
        return jsonify({"message": "Robot not connected"}), 400

    data = request.get_json()
    dx = data.get("dx", 0)
    dy = data.get("dy", 0)
    dz = data.get("dz", 0)

    try:
        current_robot_command = {"name": "moveRelative", "status": "in_progress", "delta": {"dx": dx, "dy": dy, "dz": dz}}
        # Assuming you have a `move_relative` function in `robot.py`
        # If not, you'll need to calculate target position based on current_robot_coordinates
        target_x = current_robot_coordinates["x"] + dx
        target_y = current_robot_coordinates["y"] + dy
        target_z = current_robot_coordinates["z"] + dz
        move_to_position(robot_serial_connection, x=target_x, y=target_y, z=target_z)
        update_robot_coordinates(target_x, target_y, target_z)
        current_robot_command = {"name": "moveRelative", "status": "completed"}
        return jsonify(current_robot_command), 200
    except Exception as e:
        current_robot_command = {"name": "moveRelative", "status": "failed", "error": str(e)}
        return jsonify(current_robot_command), 500

@app.route("/api/home", methods=["POST"])
def home_robot():
    global current_robot_command
    if not robot_connected:
        return jsonify({"message": "Robot not connected"}), 400
    try:
        current_robot_command = {"name": "home", "status": "in_progress"}
        send_gcode(robot_serial_connection, "G28")
        update_robot_coordinates(0, 0, 0) # Assuming homing resets to 0,0,0
        current_robot_command = {"name": "home", "status": "completed"}
        return jsonify(current_robot_command), 200
    except Exception as e:
        current_robot_command = {"name": "home", "status": "failed", "error": str(e)}
        return jsonify(current_robot_command), 500

@app.route("/api/stop", methods=["POST"])
def stop_robot():
    global current_robot_command
    if not robot_connected:
        return jsonify({"message": "Robot not connected"}), 400
    try:
        current_robot_command = {"name": "stop", "status": "in_progress"}
        # You'll need a way to send a stop command to your robot, e.g., M112 for emergency stop or a custom stop command.
        # For now, let's just simulate it.
        # send_gcode(robot_serial_connection, "M112") # Example: Emergency Stop
        print("Robot stop command issued (simulated if no actual stop G-code).")
        current_robot_command = {"name": "stop", "status": "completed"}
        return jsonify(current_robot_command), 200
    except Exception as e:
        current_robot_command = {"name": "stop", "status": "failed", "error": str(e)}
        return jsonify(current_robot_command), 500

# --- Object Detection and Pick/Place Logic (runs in the background or triggered) ---
# This part needs to be carefully integrated. Running it directly within Flask's main thread
# for continuous polling is not ideal for a responsive API. Consider using threading or
# a separate process for the vision part. For demonstration, we'll keep it as a function.

# Original object detection and robot action loop (can be triggered by an API call or run in a separate thread)
def run_vision_and_robot_actions():
    global robot_serial_connection
    print("Starting vision and robot action loop...")
    while robot_connected: # Only run if connected
        try:
            response = requests.get("http://127.0.0.1:5000/once") # This assumes your vision server is running on 5000
            if response.status_code != 200:
                print("Error fetching vision data:", response.status_code)
                time.sleep(1) # Wait a bit before retrying
                continue

            data = response.json()
            for color, found_items in data.items():
                if found_items:
                    for item in found_items:
                        print(f"Found {color} object")
                        x, y, width, height, area = item["x"], item["y"], item["width"], item["height"], item["area"]
                        print(f"Image coords: x={x}, y={y}, w={width}, h={height}, area={area}")

                        cx = x + width / 2
                        cy = y + height / 2

                        rx, ry = camera_to_robot(cx, cy)
                        print(f"→ Robot coords: X={rx:.2f}, Y={ry:.2f}")

                        if robot_serial_connection: # Ensure serial connection is active
                            move_to_position(robot_serial_connection, x=rx, y=ry, z=-800) # Move to object height
                            pick(robot_serial_connection) # Assume pick grasps at current Z
                            
                            drop_pos = COLOR_POSITIONS.get(color, {"x": 0, "y": 0})
                            move_to_position(robot_serial_connection, x=drop_pos["x"], y=drop_pos["y"], z=-800) # Move to drop height
                            place(robot_serial_connection) # Assume place releases at current Z
                            
                            # After placing, move back to a safe observation height
                            move_to_position(robot_serial_connection, z=-800) 
                        else:
                            print("Robot serial connection not active for movement.")
            time.sleep(0.1) # Small delay to avoid hammering the vision server
        except requests.exceptions.ConnectionError:
            print("Vision server not reachable. Retrying...")
            time.sleep(1)
        except Exception as e:
            print(f"An error occurred in vision/robot loop: {e}")
            time.sleep(1)


# --- Main execution for Flask ---
if __name__ == "__main__":
    # You might want to start the `run_vision_and_robot_actions` in a separate thread
    # so the Flask API remains responsive.
    import threading
    robot_action_thread = threading.Thread(target=run_vision_and_robot_actions, daemon=True)
    robot_action_thread.start()

    app.run(host="127.0.0.1", port=5000, debug=True)