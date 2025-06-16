import cv2
import numpy as np
from robot import *
import threading
from queue import Queue
import time

# === SETTINGS ===
CAMERA_INDEX = 0       # 0 = default webcam
NUM_POINTS = 10        # Minimum 4 for homography
ROBOT_GRID = [        # Robot coordinates to visit
    (100, 100),
    (100, -100),
    (-100, -100),
    (-100, 100),
    (0, 0),
    (150, 0),
    (-150, 0),
    (0, 150),
    (0, -150),
    (50, 50)
]

# === GLOBAL STATE ===
clicks = []
robot_pts = []
current_point = 0
waiting_for_click = False
latest_frame = None
frame_lock = threading.Lock()
robot_queue = Queue()
camera_running = True

def camera_thread(cap):
    global latest_frame, camera_running
    while camera_running:
        ret, frame = cap.read()
        if ret:
            with frame_lock:
                global latest_frame
                latest_frame = frame.copy()
        # time.sleep(0.03)  # ~30fps

def robot_thread(ser):
    while camera_running:
        try:
            command = robot_queue.get(timeout=0.1)
            if command:
                x, y = command
                move_to_position(ser, x=x, y=y)
                time.sleep(0.5)  # Brief delay after movement
            robot_queue.task_done()
        except:
            pass

def on_mouse(event, x, y, flags, param):
    global clicks, robot_pts, current_point, waiting_for_click

    if event == cv2.EVENT_LBUTTONDOWN and current_point < len(ROBOT_GRID) and waiting_for_click:
        clicks.append((x, y))
        robot_pts.append(ROBOT_GRID[current_point])
        print(f"Point {current_point + 1}: Image({x}, {y}) -> Robot{ROBOT_GRID[current_point]}")
        current_point += 1
        waiting_for_click = False  # Reset waiting state to trigger next movement

def main():
    global latest_frame, camera_running, waiting_for_click

    waiting_for_click = False
    last_point_collected = -1  # Track last collected point

    # Initialize robot connection
    with connect_serial(port='COM3', baud=115200, timeout=2, graceful_exit_flag=True) as ser:
        if ser is None:
            print("❌ Error: Cannot connect to robot")
            return

        # Try to connect to camera multiple times
        retries = 3
        cap = None
        while retries > 0:
            cap = cv2.VideoCapture("rtmp://localhost/live/stream")
            if cap.isOpened():
                # Ensure we can actually read frames
                ret, frame = cap.read()
                if ret:
                    break
            cap.release()
            print(f"Retrying camera connection... ({retries} attempts left)")
            # time.sleep(1)
            retries -= 1

        if not cap or not cap.isOpened():
            print("❌ Error: Cannot open camera after multiple attempts.")
            return

        # Start camera and robot threads
        cam_thread = threading.Thread(target=camera_thread, args=(cap,))
        rob_thread = threading.Thread(target=robot_thread, args=(ser,))
        cam_thread.daemon = True
        rob_thread.daemon = True
        cam_thread.start()
        rob_thread.start()

        print("\n=== Interactive Homography Calibrator ===")
        print(f"1. Robot will move to {len(ROBOT_GRID)} different positions")
        print("2. For each position, click where the robot end effector is in the image")
        print("Press ESC to cancel\n")

        cv2.namedWindow("Live Feed")
        cv2.setMouseCallback("Live Feed", on_mouse)

        # Home the robot
        send_gcode(ser, "G28")
        move_to_position(ser, z=-965)  # Move to safe working height
        time.sleep(0.5)  # Give more time for robot to settle

        waiting_for_click = False
        while True:
            with frame_lock:
                if latest_frame is None:
                    continue
                frame = latest_frame.copy()

            # Move robot to next position if needed
            if current_point < len(ROBOT_GRID) and not waiting_for_click:
                robot_queue.put(ROBOT_GRID[current_point])
                waiting_for_click = True

            # Draw clicked points and current target
            for i, pt in enumerate(clicks):
                cv2.circle(frame, pt, 5, (0, 255, 0), -1)
                cv2.putText(frame, str(i+1), (pt[0]+5, pt[1]-5),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

            if waiting_for_click:
                # Draw text indicating waiting for click
                cv2.putText(frame, "Click end effector position", (10, 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

            cv2.imshow("Live Feed", frame)

            key = cv2.waitKey(1)
            if key == 27:  # ESC
                print("✋ Cancelled.")
                break
            if current_point >= len(ROBOT_GRID):
                break

            time.sleep(0.03)  # Add small delay to prevent CPU overload

        # Cleanup
        camera_running = False
        cam_thread.join()
        rob_thread.join()
        cap.release()
        cv2.destroyAllWindows()

        if len(clicks) < 4:
            print("❌ Not enough points collected.")
            return

        img_pts = np.array(clicks, dtype=np.float32)
        rob_pts = np.array(robot_pts, dtype=np.float32)

        H, status = cv2.findHomography(img_pts, rob_pts)
        if H is None:
            print("❌ Homography failed.")
            return

        print("\n✅ Homography matrix:")
        print(H)
        np.save("homography.npy", H)
        print("✔ Saved as homography.npy")

if __name__ == "__main__":
    main()
