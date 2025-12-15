import pyrealsense2 as rs
import numpy as np
import cv2
import torch
import pathlib
from pathlib import Path
import time

from move_function import *

# Path to custom weights file
weights_path = r"C:\Users\NIIL\Desktop\240808_krri.pt"

# Replace pathlib.PosixPath with pathlib.WindowsPath for path conversion
pathlib.PosixPath = pathlib.WindowsPath

# Load YOLOv5 model with custom weights
model = torch.hub.load('ultralytics/yolov5', 'custom', path=weights_path, force_reload=True)

# Initialize RealSense pipeline
pipeline = rs.pipeline()
config = rs.config()

config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 60)
config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 60)

# Start the pipeline and get camera intrinsics
profile = pipeline.start(config)
depth_sensor = profile.get_device().first_depth_sensor()
depth_scale = depth_sensor.get_depth_scale()

# Camera specifications
focal_length_mm = 1.88  # Focal length in millimeters
sensor_width_mm = 3.67  # Sensor width in millimeters
sensor_height_mm = 2.74  # Sensor height in millimeters

# Active pixels
active_pixels_width = 1280
active_pixels_height = 720

# Calculate focal length in pixels
focal_length_px = (focal_length_mm * active_pixels_height) / sensor_height_mm

start_time = time.time()

fourcc = cv2.VideoWriter_fourcc(*'XVID')
out = cv2.VideoWriter('5nd_palette.mp4', fourcc, 15, (640, 360))

# Initialize a flag to ensure the fork-up action is performed only once
fork_action_taken = False

try:
    while True:
        # Wait for a new set of frames
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()

        if not depth_frame or not color_frame:
            continue

        # Convert images to numpy arrays
        depth_image = np.asanyarray(depth_frame.get_data()) * depth_scale
        color_image = np.asanyarray(color_frame.get_data())

        # Perform object detection using YOLOv5
        results = model(color_image)

        # Get detection results
        detections = results.xyxy[0].cpu().numpy()

        bottom_box = []

        now = time.time() - start_time
        break_time = 15

        sort_detections = sorted(detections, key=lambda x: x[3], reverse=True)
        pick_palette = 5

        for idx, detection in enumerate(sort_detections):
            x1, y1, x2, y2, conf, cls = detection

            # Draw bounding boxes for visualization
            label = f'{model.names[int(cls)]}'
            if now < break_time:
                if idx == (pick_palette - 1):
                    # Highlight the picked palette in red
                    cv2.rectangle(color_image, (int(x1), int(y1)), (int(x2), int(y2)), (0, 0, 235), 2)
                    cv2.putText(color_image, label, (int(x1), int(y1) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 235),
                                2)
                else:
                    # Other palettes in green
                    cv2.rectangle(color_image, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
                    cv2.putText(color_image, label, (int(x1), int(y1) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0),
                                2)

            # Find the bottom-most box to use as a reference point (y=0)
            if not bottom_box or y2 > bottom_box[3]:
                bottom_box = [x1, y1, x2, y2]

        try:
            # Proceed only if enough palettes are detected
            if len(sort_detections) >= pick_palette:
                pick_x1, pick_y1, pick_x2, pick_y2, _, _ = sort_detections[pick_palette - 1]
                up_point_y = pick_y1
                down_point_y = pick_y2
                up_point_x = pick_x1 + (pick_x2 - pick_x1) / 2

                # Use the y2 of the bottom-most box as the zero-point
                if bottom_box:
                    _, _, _, y2_bottom = bottom_box
                    zero_point_y = int(y2_bottom)

                    # Calculate the center of the target palette for depth measurement
                    target_point_y = int(up_point_y) + int((down_point_y - up_point_y) / 2)

                    # Get depth value from a small area around the target point
                    depth_value = np.mean(depth_image[int(target_point_y - 5):int(target_point_y + 5),
                                          int(up_point_x - 5):int(up_point_x + 5)])

                    # Calculate the real-world height relative to the bottom palette
                    # Ensure depth_value is valid to avoid errors
                    if not np.isnan(depth_value) and depth_value > 0:
                        real_world_height = ((zero_point_y - target_point_y) * 1.6 * depth_value) / focal_length_px

                        depth_height_label = f'Depth: {depth_value:.3f}m, Height: {real_world_height:.3f}m'
                        if now < break_time:
                            cv2.putText(color_image, depth_height_label, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                                        (255, 255, 255), 2)

                        # --- EXECUTE FORK UP AND MOVE FORWARD ACTION ---
                        # If height is calculated and action has not been taken, call fork_up() and go_forward()
                        if not fork_action_taken and real_world_height > 0:
                            print(f"Target height calculated: {real_world_height:.3f}m. Initiating forking sequence.")

                            # 1. Raise the fork to the calculated height
                            fork_up(real_world_height)
                            print("fork_up command has been sent.")

                            # 2. Move forward to insert the fork into the pallet
                            # Move forward by 80cm, considering the fork length is about 90cm.
                            go_forward(80)
                            print("go_forward(80) command has been sent.")

                            # 3. Slightly lift the fork to prevent collision with the pallet below.
                            fork_up(10)
                            print("fork_up(10) command has been sent to lift the pallet slightly.")

                            # 4. Move backward to pull the pallet out from the stack.
                            go_backward(80)
                            print("go_backward(80) command has been sent.")

                            fork_action_taken = True  # Set flag to true to prevent repeated calls

            cv2.imshow('RealSense YOLOv5', color_image)

        except Exception as e:
            # Display the image even if calculations fail
            print(f"An error occurred during calculation: {e}")
            cv2.imshow('RealSense YOLOv5', color_image)

        out.write(color_image)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
finally:
    pipeline.stop()
    cv2.destroyAllWindows()
    out.release()

