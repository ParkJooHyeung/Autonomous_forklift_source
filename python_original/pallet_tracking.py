import cv2
import numpy as np
import pyrealsense2 as rs
import torch
import keyboard
import os
import logging
from datetime import datetime


# Import custom modules
from move_function import *
from kalman_filter import KalmanFilterWrapper  # Import the Kalman Filter wrapper

# --- 1. Configuration ---
weights = r"C:\Users\NIIL\Desktop\241129_palette.pt"
model_path = r"C:\Users\NIIL\Desktop\20250821_source_code\yolov5"
depth_scale = 0.095
WIN_NAME = "RealSense YOLOv5"


def get_box_info():
    # --- 2. Model Loading ---
    try:
        model = torch.hub.load(model_path, 'custom', source='local', path=weights)
        model.eval()
        print("YOLOv5 model loading successful")
    except Exception as e:
        print(f"YOLOv5 model loading failed: {e}")
        return

    # --- 3. RealSense Camera Setup ---
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
    config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)

    try:
        pipeline.start(config)
        print("RealSense Camera start Success")
    except RuntimeError as e:
        print(f"Failed to start the RealSense camera: {e}")
        return

    # Initialize Kalman Filter for the x-coordinate
    # R=5: Moderate trust in detection. Q=0.1: Expect some movement changes.
    # These values may need tuning for optimal performance.
    kf_x = KalmanFilterWrapper(R=5, Q_var=0.1)

    # Initialize global status values
    global last_cls, last_object_depth, last_center_x, last_p_distance
    last_cls, last_object_depth, last_p_distance = "", 0.0, 0.0
    last_center_x = 640  # Start with the screen center

    # --- 4. Logging and Video Recording Setup ---
    run_dir = f"run_{datetime.now().strftime('%Y-%m-%d_%H-%M-%S')}"
    os.makedirs(run_dir, exist_ok=True)
    log_file = os.path.join(run_dir, 'log.txt')
    logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(message)s',
                        handlers=[logging.FileHandler(log_file), logging.StreamHandler()])

    video_writer = None
    try:
        video_path = os.path.join(run_dir, 'output.mp4')
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        video_writer = cv2.VideoWriter(video_path, fourcc, 30.0, (1280, 720))
        logging.info(f"Starting log and video saving in '{run_dir}' directory.")

        cv2.namedWindow(WIN_NAME, cv2.WINDOW_NORMAL)
        target_reached = False

        # --- 5. Main Loop ---
        while not keyboard.is_pressed('q') and not target_reached:
            # === 5-1. Frame and Object Detection ===
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            depth_frame = frames.get_depth_frame()
            if not color_frame or not depth_frame:
                continue

            color_image = np.asanyarray(color_frame.get_data())
            results = model(color_image)
            det = results.xyxy[0].cpu().numpy()

            if det.shape[0] == 0:  # If no object is detected
                if video_writer: video_writer.write(color_image)
                cv2.imshow(WIN_NAME, color_image)
                if cv2.waitKey(1) & 0xFF == 27: break
                continue

            # Extract best detection
            best_idx = int(np.argmax(det[:, 4]))
            x1, y1, x2, y2, confidence, class_id = det[best_idx]
            x1i, y1i, x2i, y2i = map(int, [x1, y1, x2, y2])

            raw_center_x = int((x1 + x2) / 2.0)
            center_y = int((y1 + y2) / 2.0)

            # === 5-2. Apply Kalman Filter ===
            kf_x.predict()
            filtered_center_x = int(kf_x.update(raw_center_x))

            # Get depth and other info
            depth_image = np.asanyarray(depth_frame.get_data())
            depth_image_cm = depth_image * depth_scale
            roi_depth = depth_image_cm[y1i:y2i, x1i:x2i]
            object_depth = np.median(roi_depth) if roi_depth.size > 0 else float('inf')
            p_distance = float(calc_p_distance(filtered_center_x, object_depth))
            cls_name = model.names[int(class_id)]

            # Update status with filtered values
            last_cls, last_object_depth, last_center_x, last_p_distance = cls_name, object_depth, filtered_center_x, p_distance

            # === 5-3. Visualization ===
            label = f"{cls_name}: {object_depth:.2f}cm"
            cv2.rectangle(color_image, (x1i, y1i), (x2i, y2i), (252, 119, 30), 2)
            cv2.putText(color_image, label, (x1i, max(0, y1i - 10)), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (252, 119, 30), 2)
            # Draw raw detection center (Red)
            cv2.circle(color_image, (raw_center_x, center_y), 7, (0, 0, 255), -1)
            # Draw filtered center (Green)
            cv2.circle(color_image, (filtered_center_x, center_y), 7, (0, 255, 0), -1)

            if video_writer: video_writer.write(color_image)
            cv2.imshow(WIN_NAME, color_image)
            cv2.waitKey(1)

            logging.info(
                f"Detected: {last_cls} (Raw CX: {raw_center_x}, Filtered CX: {last_center_x}, Depth: {last_object_depth:.2f}cm)")

            # === 5-4. Check for Goal Completion ===
            # Use the stable, filtered center_x for the final check
            if 190 <= last_object_depth <= 200 and last_cls == 'neutral' and (640 - 5) <= last_center_x <= (640 + 5):
                target_reached = True
                logging.info("--- 🚀 Final target reached! Terminating all control sequences. ---")
                continue

            # === 5-5. Robot Control Logic (using filtered_center_x) ===
            if not target_reached:
                # Dynamic tolerance calculation constants
                MIN_TOLERANCE, MAX_TOLERANCE = 5, 50
                MIN_DEPTH, MAX_DEPTH = 200, 500

                # Calculate dynamic tolerance for centering
                ratio = np.clip((last_object_depth - MIN_DEPTH) / (MAX_DEPTH - MIN_DEPTH), 0, 1)
                center_tolerance = MIN_TOLERANCE + ratio * (MAX_TOLERANCE - MIN_TOLERANCE)
                logging.info(f"Dynamic tolerance: {center_tolerance:.2f} pixels")

                left_bound, right_bound = 640 - center_tolerance, 640 + center_tolerance

                # Execute control based on filtered position
                if left_bound <= last_center_x <= right_bound:
                    if last_object_depth > 200:
                        move_distance = max(0, last_object_depth - 245)
                        go_forward(move_distance)
                else:
                    # Simplified turning logic
                    if last_center_x < left_bound:
                        turnCCW(5)  # Example: turn by a fixed angle or calculate dynamically
                    elif last_center_x > right_bound:
                        turnCW(5)
                logging.info("--- Control logic executed ---")

    except Exception as e:
        logging.error(f"[CRITICAL] Exception in main loop: {e}", exc_info=True)
    finally:
        # --- 6. Cleanup ---
        logging.info("Exiting program. Cleaning up resources.")
        if video_writer: video_writer.release()
        cv2.destroyAllWindows()
        pipeline.stop()
        logging.shutdown()


if __name__ == "__main__":
    get_box_info()
