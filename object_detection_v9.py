import pyrealsense2 as rs
import numpy as np
import cv2
import torch
import pathlib
from pathlib import Path
import time


# 사용자 정의 가중치 파일 경로
# weights_path = r"C:\Users\NIIL\Desktop\241129_loader_2.pt"
weights_path = r"C:\Users\NIIL\Desktop\241129_palette.pt"

# 경로 변환을 위해 pathlib.PosixPath를 pathlib.WindowsPath로 대체
pathlib.PosixPath = pathlib.WindowsPath

# # 사용자 정의 가중치를 사용하여 YOLOv5 모델 로드
model = torch.hub.load('./yolov5', 'custom', path=weights_path, force_reload=True, source='local')


# RealSense 파이프라인 초기화
pipeline = rs.pipeline()
config = rs.config()

config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)

# Start the pipeline and get camera intrinsics
profile = pipeline.start(config)
depth_sensor = profile.get_device().first_depth_sensor()
depth_scale = depth_sensor.get_depth_scale()

# Camera specifications
focal_length_mm = 1.88  # Focal length in millimeters
sensor_width_mm = 3.67  # Sensor width in millimeters
sensor_height_mm = 2.74  # Sensor height in millimeters

# Active pixels
active_pixels_width = 1920
active_pixels_height = 1080

# Calculate focal length in pixels
focal_length_px = (focal_length_mm * active_pixels_height) / sensor_height_mm

start_time = time.time()

# save_name = '241129_loader_real_test.mp4'
save_name = '241129_palette_real_test.mp4'
fourcc = cv2.VideoWriter_fourcc(*'XVID')
out = cv2.VideoWriter(save_name, fourcc, 10, (1280, 720))
out_detect = cv2.VideoWriter(save_name.split('.')[0] + '_detect.mp4', fourcc, 10, (1280, 720))

try:
    while True:
        # 새 프레임 세트를 기다림
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()

        if not depth_frame or not color_frame:
            continue

        # 이미지를 numpy 배열로 변환
        depth_image = np.asanyarray(depth_frame.get_data()) * depth_scale
        save_image = np.asanyarray(color_frame.get_data())
        color_image = save_image.copy()

        # YOLOv5를 사용하여 객체 탐지 수행
        results = model(color_image)

        # 결과 정보 가져오기
        detections = results.xyxy[0].cpu().numpy()

        top_box = []
        bottom_box = []
        y2_rank = []

        # now = time.time() - start_time
        break_time = 15
        print(detections)
        sort_detections = sorted(detections, key=lambda x: x[3], reverse=True)
        pick_palette = 5

        for idx, detection in enumerate(sort_detections):
            print("depth_scale", depth_scale)
            x1, y1, x2, y2, conf, cls = detection

            bbox_width = x2 - x1
            bbox_height = y2 - y1
            bbox_area = bbox_width * bbox_height

            area_center_x = int(x1 + bbox_width / 2)
            area_center_y = int(y1 + bbox_height / 2)
            print("area_center_x,area_center_y", area_center_x, area_center_y)
            print("bbox_area", bbox_area)

            depth_value = np.median(
                depth_image[int(y1):int(y1 + bbox_height / 2), int(area_center_x - 10):int(area_center_x + 10)])
            print("depth_value", depth_value)

            bbox_bottom = y2
            print("bbox_bottom: ", bbox_bottom)

            bbox_height_pixels = bbox_bottom - area_center_y
            print("palette y2: ", bbox_height_pixels)

            real_world_height = (bbox_height_pixels * depth_value) / focal_length_px
            print("Real World Height: ", real_world_height)

            label = f'{model.names[int(cls)]}'
            # if now < break_time:
            # 타겟 팔레트 빨간색, 나머지 초록색
            # if idx == (pick_palette-1):
            #     cv2.rectangle(color_image, (int(x1), int(y1)), (int(x2), int(y2)), (0, 0, 235), 2)
            #     cv2.putText(color_image, label, (int(x1), int(y1) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 235), 2)
            # else:
            #     cv2.rectangle(color_image, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
            #     cv2.putText(color_image, label, (int(x1), int(y1) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            # 모든 팔레트 초록색
            cv2.rectangle(color_image, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
            cv2.putText(color_image, label, (int(x1), int(y1) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            # 팔레트 홈 point
            home_ratio = 1.2
            left_home_x = x1 + ((area_center_x - x1)/2) * home_ratio
            right_home_x = area_center_x + ((x2 - area_center_x)/2) * (1/home_ratio)
            home_y = area_center_y

            # 좌표에 초록색 점 표시
            cv2.circle(color_image, (int(left_home_x), int(home_y)), 3, (0, 0, 235), -1)
            cv2.circle(color_image, (int(right_home_x), int(home_y)), 3, (0, 0, 235), -1)


            # 가장 위에 있는 bounding box의 좌표를 찾기 위해 bbox_bottom을 비교
            if not top_box or bbox_bottom < top_box[3]:
                top_box = [x1, y1, x2, y2]

            # 가장 아래에 있는 bounding box의 좌표를 찾기 위해 bbox_bottom을 비교
            if not bottom_box or bbox_bottom > bottom_box[3]:
                bottom_box = [x1, y1, x2, y2]

        print("Bottom Box Coordinates: ", bottom_box)

        try:
            pick_x1, pick_y1, pick_x2, pick_y2, _, _ = sort_detections[pick_palette-1]
            up_point_y = pick_y1
            down_point_y = pick_y2
            up_point_x = pick_x1 + (pick_x2-pick_x1)/2
            bottom_box_label = f'pick Box: ({pick_x1:.0f}, {pick_y1:.0f}, {pick_x2:.0f}, {pick_y2:.0f})'
                # if now < break_time:
                #     cv2.putText(color_image, bottom_box_label, (340, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)


            # 가장 아래에 있는 bounding box의 depth와 height 정보를 포함한 라벨 추가
            if bottom_box:
                x1, y1, x2, y2 = bottom_box
                zero_point_y = int(y2)
                bottom_box_label = f'Bottom Box: ({x1:.0f}, {y1:.0f}, {x2:.0f}, {y2:.0f})'
                # if now < break_time:
                #     cv2.putText(color_image, bottom_box_label, (340, 330), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
                print("zero_point_y: ", zero_point_y)


            target_point = int(up_point_y) + int((down_point_y - up_point_y) / 2)
            # depth_value = np.mean(depth_image[int(target_point-5):int(target_point+5), int(up_point_x-5):int(up_point_x+5)])
            real_world_height = ((zero_point_y-target_point) * 1.6 * depth_value) / focal_length_px

            depth_height_label = f'Depth: {depth_value:.3f}m, Height: {real_world_height:.3f}m'
            # if now < break_time:
            #     cv2.putText(color_image, depth_height_label, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
            cv2.imshow('RealSense YOLOv5', color_image)

        except:
            cv2.imshow('RealSense YOLOv5', color_image)

        out.write(save_image)
        out_detect.write(color_image)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
finally:
    pipeline.stop()
    cv2.destroyAllWindows()
    out.release()
    out_detect.release()
