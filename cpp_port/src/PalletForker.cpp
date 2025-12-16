#include "PalletForker.h"
#include <iostream>
#include <filesystem>
#include <algorithm>
#include <numeric>

namespace AutonomousForklift {

PalletForker::PalletForker(const Config& config, std::shared_ptr<Control::ForkliftController> forklift_controller)
    : BaseController(config, forklift_controller, "forker"),
      config_(config),
      state_(ForkingState::ALIGNING)
{
    focal_length_px_ = (config_.focal_length_mm * config_.camera_height) / config_.sensor_height_mm;
    log("PalletForker child class initialized.");
}

void PalletForker::handleStateAligning(double current_time_sec, const std::vector<Vision::Detection>& detections, const cv::Mat& depth_image) {
    log("State: ALIGNING");
    if (current_time_sec > config_.align_wait_sec) {
        if (!detections.empty()) {
            processDetections(detections, depth_image);
        } else {
            log("No detections, cannot proceed to fork.");
        }
    } else {
        log("Aligning... waiting for align_wait_sec to pass.");
    }
}

void PalletForker::handleStateForkUp() {
    log("State: FORK_UP complete. Now going forward.");
    forklift_controller_->goForward(config_.go_forward_cm);
    state_ = ForkingState::GOING_FORWARD;
}

void PalletForker::handleStateGoingForward() {
    log("State: GOING_FORWARD complete. Lifting pallet.");
    forklift_controller_->forkUp(config_.slight_fork_up_cm);
    state_ = ForkingState::LIFTING_PALLET;
}

void PalletForker::handleStateLiftingPallet() {
    log("State: LIFTING_PALLET complete. Now going backward.");
    forklift_controller_->goBackward(config_.go_backward_cm);
    state_ = ForkingState::GOING_BACKWARD;
}

void PalletForker::handleStateGoingBackward() {
    log("State: GOING_BACKWARD complete. Forking sequence finished.");
    state_ = ForkingState::COMPLETED;
}

void PalletForker::processDetections(const std::vector<Vision::Detection>& detections, const cv::Mat& depth_image) {
    if (detections.size() < config_.pick_palette_idx) {
        log("Not enough palettes detected.");
        return;
    }

    std::vector<Vision::Detection> sorted_detections = detections;
    std::sort(sorted_detections.begin(), sorted_detections.end(), [](const Vision::Detection& a, const Vision::Detection& b) {
        return (a.box.y + a.box.height) > (b.box.y + b.box.height);
    });

    const Vision::Detection& pick_palette_det = sorted_detections[config_.pick_palette_idx - 1];
    const cv::Rect& bottom_box = sorted_detections[0].box;

    int pick_y1 = pick_palette_det.box.y;
    int pick_y2 = pick_palette_det.box.y + pick_palette_det.box.height;
    int pick_x_center = pick_palette_det.box.x + pick_palette_det.box.width / 2;
    int zero_point_y = bottom_box.y + bottom_box.height;
    int target_point_y = pick_y1 + (pick_y2 - pick_y1) / 2;

    cv::Rect depth_roi(pick_x_center - 5, target_point_y - 5, 10, 10);
    depth_roi &= cv::Rect(0, 0, depth_image.cols, depth_image.rows);
    
    double depth_value = 0.0;
    if (!depth_roi.empty()) {
        cv::Mat roi_depth_mat = depth_image(depth_roi);
        std::vector<uint16_t> valid_depths;
        for (int r = 0; r < roi_depth_mat.rows; ++r) {
            for (int c = 0; c < roi_depth_mat.cols; ++c) {
                uint16_t val = roi_depth_mat.at<uint16_t>(r, c);
                if (val > 0) valid_depths.push_back(val);
            }
        }
        if (!valid_depths.empty()) {
            depth_value = static_cast<double>(std::accumulate(valid_depths.begin(), valid_depths.end(), 0.0) / valid_depths.size()) * camera_.getDepthScale();
        }
    }

    if (depth_value > 0.0) {
        double real_world_height_m = (static_cast<double>(zero_point_y - target_point_y) * 1.6 * depth_value) / focal_length_px_;
        double real_world_height_cm = real_world_height_m * 100.0;

        if (real_world_height_cm > 0.0) {
            log("Target height calculated: " + std::to_string(static_cast<int>(real_world_height_cm)) + "cm. Initiating forking sequence.");
            forklift_controller_->forkUp(real_world_height_cm);
            state_ = ForkingState::FORK_UP;
        } else {
            log("Calculated height is not positive. Retrying.");
        }
    } else {
        log("Invalid depth value obtained. Retrying.");
    }
}

bool PalletForker::run() {
    cv::Mat color_image, depth_image;
    
    while (state_ != ForkingState::COMPLETED && state_ != ForkingState::FAILED && cv::waitKey(1) != 'q') {
        if (!camera_.getFrames(color_image, depth_image)) {
            log("Failed to get camera frames. Exiting run loop.");
            state_ = ForkingState::FAILED;
            break;
        }

        std::vector<Vision::Detection> detections = detector_.detect(color_image);
        for (const auto& det : detections) {
            cv::rectangle(color_image, det.box, cv::Scalar(0, 255, 0), 2);
        }

        if (!forklift_controller_->isBusy()) {
            auto loop_start_time = std::chrono::high_resolution_clock::now();
            double current_time_sec = std::chrono::duration<double>(loop_start_time - start_time_).count();

            switch (state_) {
                case ForkingState::ALIGNING:       handleStateAligning(current_time_sec, detections, depth_image); break;
                case ForkingState::FORK_UP:        handleStateForkUp(); break;
                case ForkingState::GOING_FORWARD:  handleStateGoingForward(); break;
                case ForkingState::LIFTING_PALLET: handleStateLiftingPallet(); break;
                case ForkingState::GOING_BACKWARD: handleStateGoingBackward(); break;
                default: break;
            }
        }

        forklift_controller_->update();
        if (video_writer_.isOpened()) video_writer_.write(color_image);
        cv::imshow(config_.window_name, color_image);
    }

    cleanup();
    return state_ == ForkingState::COMPLETED;
}

} // namespace AutonomousForklift