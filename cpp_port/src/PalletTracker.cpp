#include "PalletTracker.h"
#include <iostream>
#include <numeric>
#include <algorithm>

namespace AutonomousForklift {

PalletTracker::PalletTracker(const Config& config, std::shared_ptr<Control::ForkliftController> forklift_controller)
    : BaseController(config, forklift_controller, "tracker"),
      config_(config),
      kf_x_(config.kf_r_val, config.kf_q_var),
      last_cls_(""), 
      last_object_depth_(0.0), 
      last_center_x_(config.camera_width / 2), 
      last_p_distance_(0.0)
{
    log("PalletTracker child class initialized.");
}

double PalletTracker::calculateMedian(std::vector<double>& data) {
    if (data.empty()) return 0.0;
    size_t mid_idx = data.size() / 2;
    std::nth_element(data.begin(), data.begin() + mid_idx, data.end());
    double median = data[mid_idx];
    if (data.size() % 2 == 0) {
        auto max_it = std::max_element(data.begin(), data.begin() + mid_idx);
        median = (*max_it + median) / 2.0;
    }
    return median;
}

bool PalletTracker::processFrame(cv::Mat& color_image, cv::Mat& depth_image) {
    std::vector<Vision::Detection> detections = detector_.detect(color_image);
    if (detections.empty()) return false;

    auto best_det_it = std::max_element(detections.begin(), detections.end(),
                                       [](const Vision::Detection& a, const Vision::Detection& b) {
                                           return a.confidence < b.confidence;
                                       });
    Vision::Detection& best_det = *best_det_it;
    
    int raw_center_x = best_det.box.x + best_det.box.width / 2;
    int center_y = best_det.box.y + best_det.box.height / 2;

    kf_x_.predict();
    last_center_x_ = static_cast<int>(kf_x_.update(raw_center_x));
    
    cv::Rect roi = best_det.box & cv::Rect(0, 0, depth_image.cols, depth_image.rows);
    std::vector<double> roi_depth_values;
    if (!roi.empty()) {
        cv::Mat roi_depth_mat = depth_image(roi);
        for (int r = 0; r < roi_depth_mat.rows; ++r) {
            for (int c = 0; c < roi_depth_mat.cols; ++c) {
                uint16_t depth_val_raw = roi_depth_mat.at<uint16_t>(r, c);
                if (depth_val_raw > 0) {
                    roi_depth_values.push_back(static_cast<double>(depth_val_raw) * camera_.getDepthScale() * 100.0);
                }
            }
        }
    }
    
    last_object_depth_ = calculateMedian(roi_depth_values);
    if (roi_depth_values.empty()) {
        last_object_depth_ = std::numeric_limits<double>::infinity();
    }

    last_cls_ = "pallet";
    last_p_distance_ = AutonomousForklift::Calculations::calc_p_distance(last_center_x_, last_object_depth_);

    std::string label = last_cls_ + ": " + std::to_string(static_cast<int>(last_object_depth_)) + "cm";
    cv::rectangle(color_image, best_det.box, cv::Scalar(30, 119, 252), 2);
    cv::putText(color_image, label, cv::Point(best_det.box.x, std::max(0, best_det.box.y - 10)), cv::FONT_HERSHEY_SIMPLEX, 0.9, cv::Scalar(30, 119, 252), 2);
    cv::circle(color_image, cv::Point(raw_center_x, center_y), 7, cv::Scalar(0, 0, 255), -1);
    cv::circle(color_image, cv::Point(last_center_x_, center_y), 7, cv::Scalar(0, 255, 0), -1);
    
    log("Detection: " + last_cls_ + ", Filtered CX: " + std::to_string(last_center_x_) + ", Depth: " + std::to_string(static_cast<int>(last_object_depth_)) + "cm");

    return true;
}

void PalletTracker::handleControlLogic() {
    if (forklift_controller_->isBusy()) return;

    if (last_object_depth_ >= config_.target_depth_min_cm && last_object_depth_ <= config_.target_depth_max_cm &&
        std::abs(last_center_x_ - (config_.camera_width / 2)) <= config_.target_center_tolerance) {
        log("--- Final target reached! ---");
        forklift_controller_->stop();
    } else {
        double ratio = (last_object_depth_ > config_.min_depth_cm_for_move) ? std::min(1.0, (last_object_depth_ - config_.min_depth_cm_for_move) / (config_.max_depth_cm_for_move - config_.min_depth_cm_for_move)) : 0.0;
        double center_tolerance = config_.min_tolerance + ratio * (config_.max_tolerance - config_.min_tolerance);
        
        double left_bound = (config_.camera_width / 2) - center_tolerance;
        double right_bound = (config_.camera_width / 2) + center_tolerance;

        if (last_center_x_ >= left_bound && last_center_x_ <= right_bound) {
            if (last_object_depth_ > config_.min_depth_cm_for_move) {
                double move_distance = std::max(0.0, last_object_depth_ - config_.forward_move_target_cm);
                forklift_controller_->goForward(move_distance);
            }
        } else {
            if (last_center_x_ < left_bound) {
                forklift_controller_->turnCCW(config_.turn_angle_deg);
            } else {
                forklift_controller_->turnCW(config_.turn_angle_deg);
            }
        }
    }
}

bool PalletTracker::run() {
    bool target_reached = false;
    cv::Mat color_image, depth_image;

    while (cv::waitKey(1) != 'q') {
        if (!camera_.getFrames(color_image, depth_image)) {
            log("Failed to get camera frames. Exiting run loop.");
            break;
        }

        if (processFrame(color_image, depth_image)) {
            handleControlLogic();
        }
        
        forklift_controller_->update();
        if (video_writer_.isOpened()) video_writer_.write(color_image);
        cv::imshow(config_.window_name, color_image);

        if (last_object_depth_ >= config_.target_depth_min_cm && last_object_depth_ <= config_.target_depth_max_cm &&
            std::abs(last_center_x_ - (config_.camera_width / 2)) <= config_.target_center_tolerance) {
            target_reached = true;
            std::this_thread::sleep_for(std::chrono::seconds(1));
            break;
        }
    }

    cleanup();
    return target_reached;
}

} // namespace AutonomousForklift