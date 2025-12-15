#include "PalletTracker.h"
#include <iostream>
#include <filesystem>
#include <algorithm> // For std::sort

namespace AutonomousForklift {

double PalletTracker::calculateMedian(std::vector<double>& data) {
    if (data.empty()) {
        return 0.0;
    }
    std::sort(data.begin(), data.end());
    size_t mid = data.size() / 2;
    return data.size() % 2 == 0 ? (data[mid - 1] + data[mid]) / 2.0 : data[mid];
}

void PalletTracker::log(const std::string& message) {
    std::cout << message << std::endl;
    if (log_file_.is_open()) {
        auto now = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> diff = now - start_time_;
        log_file_ << std::fixed << std::setprecision(3) << diff.count() << "s - " << message << std::endl;
    }
}

PalletTracker::PalletTracker(const Config& config, std::shared_ptr<Can::ICanController> can_controller)
    : config_(config),
      camera_(config.camera_width, config.camera_height, config.camera_fps),
      detector_(config.yolo_model_path, config.yolo_conf_threshold, config.yolo_nms_threshold),
      kf_x_(config.kf_r_val, config.kf_q_var),
      forklift_controller_(std::move(can_controller)),
      last_cls_(""), last_object_depth_(0.0), last_center_x_(config.camera_width / 2), last_p_distance_(0.0)
{
    if (config_.depth_scale_override > 0) {
        // Potentially override camera's actual depth scale if a specific value is needed for calculations.
        // For librealsense, the depth scale is obtained directly from the sensor.
        // This config option might be more relevant if the original Python code had a fixed override.
    }

    // Setup logging and video
    if (!setupLoggingAndVideo()) {
        throw std::runtime_error("Failed to set up logging and video recording.");
    }
    log("PalletTracker initialized.");
}

bool PalletTracker::setupLoggingAndVideo() {
    namespace fs = std::filesystem;
    start_time_ = std::chrono::high_resolution_clock::now();

    // Create run directory
    try {
        if (!fs::exists(config_.log_dir)) {
            fs::create_directories(config_.log_dir);
        }
    } catch (const fs::filesystem_error& e) {
        std::cerr << "Error creating log directory: " << e.what() << std::endl;
        return false;
    }

    // Open log file
    std::string log_file_path = config_.log_dir + "/log.txt";
    log_file_.open(log_file_path, std::ios_base::out | std::ios_base::app);
    if (!log_file_.is_open()) {
        std::cerr << "Error opening log file: " << log_file_path << std::endl;
        return false;
    }

    // Setup video writer
    std::string video_path = config_.log_dir + "/output.mp4";
    int fourcc = cv::VideoWriter::fourcc('M', 'P', '4', 'V');
    video_writer_.open(video_path, fourcc, config_.camera_fps, cv::Size(config_.camera_width, config_.camera_height));
    if (!video_writer_.isOpened()) {
        std::cerr << "Error opening video writer: " << video_path << std::endl;
        // Not critical, can continue without video if desired
    }

    log("Starting log and video saving in '" + config_.log_dir + "' directory.");
    cv::namedWindow(config_.window_name, cv::WINDOW_NORMAL);
    return true;
}

void PalletTracker::cleanup() {
    log("Exiting program. Cleaning up resources.");
    if (video_writer_.isOpened()) {
        video_writer_.release();
    }
    cv::destroyAllWindows();
    if (log_file_.is_open()) {
        log_file_.close();
    }
    // RealSense pipeline is stopped in RealSenseManager's destructor.
}

bool PalletTracker::run() {
    bool target_reached = false;
    cv::Mat color_image, depth_image;

    try {
        while (!target_reached && cv::waitKey(1) != 'q') { // 'q' key for exit
            if (!camera_.getFrames(color_image, depth_image)) {
                log("Failed to get camera frames.");
                continue;
            }

            std::vector<Vision::Detection> detections = detector_.detect(color_image);

            if (detections.empty()) {
                if (video_writer_.isOpened()) {
                    video_writer_.write(color_image);
                }
                cv::imshow(config_.window_name, color_image);
                continue;
            }

            // Extract best detection (highest confidence)
            auto best_det_it = std::max_element(detections.begin(), detections.end(),
                                               [](const Vision::Detection& a, const Vision::Detection& b) {
                                                   return a.confidence < b.confidence;
                                               });
            Vision::Detection best_det = *best_det_it;

            int x1i = best_det.box.x;
            int y1i = best_det.box.y;
            int x2i = best_det.box.x + best_det.box.width;
            int y2i = best_det.box.y + best_det.box.height;

            int raw_center_x = best_det.box.x + best_det.box.width / 2;
            int center_y = best_det.box.y + best_det.box.height / 2;

            // Apply Kalman Filter
            kf_x_.predict();
            last_center_x_ = static_cast<int>(kf_x_.update(raw_center_x));

            // Get depth and other info
            // Ensure ROI is valid
            cv::Rect roi(x1i, y1i, x2i - x1i, y2i - y1i);
            roi = roi & cv::Rect(0, 0, depth_image.cols, depth_image.rows); // Clamp ROI to image bounds

            std::vector<double> roi_depth_values;
            if (!roi.empty()) {
                cv::Mat roi_depth_mat = depth_image(roi);
                roi_depth_values.reserve(roi_depth_mat.total());
                for (int r = 0; r < roi_depth_mat.rows; ++r) {
                    for (int c = 0; c < roi_depth_mat.cols; ++c) {
                        uint16_t depth_val_raw = roi_depth_mat.at<uint16_t>(r, c);
                        if (depth_val_raw > 0) { // Filter out invalid depth readings
                            roi_depth_values.push_back(static_cast<double>(depth_val_raw) * camera_.getDepthScale() * 100.0); // Convert to cm
                        }
                    }
                }
            }

            last_object_depth_ = calculateMedian(roi_depth_values);
            // If median is 0 (e.g., all filtered out or empty ROI), set to inf for control logic
            if (last_object_depth_ == 0.0 && !roi_depth_values.empty()) {
                 last_object_depth_ = std::numeric_limits<double>::infinity();
            } else if (roi_depth_values.empty()) {
                last_object_depth_ = std::numeric_limits<double>::infinity();
            }


            // Placeholder for class name based on ID (YOLO model should provide this)
            // For now, assuming single class pallet or similar.
            // Python original had model.names[int(class_id)]
            // This requires detector_ to provide class names, or hardcode it.
            // Temporarily hardcoding for demonstration:
            last_cls_ = "pallet"; // Replace with actual class name lookup if available

            last_p_distance_ = AutonomousForklift::Calculations::calc_p_distance(last_center_x_, last_object_depth_);

            // Visualization
            std::string label = last_cls_ + ": " + std::to_string(static_cast<int>(last_object_depth_)) + "cm";
            cv::rectangle(color_image, best_det.box, cv::Scalar(30, 119, 252), 2); // BGR for OpenCV
            cv::putText(color_image, label, cv::Point(x1i, std::max(0, y1i - 10)), cv::FONT_HERSHEY_SIMPLEX, 0.9, cv::Scalar(30, 119, 252), 2);
            cv::circle(color_image, cv::Point(raw_center_x, center_y), 7, cv::Scalar(0, 0, 255), -1); // Raw (Red)
            cv::circle(color_image, cv::Point(last_center_x_, center_y), 7, cv::Scalar(0, 255, 0), -1); // Filtered (Green)

            if (video_writer_.isOpened()) {
                video_writer_.write(color_image);
            }
            cv::imshow(config_.window_name, color_image);

            log("Detected: " + last_cls_ + " (Raw CX: " + std::to_string(raw_center_x) + ", Filtered CX: " + std::to_string(last_center_x_) + ", Depth: " + std::to_string(static_cast<int>(last_object_depth_)) + "cm)");

            // Check for Goal Completion
            // Note: Python's 'neutral' class would need to be mapped to class_id if used.
            // Assuming 190-200 depth for 'neutral' means it's ready for forking.
            if (last_object_depth_ >= config_.target_depth_min_cm && last_object_depth_ <= config_.target_depth_max_cm &&
                std::abs(last_center_x_ - (config_.camera_width / 2)) <= config_.target_center_tolerance) {
                target_reached = true;
                log("--- Final target reached! Terminating all control sequences. ---");
                continue;
            }

            // Robot Control Logic (using filtered_center_x)
            if (!target_reached) {
                // Dynamic tolerance calculation constants
                const double MIN_TOLERANCE = 5.0; // Corresponds to Python's MIN_TOLERANCE
                const double MAX_TOLERANCE = 50.0; // Corresponds to Python's MAX_TOLERANCE
                const double MIN_DEPTH_CM = 200.0; // Corresponds to Python's MIN_DEPTH
                const double MAX_DEPTH_CM = 500.0; // Corresponds to Python's MAX_DEPTH

                // Calculate dynamic tolerance for centering
                double ratio = 0.0;
                if (MAX_DEPTH_CM - MIN_DEPTH_CM > 0) {
                    ratio = std::min(1.0, std::max(0.0, (last_object_depth_ - MIN_DEPTH_CM) / (MAX_DEPTH_CM - MIN_DEPTH_CM)));
                }
                double center_tolerance = MIN_TOLERANCE + ratio * (MAX_TOLERANCE - MIN_TOLERANCE);
                log("Dynamic tolerance: " + std::to_string(static_cast<int>(center_tolerance)) + " pixels");

                double left_bound = (config_.camera_width / 2) - center_tolerance;
                double right_bound = (config_.camera_width / 2) + center_tolerance;

                if (last_center_x_ >= left_bound && last_center_x_ <= right_bound) {
                    if (last_object_depth_ > MIN_DEPTH_CM) { // Still too far, move forward
                        double move_distance = std::max(0.0, last_object_depth_ - 245.0); // 245cm is magic number from python code
                        forklift_controller_.goForward(move_distance);
                    }
                } else {
                    if (last_center_x_ < left_bound) {
                        forklift_controller_.turnCCW(5); // Example: turn by a fixed angle
                    } else if (last_center_x_ > right_bound) {
                        forklift_controller_.turnCW(5);
                    }
                }
                log("--- Control logic executed ---");
            }
        }
    } catch (const std::exception& e) {
        log("[CRITICAL] Exception in main loop: " + std::string(e.what()));
        cleanup(); // Ensure cleanup on critical error
        return false;
    }

    cleanup();
    return target_reached;
}

} // namespace AutonomousForklift
