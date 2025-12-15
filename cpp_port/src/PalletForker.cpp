#include "PalletForker.h"
#include <iostream>
#include <filesystem>
#include <algorithm> // For std::sort, std::max_element
#include <numeric>   // For std::accumulate

namespace AutonomousForklift {

PalletForker::PalletForker(const Config& config, std::shared_ptr<Can::ICanController> can_controller)
    : config_(config),
      camera_(config.camera_width, config.camera_height, config.camera_fps),
      detector_(config.yolo_model_path, config.yolo_conf_threshold, config.yolo_nms_threshold),
      forklift_controller_(std::move(can_controller)),
      fork_action_taken_(false)
{
    // Calculate focal length in pixels
    // Python original: focal_length_px = (focal_length_mm * active_pixels_height) / sensor_height_mm
    focal_length_px_ = (config_.focal_length_mm * config_.camera_height) / config_.sensor_height_mm;

    // Setup logging and video
    if (!setupLoggingAndVideo()) {
        throw std::runtime_error("Failed to set up logging and video recording for PalletForker.");
    }
    log("PalletForker initialized.");
}

bool PalletForker::setupLoggingAndVideo() {
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
    std::string log_file_path = config_.log_dir + "/forking_log.txt"; // Separate log for forking
    log_file_.open(log_file_path, std::ios_base::out | std::ios_base::app);
    if (!log_file_.is_open()) {
        std::cerr << "Error opening log file: " << log_file_path << std::endl;
        return false;
    }

    // Setup video writer
    std::string video_path = config_.log_dir + "/forking_output.mp4"; // Separate video for forking
    int fourcc = cv::VideoWriter::fourcc('M', 'P', '4', 'V');
    video_writer_.open(video_path, fourcc, config_.camera_fps, cv::Size(config_.camera_width, config_.camera_height));
    if (!video_writer_.isOpened()) {
        std::cerr << "Error opening video writer: " << video_path << std::endl;
        // Not critical, can continue without video if desired
    }

    log("Starting log and video saving in '" + config_.log_dir + "' directory for PalletForker.");
    cv::namedWindow(config_.window_name, cv::WINDOW_NORMAL);
    return true;
}

void PalletForker::cleanup() {
    log("Exiting PalletForker. Cleaning up resources.");
    if (video_writer_.isOpened()) {
        video_writer_.release();
    }
    cv::destroyAllWindows();
    if (log_file_.is_open()) {
        log_file_.close();
    }
}

void PalletForker::log(const std::string& message) {
    std::cout << message << std::endl;
    if (log_file_.is_open()) {
        auto now = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> diff = now - start_time_;
        log_file_ << std::fixed << std::setprecision(3) << diff.count() << "s - " << message << std::endl;
    }
}

bool PalletForker::run() {
    cv::Mat color_image, depth_image;
    double current_time_sec = 0.0;
    const double break_time = 15.0; // Python's 'break_time'

    try {
        while (cv::waitKey(1) != 'q') {
            auto loop_start_time = std::chrono::high_resolution_clock::now();
            current_time_sec = std::chrono::duration<double>(loop_start_time - start_time_).count();

            if (!camera_.getFrames(color_image, depth_image)) {
                log("Failed to get camera frames.");
                continue;
            }

            std::vector<Vision::Detection> detections = detector_.detect(color_image);

            if (detections.empty()) {
                if (video_writer_.isOpened()) { video_writer_.write(color_image); }
                cv::imshow(config_.window_name, color_image);
                continue;
            }

            // Sort detections by their bottom Y-coordinate (y2) in descending order
            // This means the bottom-most palettes come first
            std::sort(detections.begin(), detections.end(), [](const Vision::Detection& a, const Vision::Detection& b) {
                return a.box.y + a.box.height > b.box.y + b.box.height;
            });

            cv::Rect bottom_box;
            if (!detections.empty()) {
                 // The first one after sorting is the bottom-most
                bottom_box = detections[0].box;
            }

            // Visualization & Selection of pick_palette
            if (current_time_sec < break_time) {
                for (size_t idx = 0; idx < detections.size(); ++idx) {
                    const auto& det = detections[idx];
                    std::string label = "pallet"; // Replace with actual class name if detector_ provides them
                    if (idx == (config_.pick_palette_idx - 1)) {
                        cv::rectangle(color_image, det.box, cv::Scalar(0, 0, 235), 2); // Highlight in Red
                        cv::putText(color_image, label, cv::Point(det.box.x, det.box.y - 10), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 235), 2);
                    } else {
                        cv::rectangle(color_image, det.box, cv::Scalar(0, 255, 0), 2); // Others in Green
                        cv::putText(color_image, label, cv::Point(det.box.x, det.box.y - 10), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 2);
                    }
                }
            }

            // Proceed only if enough palettes are detected
            if (detections.size() >= config_.pick_palette_idx && !fork_action_taken_) {
                Vision::Detection pick_palette_det = detections[config_.pick_palette_idx - 1];
                int pick_y1 = pick_palette_det.box.y;
                int pick_y2 = pick_palette_det.box.y + pick_palette_det.box.height;
                int pick_x_center = pick_palette_det.box.x + pick_palette_det.box.width / 2;

                int zero_point_y = bottom_box.y + bottom_box.height;
                int target_point_y = pick_y1 + (pick_y2 - pick_y1) / 2;

                // Get depth value from a small area around the target point
                cv::Rect depth_roi(pick_x_center - 5, target_point_y - 5, 10, 10);
                depth_roi = depth_roi & cv::Rect(0, 0, depth_image.cols, depth_image.rows);
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
                        // Average non-zero depth values
                        depth_value = static_cast<double>(std::accumulate(valid_depths.begin(), valid_depths.end(), 0.0) / valid_depths.size()) * camera_.getDepthScale(); // Convert to meters
                    }
                }

                if (depth_value > 0.0) { // Ensure valid depth reading
                    // Calculate the real-world height relative to the bottom palette
                    // Python: real_world_height = ((zero_point_y - target_point_y) * 1.6 * depth_value) / focal_length_px
                    double real_world_height_m = (static_cast<double>(zero_point_y - target_point_y) * 1.6 * depth_value) / focal_length_px_;
                    double real_world_height_cm = real_world_height_m * 100.0;

                    if (current_time_sec < break_time) {
                        std::string depth_height_label = "Depth: " + std::to_string(static_cast<int>(depth_value * 100)) + "cm, Height: " + std::to_string(static_cast<int>(real_world_height_cm)) + "cm";
                        cv::putText(color_image, depth_height_label, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 2);
                    }

                    // --- EXECUTE FORK UP AND MOVE FORWARD ACTION ---
                    if (real_world_height_cm > 0.0 && !fork_action_taken_) {
                        log("Target height calculated: " + std::to_string(static_cast<int>(real_world_height_cm)) + "cm. Initiating forking sequence.");

                        // 1. Raise the fork to the calculated height
                        forklift_controller_.forkUp(real_world_height_cm);
                        log("fork_up command has been sent.");

                        // 2. Move forward to insert the fork into the pallet
                        forklift_controller_.goForward(config_.go_forward_cm);
                        log("go_forward(" + std::to_string(config_.go_forward_cm) + ") command has been sent.");

                        // 3. Slightly lift the fork to prevent collision with the pallet below.
                        forklift_controller_.forkUp(config_.slight_fork_up_cm);
                        log("fork_up(" + std::to_string(config_.slight_fork_up_cm) + ") command has been sent to lift the pallet slightly.");

                        // 4. Move backward to pull the pallet out from the stack.
                        forklift_controller_.goBackward(config_.go_backward_cm);
                        log("go_backward(" + std::to_string(config_.go_backward_cm) + ") command has been sent.");

                        fork_action_taken_ = true; // Set flag to true to prevent repeated calls
                        return true; // Forking sequence completed
                    }
                } else {
                    log("Invalid depth value obtained. Skipping forking action.");
                }
            } else if (fork_action_taken_) {
                log("Forking action already completed. Exiting forker.");
                return true; // Already forked, so consider it completed
            }

            if (video_writer_.isOpened()) { video_writer_.write(color_image); }
            cv::imshow(config_.window_name, color_image);
        }
    } catch (const std::exception& e) {
        log("[CRITICAL] Exception in PalletForker main loop: " + std::string(e.what()));
        cleanup(); // Ensure cleanup on critical error
        return false;
    }

    cleanup();
    return fork_action_taken_; // Return true if forking was done, false otherwise
}

} // namespace AutonomousForklift
