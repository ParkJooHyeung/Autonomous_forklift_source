#include "BaseController.h"
#include <iostream>
#include <filesystem>
#include <iomanip> // For std::setprecision

namespace AutonomousForklift {

namespace fs = std::filesystem;

BaseController::BaseController(const BaseConfig& config, std::shared_ptr<Control::ForkliftController> forklift_controller, const std::string& mode_name)
    : config_(config),
      camera_(config.camera_width, config.camera_height, config.camera_fps),
      detector_(config.yolo_model_path, config.yolo_conf_threshold, config.yolo_nms_threshold),
      forklift_controller_(std::move(forklift_controller))
{
    if (!setupLoggingAndVideo(mode_name)) {
        throw std::runtime_error("Failed to set up logging and video for " + mode_name);
    }
}

BaseController::~BaseController() {
    cleanup();
}

bool BaseController::setupLoggingAndVideo(const std::string& mode_name) {
    start_time_ = std::chrono::high_resolution_clock::now();

    // Create a unique directory for this run
    auto t = std::time(nullptr);
    auto tm = *std::localtime(&t);
    std::ostringstream oss;
    oss << config_.log_dir_base << "_" << mode_name << "_" << std::put_time(&tm, "%Y-%m-%d_%H-%M-%S");
    current_run_dir_ = oss.str();

    try {
        if (!fs::exists(current_run_dir_)) {
            fs::create_directories(current_run_dir_);
        }
    } catch (const fs::filesystem_error& e) {
        std::cerr << "Error creating run directory: " << e.what() << std::endl;
        return false;
    }

    // Open log file
    std::string log_file_path = current_run_dir_ + "/log.txt";
    log_file_.open(log_file_path, std::ios_base::out | std::ios_base::app);
    if (!log_file_.is_open()) {
        std::cerr << "Error opening log file: " << log_file_path << std::endl;
        return false;
    }

    // Setup video writer
    std::string video_path = current_run_dir_ + "/output.mp4";
    int fourcc = cv::VideoWriter::fourcc('M', 'P', '4', 'V');
    video_writer_.open(video_path, fourcc, config_.camera_fps, cv::Size(config_.camera_width, config_.camera_height));
    if (!video_writer_.isOpened()) {
        std::cerr << "Could not open the output video file for write: " << video_path << std::endl;
        // Continue without video, but log the error.
        log("Video recording is disabled.");
    }

    log("Logging and video initialized for " + mode_name + " in " + current_run_dir_);
    cv::namedWindow(config_.window_name, cv::WINDOW_NORMAL);
    return true;
}

void BaseController::cleanup() {
    log("Cleaning up resources.");
    if (video_writer_.isOpened()) {
        video_writer_.release();
    }
    cv::destroyAllWindows();
    if (log_file_.is_open()) {
        log_file_.close();
    }
}

void BaseController::log(const std::string& message) {
    std::cout << message << std::endl;
    if (log_file_.is_open()) {
        auto now = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> diff = now - start_time_;
        log_file_ << std::fixed << std::setprecision(3) << diff.count() << "s - " << message << std::endl;
    }
}

} // namespace AutonomousForklift
