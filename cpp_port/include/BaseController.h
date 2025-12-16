#ifndef AUTONOMOUS_FORKLIFT_BASE_CONTROLLER_H
#define AUTONOMOUS_FORKLIFT_BASE_CONTROLLER_H

#include "RealSenseManager.h"
#include "YoloDetector.h"
#include "ForkliftController.h"
#include <opencv2/opencv.hpp>
#include <string>
#include <fstream>
#include <chrono>
#include <memory>

namespace AutonomousForklift {

// A generic config struct that can be extended by specific controllers
struct BaseConfig {
    int camera_width = 640;
    int camera_height = 480;
    int camera_fps = 30;
    std::string yolo_model_path;
    float yolo_conf_threshold = 0.5f;
    float yolo_nms_threshold = 0.4f;
    std::string log_dir_base = "run";
    std::string window_name = "Autonomous Forklift";
};

class BaseController {
public:
    BaseController(const BaseConfig& config, std::shared_ptr<Control::ForkliftController> forklift_controller, const std::string& mode_name);
    virtual ~BaseController();

    // Pure virtual function to be implemented by derived classes
    virtual bool run() = 0;

protected:
    bool setupLoggingAndVideo(const std::string& mode_name);
    void cleanup();
    void log(const std::string& message);
    
    // Common components
    BaseConfig config_;
    Sensors::RealSenseManager camera_;
    Vision::YoloDetector detector_;
    std::shared_ptr<Control::ForkliftController> forklift_controller_;

    // Logging and video recording
    std::ofstream log_file_;
    cv::VideoWriter video_writer_;
    std::chrono::high_resolution_clock::time_point start_time_;
    std::string current_run_dir_;
};

} // namespace AutonomousForklift

#endif // AUTONOMOUS_FORKLIFT_BASE_CONTROLLER_H
