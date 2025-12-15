#ifndef AUTONOMOUS_FORKLIFT_PALLET_FORKER_H
#define AUTONOMOUS_FORKLIFT_PALLET_FORKER_H

#include "RealSenseManager.h"
#include "YoloDetector.h"
#include "ForkliftController.h"

#include <memory>
#include <string>
#include <opencv2/opencv.hpp>
#include <chrono>
#include <fstream>
#include <iomanip>
#include <numeric> // For std::accumulate

namespace AutonomousForklift {

class PalletForker {
public:
    struct Config {
        std::string yolo_model_path;
        int camera_width;
        int camera_height;
        int camera_fps;
        float yolo_conf_threshold;
        float yolo_nms_threshold;
        float focal_length_mm;
        float sensor_height_mm; // For focal length in pixels calculation
        std::string log_dir;
        std::string window_name;
        int pick_palette_idx; // e.g., 5th palette from bottom
        int go_forward_cm; // distance to move forward after fork up
        int go_backward_cm; // distance to move backward after fork up
        int slight_fork_up_cm; // distance to slightly lift fork after moving forward
    };

    /**
     * @brief Constructor for PalletForker.
     * @param config Configuration parameters for the forker.
     * @param can_controller Shared pointer to the CAN controller.
     */
    PalletForker(const Config& config, std::shared_ptr<Can::ICanController> can_controller);

    /**
     * @brief Runs the main pallet forking sequence.
     * @return True if the forking sequence completed, false otherwise.
     */
    bool run();

private:
    Config config_;
    Sensors::RealSenseManager camera_;
    Vision::YoloDetector detector_;
    Control::ForkliftController forklift_controller_;

    // Logging
    std::ofstream log_file_;
    std::chrono::high_resolution_clock::time_point start_time_;
    cv::VideoWriter video_writer_;

    float focal_length_px_; // Calculated focal length in pixels

    bool fork_action_taken_; // Flag to ensure forking action is performed once

    /**
     * @brief Initializes logging and video recording.
     * @return True on success, false on failure.
     */
    bool setupLoggingAndVideo();

    /**
     * @brief Cleans up resources.
     */
    void cleanup();

    /**
     * @brief Logs a message to file and console.
     * @param message The message to log.
     */
    void log(const std::string& message);
};

} // namespace AutonomousForklift

#endif // AUTONOMOUS_FORKLIFT_PALLET_FORKER_H
