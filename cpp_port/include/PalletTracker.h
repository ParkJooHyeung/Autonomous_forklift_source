#ifndef AUTONOMOUS_FORKLIFT_PALLET_TRACKER_H
#define AUTONOMOUS_FORKLIFT_PALLET_TRACKER_H

#include "RealSenseManager.h"
#include "YoloDetector.h"
#include "KalmanFilterWrapper.h"
#include "ForkliftController.h"
#include "calculations.h" // For calc_p_distance

#include <memory>
#include <string>
#include <opencv2/opencv.hpp>
#include <chrono> // For logging timestamps
#include <fstream> // For file logging
#include <iomanip> // For std::fixed, std::setprecision

namespace AutonomousForklift {

class PalletTracker {
public:
    struct Config {
        std::string yolo_model_path;
        int camera_width;
        int camera_height;
        int camera_fps;
        float yolo_conf_threshold;
        float yolo_nms_threshold;
        double kf_r_val;
        double kf_q_var;
        float depth_scale_override; // If 0, use camera's actual depth scale
        int target_depth_min_cm;
        int target_depth_max_cm;
        int target_center_tolerance;
        std::string log_dir;
        std::string window_name;
    };

    /**
     * @brief Constructor for PalletTracker.
     * @param config Configuration parameters for the tracker.
     * @param can_controller Shared pointer to the CAN controller.
     */
    PalletTracker(const Config& config, std::shared_ptr<Can::ICanController> can_controller);

    /**
     * @brief Runs the main pallet tracking and control loop.
     * @return True if the target was reached, false otherwise (e.g., program exited early).
     */
    bool run();

private:
    Config config_;
    Sensors::RealSenseManager camera_;
    Vision::YoloDetector detector_;
    Filters::KalmanFilterWrapper kf_x_;
    Control::ForkliftController forklift_controller_;

    // Last known state variables
    std::string last_cls_;
    double last_object_depth_;
    int last_center_x_;
    double last_p_distance_;

    // Logging
    std::ofstream log_file_;
    std::chrono::high_resolution_clock::time_point start_time_;
    cv::VideoWriter video_writer_;

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
     * @brief Calculates median of a vector of doubles.
     * @param data Input vector.
     * @return Median value.
     */
    double calculateMedian(std::vector<double>& data);

    /**
     * @brief Logs a message to file and console.
     * @param message The message to log.
     */
    void log(const std::string& message);
};

} // namespace AutonomousForklift

#endif // AUTONOMOUS_FORKLIFT_PALLET_TRACKER_H
