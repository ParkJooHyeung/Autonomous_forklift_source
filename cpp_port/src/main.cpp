#include <iostream>
#include <memory>
#include <string>
#include <stdexcept>
#include <chrono>
#include <ctime>
#include <iomanip>
#include <filesystem>

// Custom modules
#include "ICanController.h"
#include "MockCanController.h"
#include "PalletTracker.h"
#include "PalletForker.h"

// Function to generate a timestamped directory name
std::string generateTimestampedLogDir() {
    auto now = std::chrono::system_clock::now();
    std::time_t now_c = std::chrono::system_clock::to_time_t(now);
    std::stringstream ss;
    ss << "run_" << std::put_time(std::localtime(&now_c), "%Y-%m-%d_%H-%M-%S");
    return ss.str();
}

int main(int argc, char* argv[]) {
    std::cout << "Starting Autonomous Forklift C++ application..." << std::endl;

    std::string mode = "tracker"; // Default mode

    // Basic command-line argument parsing for mode selection
    if (argc > 1) {
        mode = argv[1];
    }

    if (mode != "tracker" && mode != "forker") {
        std::cerr << "Usage: " << argv[0] << " [tracker|forker]" << std::endl;
        return 1;
    }

    // 1. Initialize CAN Controller (using mock for now)
    std::shared_ptr<AutonomousForklift::Can::ICanController> can_controller =
        std::make_shared<AutonomousForklift::Can::MockCanController>();

    try {
        if (mode == "tracker") {
            std::cout << "Running in Pallet Tracking mode." << std::endl;
            AutonomousForklift::PalletTracker::Config config;

            config.log_dir = generateTimestampedLogDir();

            // IMPORTANT: User needs to provide a path to their YOLOv5 ONNX model.
            // For PalletTracker, this was "241129_palette.pt" in Python.
            config.yolo_model_path = "241129_palette.onnx"; // <<< USER: Update this path!
            if (!std::filesystem::exists(config.yolo_model_path)) {
                std::cerr << "ERROR: YOLO model not found at " << config.yolo_model_path << std::endl;
                std::cerr << "Please ensure your YOLOv5 model is exported to ONNX format and placed at the specified path." << std::endl;
                return 1;
            }

            config.camera_width = 1280;
            config.camera_height = 720;
            config.camera_fps = 30;
            config.yolo_conf_threshold = 0.5f;
            config.yolo_nms_threshold = 0.4f;
            config.kf_r_val = 5.0;
            config.kf_q_var = 0.1;
            config.depth_scale_override = 0.0f; // Use camera's actual depth scale
            config.target_depth_min_cm = 190;
            config.target_depth_max_cm = 200;
            config.target_center_tolerance = 5;
            config.window_name = "RealSense YOLOv5 Tracking (C++ Port)";

            AutonomousForklift::PalletTracker pallet_tracker(config, can_controller);
            bool target_reached = pallet_tracker.run();

            if (target_reached) {
                std::cout << "Pallet tracking completed successfully. Target reached." << std::endl;
            } else {
                std::cout << "Pallet tracking exited without reaching the final target." << std::endl;
            }
        } else if (mode == "forker") {
            std::cout << "Running in Pallet Forking mode." << std::endl;
            AutonomousForklift::PalletForker::Config config;

            config.log_dir = generateTimestampedLogDir();

            // IMPORTANT: User needs to provide a path to their YOLOv5 ONNX model.
            // For PalletForker, this was "240808_krri.pt" in Python.
            config.yolo_model_path = "240808_krri.onnx"; // <<< USER: Update this path!
            if (!std::filesystem::exists(config.yolo_model_path)) {
                std::cerr << "ERROR: YOLO model for forking not found at " << config.yolo_model_path << std::endl;
                std::cerr << "Please ensure your YOLOv5 model is exported to ONNX format and placed at the specified path." << std::endl;
                return 1;
            }

            config.camera_width = 1280;
            config.camera_height = 720;
            config.camera_fps = 60; // Python script used 60fps
            config.yolo_conf_threshold = 0.5f;
            config.yolo_nms_threshold = 0.4f;
            config.focal_length_mm = 1.88f; // From Python script
            config.sensor_height_mm = 2.74f; // From Python script
            config.pick_palette_idx = 5; // From Python script
            config.go_forward_cm = 80;
            config.go_backward_cm = 80;
            config.slight_fork_up_cm = 10;
            config.window_name = "RealSense YOLOv5 Forking (C++ Port)";

            AutonomousForklift::PalletForker pallet_forker(config, can_controller);
            bool forking_completed = pallet_forker.run();

            if (forking_completed) {
                std::cout << "Pallet forking sequence completed successfully." << std::endl;
            } else {
                std::cout << "Pallet forking sequence exited prematurely or failed." << std::endl;
            }
        }
    } catch (const std::exception& e) {
        std::cerr << "Unhandled exception: " << e.what() << std::endl;
        return 1;
    } catch (...) {
        std::cerr << "Unknown unhandled exception occurred." << std::endl;
        return 1;
    }

    std::cout << "Autonomous Forklift application finished." << std::endl;
    return 0;
}
