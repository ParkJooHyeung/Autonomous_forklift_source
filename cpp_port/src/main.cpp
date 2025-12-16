#include <iostream>
#include <memory>
#include <string>
#include <stdexcept>
#include <filesystem>
#include <fstream>
#include <nlohmann/json.hpp>

// Custom modules
#include "MockCanController.h"
#include "ForkliftController.h"
#include "PalletTracker.h"
#include "PalletForker.h"
#include "BaseController.h"

using json = nlohmann::json;

int main(int argc, char* argv[]) {
    std::cout << "Starting Autonomous Forklift C++ application..." << std::endl;

    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " [tracker|forker]" << std::endl;
        return 1;
    }
    std::string mode = argv[1];

    // --- Configuration Loading ---
    json config_json;
    try {
        std::ifstream config_file("cpp_port/config.json");
        if (!config_file.is_open()) {
            throw std::runtime_error("Could not open config.json");
        }
        config_file >> config_json;
    } catch (const std::exception& e) {
        std::cerr << "Configuration error: " << e.what() << std::endl;
        return 1;
    }
    
    // --- Controller Initialization ---
    auto can_controller = std::make_shared<AutonomousForklift::Can::MockCanController>();
    auto forklift_controller = std::make_shared<AutonomousForklift::Control::ForkliftController>(can_controller);
    std::unique_ptr<AutonomousForklift::BaseController> controller;

    try {
        const json& mode_config = config_json.at(mode);

        if (mode == "tracker") {
            std::cout << "Running in Pallet Tracking mode." << std::endl;
            
            AutonomousForklift::PalletTracker::Config config;
            config.yolo_model_path = mode_config.at("yolo_model_path");
            config.camera_width = mode_config.at("camera_width");
            config.camera_height = mode_config.at("camera_height");
            config.camera_fps = mode_config.at("camera_fps");
            config.kf_r_val = mode_config.at("kf_r_val");
            config.kf_q_var = mode_config.at("kf_q_var");
            config.target_depth_min_cm = mode_config.at("target_depth_min_cm");
            config.target_depth_max_cm = mode_config.at("target_depth_max_cm");
            config.target_center_tolerance = mode_config.at("target_center_tolerance");
            config.min_tolerance = mode_config.at("min_tolerance");
            config.max_tolerance = mode_config.at("max_tolerance");
            config.min_depth_cm_for_move = mode_config.at("min_depth_cm_for_move");
            config.max_depth_cm_for_move = mode_config.at("max_depth_cm_for_move");
            config.forward_move_target_cm = mode_config.at("forward_move_target_cm");
            config.turn_angle_deg = mode_config.at("turn_angle_deg");

            if (!std::filesystem::exists(config.yolo_model_path)) {
                std::cerr << "ERROR: Tracker YOLO model not found at " << config.yolo_model_path << std::endl;
                return 1;
            }

            controller = std::make_unique<AutonomousForklift::PalletTracker>(config, forklift_controller);

        } else if (mode == "forker") {
            std::cout << "Running in Pallet Forking mode." << std::endl;
            
            AutonomousForklift::PalletForker::Config config;
            config.yolo_model_path = mode_config.at("yolo_model_path");
            config.camera_width = mode_config.at("camera_width");
            config.camera_height = mode_config.at("camera_height");
            config.camera_fps = mode_config.at("camera_fps");
            config.focal_length_mm = mode_config.at("focal_length_mm");
            config.sensor_height_mm = mode_config.at("sensor_height_mm");
            config.pick_palette_idx = mode_config.at("pick_palette_idx");
            config.go_forward_cm = mode_config.at("go_forward_cm");
            config.go_backward_cm = mode_config.at("go_backward_cm");
            config.slight_fork_up_cm = mode_config.at("slight_fork_up_cm");
            config.align_wait_sec = mode_config.at("align_wait_sec");
            
            if (!std::filesystem::exists(config.yolo_model_path)) {
                std::cerr << "ERROR: Forker YOLO model not found at " << config.yolo_model_path << std::endl;
                return 1;
            }

            controller = std::make_unique<AutonomousForklift::PalletForker>(config, forklift_controller);
        
        } else {
            std::cerr << "Invalid mode specified. Use 'tracker' or 'forker'." << std::endl;
            return 1;
        }

        // Run the selected controller
        bool success = controller->run();
        
        if (success) {
            std::cout << "Mode '" << mode << "' completed successfully." << std::endl;
        } else {
            std::cout << "Mode '" << mode << "' exited prematurely or failed." << std::endl;
        }

    } catch (const json::out_of_range& e) {
        std::cerr << "Configuration missing for mode '" << mode << "': " << e.what() << std::endl;
        return 1;
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