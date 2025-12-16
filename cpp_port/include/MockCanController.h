#ifndef AUTONOMOUS_FORKLIFT_MOCK_CAN_CONTROLLER_H
#define AUTONOMOUS_FORKLIFT_MOCK_CAN_CONTROLLER_H

#include "ICanController.h"
#include <iostream>
#include <iomanip> // For std::hex, std::setfill, std::setw
#include <cmath>   // For M_PI, cos, sin
#include <map>     // For decoding messages

namespace AutonomousForklift {
namespace Can {

class MockCanController : public ICanController {
public:
    MockCanController() {
        // Define command data vectors for decoding
        command_map_[{0x7f, 0x60, 0x7f, 0x7f, 0x7f, 0x7f, 0x7f, 0x7f}] = "turn cw";
        command_map_[{0x7f, 0x9e, 0x7f, 0x7f, 0x7f, 0x7f, 0x7f, 0x7f}] = "turn ccw";
        command_map_[{0x7f, 0x7f, 0x44, 0x7f, 0x7f, 0x7f, 0x7f, 0x7f}] = "forward";
        command_map_[{0x7f, 0x7f, 0xbb, 0x7f, 0x7f, 0x7f, 0x7f, 0x7f}] = "backward";
        command_map_[{0xbb, 0x7f, 0x7f, 0x7f, 0x7f, 0x7f, 0x7f, 0x7f}] = "fork_up";
        command_map_[{0x44, 0x7f, 0x7f, 0x7f, 0x7f, 0x7f, 0x7f, 0x7f}] = "fork_down";
        // Ignoring fork_forward, fork_backward, stop for this simulation
    }
    ~MockCanController() override = default;

    bool initialize() override {
        std::cout << "[MockCanController] Initializing Simulation..." << std::endl;
        pose_ = {0.0, 0.0, 90.0, 0.0}; // Start at (0,0), facing up (90 degrees)
        return true;
    }

    bool sendMessage(const CanMessage& message) override {
        // We only care about the command frame with id 0x01E3
        if (message.id != 0x01E3) {
            return true;
        }

        auto it = command_map_.find(message.data);
        if (it == command_map_.end()) {
            // It's a stop command or unknown, so we don't move
            return true;
        }

        const std::string& cmd = it->second;
        const double time_delta_s = 0.060; // The loop interval from ForkliftController

        // Simulated speeds
        const double LINEAR_SPEED_M_S = 0.5; // 0.5 meters per second
        const double ANGULAR_SPEED_DEG_S = 15.0; // 15 degrees per second
        const double FORK_SPEED_CM_S = 5.0; // 5 cm per second

        if (cmd == "forward") {
            double dist = LINEAR_SPEED_M_S * time_delta_s;
            double angle_rad = pose_.angle_deg * (M_PI / 180.0);
            pose_.x_m += dist * cos(angle_rad);
            pose_.y_m += dist * sin(angle_rad);
        } else if (cmd == "backward") {
            double dist = LINEAR_SPEED_M_S * time_delta_s;
            double angle_rad = pose_.angle_deg * (M_PI / 180.0);
            pose_.x_m -= dist * cos(angle_rad);
            pose_.y_m -= dist * sin(angle_rad);
        } else if (cmd == "turn cw") {
            pose_.angle_deg -= ANGULAR_SPEED_DEG_S * time_delta_s;
        } else if (cmd == "turn ccw") {
            pose_.angle_deg += ANGULAR_SPEED_DEG_S * time_delta_s;
        } else if (cmd == "fork_up") {
            pose_.fork_height_cm += FORK_SPEED_CM_S * time_delta_s;
        } else if (cmd == "fork_down") {
            pose_.fork_height_cm -= FORK_SPEED_CM_S * time_delta_s;
        }
        
        // Normalize angle to be within [0, 360)
        pose_.angle_deg = fmod(fmod(pose_.angle_deg, 360.0) + 360.0, 360.0);

        // Print the updated pose
        std::cout << std::fixed << std::setprecision(2)
                  << "[Simulated Pose] X: " << pose_.x_m << "m, Y: " << pose_.y_m
                  << "m, Angle: " << pose_.angle_deg << "deg, Fork Height: " << pose_.fork_height_cm << "cm" << std::endl;

        return true;
    }

    bool readMessage(CanMessage& message) override {
        return false; // No incoming messages simulated for now
    }

    RobotPose getRobotPose() const override {
        return pose_;
    }

private:
    RobotPose pose_;
    std::map<std::vector<uint8_t>, std::string> command_map_;
};

} // namespace Can
} // namespace AutonomousForklift

#endif // AUTONOMOUS_FORKLIFT_MOCK_CAN_CONTROLLER_H