#include "ForkliftController.h"
#include <iostream>

namespace AutonomousForklift {
namespace Control {

ForkliftController::ForkliftController(std::shared_ptr<Can::ICanController> can_controller)
    : can_controller_(std::move(can_controller)) {
    if (!can_controller_) {
        std::cerr << "Error: ICanController is null." << std::endl;
        // Depending on desired error handling, throw exception or handle gracefully.
    }
    can_controller_->initialize(); // Initialize the CAN controller
    initializeCommandFrames();
}

void ForkliftController::initializeCommandFrames() {
    // These are translated from the Python frame2_data dictionary
    command_frames_["turn cw"] = {0x01E3, {0x7f, 0x60, 0x7f, 0x7f, 0x7f, 0x7f, 0x7f, 0x7f}, false};
    command_frames_["turn ccw"] = {0x01E3, {0x7f, 0x9e, 0x7f, 0x7f, 0x7f, 0x7f, 0x7f, 0x7f}, false};
    command_frames_["forward"] = {0x01E3, {0x7f, 0x7f, 0x44, 0x7f, 0x7f, 0x7f, 0x7f, 0x7f}, false};
    command_frames_["backward"] = {0x01E3, {0x7f, 0x7f, 0xbb, 0x7f, 0x7f, 0x7f, 0x7f, 0x7f}, false};
    command_frames_["stop"] = {0x01E3, {0x7f, 0x7f, 0x7f, 0x7f, 0x7f, 0x7f, 0x7f, 0x7f}, false};
    command_frames_["fork_up"] = {0x01E3, {0xbb, 0x7f, 0x7f, 0x7f, 0x7f, 0x7f, 0x7f, 0x7f}, false};
    command_frames_["fork_down"] = {0x01E3, {0x44, 0x7f, 0x7f, 0x7f, 0x7f, 0x7f, 0x7f, 0x7f}, false};
    command_frames_["fork_forward"] = {0x01E3, {0x7f, 0x7f, 0x7f, 0x7f, 0x7f, 0x44, 0x7f, 0x7f}, false};
    command_frames_["fork_backward"] = {0x01E3, {0x7f, 0x7f, 0x7f, 0x7f, 0x7f, 0xbb, 0x7f, 0x7f}, false};
}

void ForkliftController::executeTimedCommand(const std::string& cmd_name, double duration_sec) {
    auto it = command_frames_.find(cmd_name);
    if (it == command_frames_.end()) {
        std::cerr << "Error: Unknown command '" << cmd_name << "'" << std::endl;
        return;
    }

    Can::CanMessage command_frame = it->second;
    Can::CanMessage frame1 = {0x02E3, {0x42, 0x00, 0x00, 0x0A, 0x0A, 0x40, 0x69, 0x93}, false}; // From Python long_running_task

    std::cout << "[ForkliftController] Executing command '" << cmd_name << "' for " << duration_sec << " seconds." << std::endl;

    auto start_time = std::chrono::high_resolution_clock::now();
    auto end_time = start_time + std::chrono::duration<double>(duration_sec);

    while (std::chrono::high_resolution_clock::now() < end_time) {
        can_controller_->sendMessage(frame1);
        can_controller_->sendMessage(command_frame);
        std::this_thread::sleep_for(std::chrono::milliseconds(60)); // Equivalent to async_time_buffer(0.06)
    }
    sendStopCommand();
    std::cout << "[ForkliftController] Command '" << cmd_name << "' finished." << std::endl;
}

void ForkliftController::sendStopCommand() {
    auto it = command_frames_.find("stop");
    if (it != command_frames_.end()) {
        can_controller_->sendMessage(it->second);
        // It's good practice to send the stop frame multiple times for robustness
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        can_controller_->sendMessage(it->second);
        std::cout << "[ForkliftController] Sent stop command." << std::endl;
    }
}

void ForkliftController::goForward(double cm) {
    double sec = AutonomousForklift::Calculations::calc_distance2time(cm);
    executeTimedCommand("forward", sec);
}

void ForkliftController::turnCW(double angle) {
    double sec = AutonomousForklift::Calculations::calc_angle2time(angle);
    executeTimedCommand("turn cw", sec);
}

void ForkliftController::turnCCW(double angle) {
    double sec = AutonomousForklift::Calculations::calc_angle2time(angle);
    executeTimedCommand("turn ccw", sec);
}

void ForkliftController::goBackward(double cm) {
    double sec = AutonomousForklift::Calculations::calc_distance2time(cm);
    executeTimedCommand("backward", sec);
}

void ForkliftController::moveRight(double p_distance) {
    turnCW(30);
    std::this_thread::sleep_for(std::chrono::seconds(3));
    goForward(p_distance);
    std::this_thread::sleep_for(std::chrono::seconds(3));
    turnCCW(30);
    std::this_thread::sleep_for(std::chrono::seconds(3));
}

void ForkliftController::moveLeft(double p_distance) {
    turnCCW(30);
    std::this_thread::sleep_for(std::chrono::seconds(3));
    goForward(p_distance);
    std::this_thread::sleep_for(std::chrono::seconds(3));
    turnCW(30);
    std::this_thread::sleep_for(std::chrono::seconds(3));
}

void ForkliftController::forkUp(double cm) {
    double sec = AutonomousForklift::Calculations::calc_distance2time(cm);
    executeTimedCommand("fork_up", sec);
    std::cout << "--- Fork lifted by " << cm << "cm. (Ran for " << sec << " seconds) ---" << std::endl;
}

void ForkliftController::forkDown(double cm) {
    double sec = AutonomousForklift::Calculations::calc_distance2time(cm);
    executeTimedCommand("fork_down", sec);
    std::cout << "--- Fork lowered by " << cm << "cm. (Ran for " << sec << " seconds) ---" << std::endl;
}

void ForkliftController::forkForward(double cm) {
    double sec = AutonomousForklift::Calculations::calc_distance2time(cm);
    executeTimedCommand("fork_forward", sec);
    std::cout << "--- Fork moved forward by " << cm << "cm. (Ran for " << sec << " seconds) ---" << std::endl;
}

void ForkliftController::forkBackward(double cm) {
    double sec = AutonomousForklift::Calculations::calc_distance2time(cm);
    executeTimedCommand("fork_backward", sec);
    std::cout << "--- Fork moved backward by " << cm << "cm. (Ran for " << sec << " seconds) ---" << std::endl;
}

} // namespace Control
} // namespace AutonomousForklift
