#include "ForkliftController.h"
#include <iostream>

namespace AutonomousForklift {
namespace Control {

ForkliftController::ForkliftController(std::shared_ptr<Can::ICanController> can_controller)
    : can_controller_(std::move(can_controller)) {
    if (!can_controller_) {
        throw std::runtime_error("ICanController is null.");
    }
    can_controller_->initialize();
    initializeCommandFrames();

    // Start the command processing thread
    stop_thread_ = false;
    command_thread_ = std::thread(&ForkliftController::commandLoop, this);
}

ForkliftController::~ForkliftController() {
    stop_thread_ = true;
    if (command_thread_.joinable()) {
        command_thread_.join();
    }
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

void ForkliftController::commandLoop() {
    Can::CanMessage frame1 = {0x02E3, {0x42, 0x00, 0x00, 0x0A, 0x0A, 0x40, 0x69, 0x93}, false};

    while (!stop_thread_) {
        std::unique_lock<std::mutex> lock(command_mutex_);
        if (active_command_) {
            if (std::chrono::high_resolution_clock::now() < active_command_->end_time) {
                can_controller_->sendMessage(frame1);
                can_controller_->sendMessage(active_command_->can_frame);
            } else {
                // Command finished, stop the forklift and reset state
                sendStopCommand();
                active_command_.reset();
                command_active_ = false;
            }
        }
        lock.unlock();
        // Sleep to avoid busy-waiting and to match the original Python async loop timing
        std::this_thread::sleep_for(std::chrono::milliseconds(60));
    }
}

void ForkliftController::update() {
    // This method is now effectively handled by the commandLoop thread.
    // It's kept for API compatibility, but the core logic has moved.
    // In a more complex scenario, this could be used to poll status or feed a watchdog.
}

bool ForkliftController::isBusy() const {
    return command_active_;
}

void ForkliftController::executeCommandAsync(std::string cmd_name, double duration_sec) {
    if (isBusy()) {
        std::cout << "[ForkliftController] Warning: A command is already active. Ignoring new command '" << cmd_name << "'." << std::endl;
        return;
    }

    auto it = command_frames_.find(cmd_name);
    if (it == command_frames_.end()) {
        std::cerr << "Error: Unknown command '" << cmd_name << "'" << std::endl;
        return;
    }

    std::cout << "[ForkliftController] Starting command '" << cmd_name << "' for " << duration_sec << " seconds." << std::endl;

    std::lock_guard<std::mutex> lock(command_mutex_);
    active_command_ = std::make_unique<ActiveCommand>(
        ActiveCommand{
            cmd_name,
            std::chrono::high_resolution_clock::now() + std::chrono::milliseconds(static_cast<long long>(duration_sec * 1000)),
            it->second
        }
    );
    command_active_ = true;
}

void ForkliftController::sendStopCommand() {
    auto it = command_frames_.find("stop");
    if (it != command_frames_.end()) {
        can_controller_->sendMessage(it->second);
        // Sending stop command twice for robustness, similar to original implementation.
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        can_controller_->sendMessage(it->second);
        std::cout << "[ForkliftController] Sent stop command." << std::endl;
    }
}

void ForkliftController::stop() {
    std::lock_guard<std::mutex> lock(command_mutex_);
    if(active_command_){
        sendStopCommand();
        active_command_.reset();
        command_active_ = false;
        std::cout << "[ForkliftController] Command interrupted by stop()." << std::endl;
    }
}


// --- Public Command Methods ---
void ForkliftController::goForward(double cm) {
    double sec = AutonomousForklift::Calculations::calc_distance2time(cm);
    executeCommandAsync("forward", sec);
}

void ForkliftController::turnCW(double angle) {
    double sec = AutonomousForklift::Calculations::calc_angle2time(angle);
    executeCommandAsync("turn cw", sec);
}

void ForkliftController::turnCCW(double angle) {
    double sec = AutonomousForklift::Calculations::calc_angle2time(angle);
    executeCommandAsync("turn ccw", sec);
}

void ForkliftController::goBackward(double cm) {
    double sec = AutonomousForklift::Calculations::calc_distance2time(cm);
    executeCommandAsync("backward", sec);
}

// NOTE: moveRight and moveLeft remain blocking as they are sequences of commands.
// A more advanced implementation would use a state machine or command queue.
// For now, we will leave them as they are to limit the scope of this refactoring.
void ForkliftController::moveRight(double p_distance) {
    if (isBusy()) {
        std::cout << "[ForkliftController] Cannot execute moveRight while another command is active." << std::endl;
        return;
    }
    turnCW(30);
    std::this_thread::sleep_for(std::chrono::seconds(3)); // This is still blocking
    goForward(p_distance);
    std::this_thread::sleep_for(std::chrono::seconds(3));
    turnCCW(30);
}

void ForkliftController::moveLeft(double p_distance) {
     if (isBusy()) {
        std::cout << "[ForkliftController] Cannot execute moveLeft while another command is active." << std::endl;
        return;
    }
    turnCCW(30);
    std::this_thread::sleep_for(std::chrono::seconds(3)); // This is still blocking
    goForward(p_distance);
    std::this_thread::sleep_for(std::chrono::seconds(3));
    turnCW(30);
}


void ForkliftController::forkUp(double cm) {
    double sec = AutonomousForklift::Calculations::calc_distance2time(cm);
    executeCommandAsync("fork_up", sec);
}

void ForkliftController::forkDown(double cm) {
    double sec = AutonomousForklift::Calculations::calc_distance2time(cm);
    executeCommandAsync("fork_down", sec);
}

void ForkliftController::forkForward(double cm) {
    double sec = AutonomousForklift::Calculations::calc_distance2time(cm);
    executeCommandAsync("fork_forward", sec);
}

void ForkliftController::forkBackward(double cm) {
    double sec = AutonomousForklift::Calculations::calc_distance2time(cm);
    executeCommandAsync("fork_backward", sec);
}

} // namespace Control
} // namespace AutonomousForklift