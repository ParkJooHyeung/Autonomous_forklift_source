#ifndef AUTONOMOUS_FORKLIFT_FORKLIFT_CONTROLLER_H
#define AUTONOMOUS_FORKLIFT_FORKLIFT_CONTROLLER_H

#include "ICanController.h"
#include "calculations.h"
#include <memory>
#include <string>
#include <map>
#include <thread>
#include <chrono>
#include <atomic>
#include <mutex>
#include <functional>

namespace AutonomousForklift {
namespace Control {

class ForkliftController {
public:
    ForkliftController(std::shared_ptr<Can::ICanController> can_controller);
    ~ForkliftController();

    // --- Non-blocking Command Methods ---
    void goForward(double cm);
    void turnCW(double angle);
    void turnCCW(double angle);
    void goBackward(double cm);
    void moveRight(double p_distance);
    void moveLeft(double p_distance);
    void forkUp(double cm);
    void forkDown(double cm);
    void forkForward(double cm);
    void forkBackward(double cm);
    void stop();

    /**
     * @brief This method should be called periodically in the main application loop.
     *        It manages the state of the currently executing command.
     */
    void update();
    
    /**
     * @brief Checks if a command is currently being executed.
     * @return True if a command is running, false otherwise.
     */
    bool isBusy() const;

private:
    // --- Private Helper Methods ---
    void initializeCommandFrames();
    void commandLoop();
    void sendStopCommand();

    /**
     * @brief Initiates a new timed command, returning immediately.
     * @param cmd_name The name of the command (e.g., "forward").
     * @param duration_sec The duration for the command to run.
     */
    void executeCommandAsync(std::string cmd_name, double duration_sec);

    // --- Member Variables ---
    std::shared_ptr<Can::ICanController> can_controller_;
    std::map<std::string, Can::CanMessage> command_frames_;
    
    std::thread command_thread_;
    std::mutex command_mutex_;
    std::atomic<bool> stop_thread_ = false;
    std::atomic<bool> command_active_ = false;

    // Struct to hold the details of the active command
    struct ActiveCommand {
        std::string name;
        std::chrono::time_point<std::chrono::high_resolution_clock> end_time;
        Can::CanMessage can_frame;
    };
    
    // Using a unique_ptr to make the active command optional
    std::unique_ptr<ActiveCommand> active_command_ = nullptr;
};

} // namespace Control
} // namespace AutonomousForklift

#endif // AUTONOMOUS_FORKLIFT_FORKLIFT_CONTROLLER_H