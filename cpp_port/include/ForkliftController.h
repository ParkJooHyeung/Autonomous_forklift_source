#ifndef AUTONOMOUS_FORKLIFT_FORKLIFT_CONTROLLER_H
#define AUTONOMOUS_FORKLIFT_FORKLIFT_CONTROLLER_H

#include "ICanController.h"
#include "calculations.h" // For calc_distance2time, calc_angle2time
#include <memory>
#include <string>
#include <map>
#include <thread> // For std::this_thread::sleep_for
#include <chrono> // For std::chrono

namespace AutonomousForklift {
namespace Control {

class ForkliftController {
public:
    /**
     * @brief Constructor for ForkliftController.
     * @param can_controller A shared pointer to an ICanController instance.
     */
    ForkliftController(std::shared_ptr<Can::ICanController> can_controller);

    /**
     * @brief Moves the forklift forward for a specified distance.
     * @param cm The distance in centimeters.
     */
    void goForward(double cm);

    /**
     * @brief Turns the forklift clockwise for a specified angle.
     * @param angle The angle in degrees.
     */
    void turnCW(double angle);

    /**
     * @brief Turns the forklift counter-clockwise for a specified angle.
     * @param angle The angle in degrees.
     */
    void turnCCW(double angle);

    /**
     * @brief Moves the forklift backward for a specified distance.
     * @param cm The distance in centimeters.
     */
    void goBackward(double cm);

    /**
     * @brief Moves the forklift right for a specified pseudo-distance.
     *        This involves a sequence of turn and forward movements.
     * @param p_distance The pseudo-distance.
     */
    void moveRight(double p_distance);

    /**
     * @brief Moves the forklift left for a specified pseudo-distance.
     *        This involves a sequence of turn and forward movements.
     * @param p_distance The pseudo-distance.
     */
    void moveLeft(double p_distance);

    /**
     * @brief Lifts the forklift forks for a specified distance.
     * @param cm The distance in centimeters.
     */
    void forkUp(double cm);

    /**
     * @brief Lowers the forklift forks for a specified distance.
     * @param cm The distance in centimeters.
     */
    void forkDown(double cm);

    /**
     * @brief Moves the forklift forks forward for a specified distance.
     * @param cm The distance in centimeters.
     */
    void forkForward(double cm);

    /**
     * @brief Moves the forklift forks backward for a specified distance.
     * @param cm The distance in centimeters.
     */
    void forkBackward(double cm);

private:
    std::shared_ptr<Can::ICanController> can_controller_;
    std::map<std::string, Can::CanMessage> command_frames_;

    /**
     * @brief Initializes the command frames map with predefined CAN messages.
     *        These messages control various forklift actions.
     */
    void initializeCommandFrames();

    /**
     * @brief Executes a continuous CAN command for a specified duration.
     * @param cmd_name The name of the command (e.g., "forward", "turn cw").
     * @param duration_sec The duration in seconds to execute the command.
     */
    void executeTimedCommand(const std::string& cmd_name, double duration_sec);

    /**
     * @brief Sends a stop command to the forklift.
     */
    void sendStopCommand();
};

} // namespace Control
} // namespace AutonomousForklift

#endif // AUTONOMOUS_FORKLIFT_FORKLIFT_CONTROLLER_H
