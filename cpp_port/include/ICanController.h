#ifndef AUTONOMOUS_FORKLIFT_ICAN_CONTROLLER_H
#define AUTONOMOUS_FORKLIFT_ICAN_CONTROLLER_H

#include <vector>
#include <cstdint> // For uint8_t, etc.

namespace AutonomousForklift {
namespace Can {

// Represents the physical state of the robot
struct RobotPose {
    double x_m = 0.0;       // X position in meters
    double y_m = 0.0;       // Y position in meters
    double angle_deg = 0.0; // Angle in degrees
    double fork_height_cm = 0.0; // Fork height in cm
};

struct CanMessage {
    uint32_t id;         // CAN message ID
    std::vector<uint8_t> data; // CAN message data
    bool is_extended;    // True if extended ID, false if standard ID
};

class ICanController {
public:
    virtual ~ICanController() = default;

    /**
     * @brief Initializes the CAN controller.
     * @return True if initialization is successful, false otherwise.
     */
    virtual bool initialize() = 0;

    /**
     * @brief Sends a CAN message.
     * @param message The CAN message to send.
     * @return True if the message was sent successfully, false otherwise.
     */
    virtual bool sendMessage(const CanMessage& message) = 0;

    /**
     * @brief Reads a CAN message (non-blocking or blocking with timeout).
     *        For a mock, this might simulate receiving a message.
     * @param message Output parameter for the read CAN message.
     * @return True if a message was read, false otherwise.
     */
    virtual bool readMessage(CanMessage& message) = 0;
    
    /**
     * @brief Gets the current simulated pose of the robot.
     * @return The current RobotPose.
     */
    virtual RobotPose getRobotPose() const = 0;
};

} // namespace Can
} // namespace AutonomousForklift

#endif // AUTONOMOUS_FORKLIFT_ICAN_CONTROLLER_H