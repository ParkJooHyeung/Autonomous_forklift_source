#ifndef AUTONOMOUS_FORKLIFT_MOCK_CAN_CONTROLLER_H
#define AUTONOMOUS_FORKLIFT_MOCK_CAN_CONTROLLER_H

#include "ICanController.h"
#include <iostream>
#include <iomanip> // For std::hex, std::setfill, std::setw

namespace AutonomousForklift {
namespace Can {

class MockCanController : public ICanController {
public:
    MockCanController() = default;
    ~MockCanController() override = default;

    bool initialize() override {
        std::cout << "[MockCanController] Initializing..." << std::endl;
        return true;
    }

    bool sendMessage(const CanMessage& message) override {
        std::cout << "[MockCanController] Sending CAN message: ID=0x"
                  << std::hex << std::setfill('0') << std::setw(3) << message.id
                  << ", Data=[";
        for (size_t i = 0; i < message.data.size(); ++i) {
            std::cout << std::hex << std::setfill('0') << std::setw(2) << static_cast<int>(message.data[i]);
            if (i < message.data.size() - 1) {
                std::cout << " ";
            }
        }
        std::cout << "], Extended=" << (message.is_extended ? "True" : "False") << std::endl;
        return true;
    }

    bool readMessage(CanMessage& message) override {
        // In a mock, we can simulate receiving a message, or simply return false.
        // For now, it will just indicate no message received.
        // A more advanced mock could have a queue of messages to return.
        // std::cout << "[MockCanController] No message read (mock behavior)." << std::endl;
        return false;
    }
};

} // namespace Can
} // namespace AutonomousForklift

#endif // AUTONOMOUS_FORKLIFT_MOCK_CAN_CONTROLLER_H
