#ifndef AUTONOMOUS_FORKLIFT_PALLET_FORKER_H
#define AUTONOMOUS_FORKLIFT_PALLET_FORKER_H

#include "BaseController.h"
#include <memory>
#include <string>
#include <vector>

namespace AutonomousForklift {

class PalletForker : public BaseController {
public:
    struct Config : public BaseConfig {
        float focal_length_mm;
        float sensor_height_mm;
        int pick_palette_idx;
        int go_forward_cm;
        int go_backward_cm;
        int slight_fork_up_cm;
        double align_wait_sec;
    };

    PalletForker(const Config& config, std::shared_ptr<Control::ForkliftController> forklift_controller);

    bool run() override;

private:
    enum class ForkingState {
        IDLE,
        ALIGNING,
        FORK_UP,
        GOING_FORWARD,
        LIFTING_PALLET,
        GOING_BACKWARD,
        COMPLETED,
        FAILED
    };

    Config config_;
    float focal_length_px_;
    ForkingState state_ = ForkingState::IDLE;

    void processDetections(const std::vector<Vision::Detection>& detections, const cv::Mat& depth_image);

    // State handlers
    void handleStateAligning(double current_time_sec, const std::vector<Vision::Detection>& detections, const cv::Mat& depth_image);
    void handleStateForkUp();
    void handleStateGoingForward();
    void handleStateLiftingPallet();
    void handleStateGoingBackward();
};

} // namespace AutonomousForklift

#endif // AUTONOMOUS_FORKLIFT_PALLET_FORKER_H