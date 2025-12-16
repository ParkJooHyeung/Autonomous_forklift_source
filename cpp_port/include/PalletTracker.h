#ifndef AUTONOMOUS_FORKLIFT_PALLET_TRACKER_H
#define AUTONOMOUS_FORKLIFT_PALLET_TRACKER_H

#include "BaseController.h"
#include "KalmanFilterWrapper.h"
#include "calculations.h" 

#include <memory>
#include <string>
#include <vector>

namespace AutonomousForklift {

class PalletTracker : public BaseController {
public:
    struct Config : public BaseConfig {
        double kf_r_val;
        double kf_q_var;
        float depth_scale_override;
        int target_depth_min_cm;
        int target_depth_max_cm;
        int target_center_tolerance;
        double min_tolerance;
        double max_tolerance;
        double min_depth_cm_for_move;
        double max_depth_cm_for_move;
        double forward_move_target_cm;
        double turn_angle_deg;
    };

    PalletTracker(const Config& config, std::shared_ptr<Control::ForkliftController> forklift_controller);

    bool run() override;

private:
    Config config_;
    Filters::KalmanFilterWrapper kf_x_;

    std::string last_cls_;
    double last_object_depth_;
    int last_center_x_;
    double last_p_distance_;

    double calculateMedian(std::vector<double>& data);

    // Decomposed run() method functions
    bool processFrame(cv::Mat& color_image, cv::Mat& depth_image);
    void handleControlLogic();
};

} // namespace AutonomousForklift

#endif // AUTONOMOUS_FORKLIFT_PALLET_TRACKER_H