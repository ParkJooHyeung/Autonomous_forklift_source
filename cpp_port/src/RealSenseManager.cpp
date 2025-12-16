#include "RealSenseManager.h"
#include <iostream>
#include <stdexcept>

namespace AutonomousForklift {
namespace Sensors {

RealSenseManager::RealSenseManager(int width, int height, int fps)
    : stream_width(width), stream_height(height), stream_fps(fps), align_to_depth(RS2_STREAM_DEPTH) {
    
    // Set up the configuration for the pipeline
    cfg.enable_stream(RS2_STREAM_DEPTH, stream_width, stream_height, RS2_FORMAT_Z16, stream_fps);
    cfg.enable_stream(RS2_STREAM_COLOR, stream_width, stream_height, RS2_FORMAT_BGR8, stream_fps);

    // Try to start the pipeline
    try {
        pipe.start(cfg);
        is_streaming_ = true; // Set to true if pipeline starts successfully
        std::cout << "[RealSenseManager] RealSense pipeline started." << std::endl;

        // Get depth scale for proper depth calculation
        auto depth_sensor = pipe.get_active_profile().get_device().first<rs2::depth_sensor>();
        depth_scale = depth_sensor.get_depth_scale();

    } catch (const rs2::error& e) {
        std::cerr << "[RealSenseManager] RealSense error: " << e.what() << std::endl;
        std::cerr << "[RealSenseManager] Failed to start RealSense pipeline. No device connected or other error." << std::endl;
        // is_streaming_ remains false
    } catch (const std::exception& e) {
        std::cerr << "[RealSenseManager] Standard exception: " << e.what() << std::endl;
        // is_streaming_ remains false
    }
}

RealSenseManager::~RealSenseManager() {
    if (is_streaming_) {
        pipe.stop();
        std::cout << "[RealSenseManager] RealSense pipeline stopped." << std::endl;
    }
}

bool RealSenseManager::getFrames(cv::Mat& color_frame, cv::Mat& depth_frame) {
    if (!is_streaming_) {
        std::cerr << "[RealSenseManager] Error: Cannot get frames, RealSense pipeline is not streaming." << std::endl;
        return false;
    }

    try {
        frames = pipe.wait_for_frames(); // Wait for the next set of frames from the camera
        frames = align_to_depth.process(frames); // Align the frames

        // Get depth and color frames
        rs2::depth_frame rs_depth_frame = frames.get_depth_frame();
        rs2::video_frame rs_color_frame = frames.get_color_frame();

        // Convert RealSense frames to OpenCV Mat
        depth_frame = cv::Mat(cv::Size(stream_width, stream_height), CV_16UC1, (void*)rs_depth_frame.get_data(), cv::Mat::AUTO_STEP);
        color_frame = cv::Mat(cv::Size(stream_width, stream_height), CV_8UC3, (void*)rs_color_frame.get_data(), cv::Mat::AUTO_STEP);

        return true;
    } catch (const rs2::error& e) {
        std::cerr << "[RealSenseManager] RealSense error in getFrames: " << e.what() << std::endl;
        return false;
    } catch (const std::exception& e) {
        std::cerr << "[RealSenseManager] Standard exception in getFrames: " << e.what() << std::endl;
        return false;
    }
}

float RealSenseManager::getDepthScale() const {
    return depth_scale;
}

void RealSenseManager::setupStreams(int width, int height, int fps) {
    // This method is now integrated into the constructor as the configuration is set before starting the pipeline.
    // However, if there's a need to dynamically change stream settings without re-initializing the manager,
    // this method could be re-implemented to stop, reconfigure, and restart the pipeline.
    // For now, it's not actively used outside the constructor's implicit setup.
}

} // namespace Sensors
} // namespace AutonomousForklift