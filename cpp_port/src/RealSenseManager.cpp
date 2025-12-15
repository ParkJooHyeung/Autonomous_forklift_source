#include "RealSenseManager.h"
#include <iostream>

namespace AutonomousForklift {
namespace Sensors {

RealSenseManager::RealSenseManager(int width, int height, int fps)
    : align_to_depth(RS2_STREAM_DEPTH),
      stream_width(width),
      stream_height(height),
      stream_fps(fps),
      depth_scale(0.0f) // Will be updated after pipeline starts
{
    setupStreams(width, height, fps);

    // Start streaming
    try {
        rs2::pipeline_profile profile = pipe.start(cfg);

        // Get depth sensor and its depth scale
        auto depth_sensor = profile.get_device().first<rs2::depth_sensor>();
        depth_scale = depth_sensor.get_depth_scale();
        std::cout << "RealSense started with depth scale: " << depth_scale << std::endl;

    } catch (const rs2::error& e) {
        std::cerr << "RealSense error calling " << e.get_failed_function() << ":\n    " << e.what() << std::endl;
        // Handle error, maybe rethrow or set a flag
    }
}

RealSenseManager::~RealSenseManager() {
    try {
        pipe.stop();
        std::cout << "RealSense pipeline stopped." << std::endl;
    } catch (const rs2::error& e) {
        std::cerr << "RealSense error stopping pipeline: " << e.what() << std::endl;
    }
}

void RealSenseManager::setupStreams(int width, int height, int fps) {
    // Enable color stream
    cfg.enable_stream(RS2_STREAM_COLOR, width, height, RS2_FORMAT_BGR8, fps);
    // Enable depth stream
    cfg.enable_stream(RS2_STREAM_DEPTH, width, height, RS2_FORMAT_Z16, fps);
}

bool RealSenseManager::getFrames(cv::Mat& color_frame, cv::Mat& depth_frame) {
    try {
        // Wait for a coherent set of frames
        frames = pipe.wait_for_frames();

        // Align the depth frame to the color frame
        auto aligned_frames = align_to_depth.process(frames);

        // Get color and depth frames
        rs2::frame color_rs_frame = aligned_frames.get_color_frame();
        rs2::depth_frame depth_rs_frame = aligned_frames.get_depth_frame();

        if (!color_rs_frame || !depth_rs_frame) {
            std::cerr << "Failed to get color or depth frame." << std::endl;
            return false;
        }

        // Convert RealSense frames to OpenCV Mat
        color_frame = cv::Mat(cv::Size(stream_width, stream_height), CV_8UC3, (void*)color_rs_frame.get_data(), cv::Mat::AUTO_STEP);
        depth_frame = cv::Mat(cv::Size(stream_width, stream_height), CV_16UC1, (void*)depth_rs_frame.get_data(), cv::Mat::AUTO_STEP);

        return true;
    } catch (const rs2::error& e) {
        std::cerr << "RealSense error in getFrames: " << e.what() << std::endl;
        return false;
    }
}

float RealSenseManager::getDepthScale() const {
    return depth_scale;
}

} // namespace Sensors
} // namespace AutonomousForklift
