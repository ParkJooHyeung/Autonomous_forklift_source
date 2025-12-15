#ifndef AUTONOMOUS_FORKLIFT_REALSENSE_MANAGER_H
#define AUTONOMOUS_FORKLIFT_REALSENSE_MANAGER_H

#include <librealsense2/rs.hpp> // RealSense library
#include <opencv2/opencv.hpp>   // OpenCV for image processing

namespace AutonomousForklift {
namespace Sensors {

class RealSenseManager {
public:
    /**
     * @brief Initializes the RealSense camera and starts streaming.
     * @param width Desired width of the video stream.
     * @param height Desired height of the video stream.
     * @param fps Desired frames per second.
     */
    RealSenseManager(int width = 640, int height = 480, int fps = 30);

    /**
     * @brief Stops the RealSense camera stream.
     */
    ~RealSenseManager();

    /**
     * @brief Waits for the next set of frames and returns them.
     * @param color_frame Output parameter for the color image (cv::Mat).
     * @param depth_frame Output parameter for the depth image (cv::Mat).
     * @return True if frames were successfully retrieved, false otherwise.
     */
    bool getFrames(cv::Mat& color_frame, cv::Mat& depth_frame);

    /**
     * @brief Gets the depth scale of the camera.
     * @return The depth scale in meters per LSB.
     */
    float getDepthScale() const;

private:
    rs2::pipeline pipe;
    rs2::config cfg;
    rs2::frameset frames;
    rs2::align align_to_depth; // Used to align color frame to depth frame

    float depth_scale;
    int stream_width;
    int stream_height;
    int stream_fps;

    void setupStreams(int width, int height, int fps);
};

} // namespace Sensors
} // namespace AutonomousForklift

#endif // AUTONOMOUS_FORKLIFT_REALSENSE_MANAGER_H
