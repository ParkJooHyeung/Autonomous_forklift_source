#ifndef AUTONOMOUS_FORKLIFT_YOLO_DETECTOR_H
#define AUTONOMOUS_FORKLIFT_YOLO_DETECTOR_H

#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>
#include <vector>
#include <string>

namespace AutonomousForklift {
namespace Vision {

struct Detection {
    int class_id;
    float confidence;
    cv::Rect box;
};

class YoloDetector {
public:
    /**
     * @brief Constructor for YoloDetector.
     * @param model_path Path to the ONNX YOLO model file.
     * @param conf_threshold Confidence threshold for detections.
     * @param nms_threshold Non-Maximum Suppression threshold.
     */
    YoloDetector(const std::string& model_path, float conf_threshold = 0.5f, float nms_threshold = 0.4f);

    /**
     * @brief Performs object detection on the input image.
     * @param image The input image (cv::Mat).
     * @return A vector of detected objects.
     */
    std::vector<Detection> detect(const cv::Mat& image);

private:
    cv::dnn::Net net;
    float conf_threshold;
    float nms_threshold;
    std::vector<std::string> class_names; // Assuming we will load class names from a file or hardcode them

    void preprocess(const cv::Mat& frame, cv::Mat& blob);
    std::vector<Detection> postprocess(const cv::Mat& frame, const std::vector<cv::Mat>& outputs);
    void loadClassNames(const std::string& names_file);
};

} // namespace Vision
} // namespace AutonomousForklift

#endif // AUTONOMOUS_FORKLIFT_YOLO_DETECTOR_H
