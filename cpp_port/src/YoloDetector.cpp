#include "YoloDetector.h"
#include <fstream>
#include <iostream>
#include <algorithm>

namespace AutonomousForklift {
namespace Vision {

YoloDetector::YoloDetector(const std::string& model_path, float conf_threshold, float nms_threshold)
    : conf_threshold(conf_threshold), nms_threshold(nms_threshold) {
    try {
        net = cv::dnn::readNetFromONNX(model_path);
        // Set the backend and target to optimize for the available hardware
        // For CPU:
        net.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
        net.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
        // For GPU (if available and built with CUDA):
        // net.setPreferableBackend(cv::dnn::DNN_BACKEND_CUDA);
        // net.setPreferableTarget(cv::dnn::DNN_TARGET_CUDA);

        // Load class names - placeholder for now
        // A common practice is to have a coco.names or similar file
        // For this project, if classes are fixed (e.g., just "pallet"), it can be hardcoded.
        // For now, an empty list. User might provide a file later or it's implicitly handled.
        // loadClassNames("coco.names"); // Example, commented out for now.
    } catch (const cv::Exception& e) {
        std::cerr << "Error loading YOLO model: " << e.what() << std::endl;
        // Handle error, maybe rethrow or exit
    }
}

void YoloDetector::preprocess(const cv::Mat& frame, cv::Mat& blob) {
    // YOLO models typically expect 640x640 images. Adjust as per model.
    // Convert BGR to RGB is often done, but readNetFromONNX might handle this internally
    // or the model was trained on BGR. Defaulting to assuming BGR for OpenCV input.
    // Scaling to 1/255.0 is also common.

    // YOLOv5 input usually 640x640, scale factor 1/255, mean 0, swapRB true
    cv::dnn::blobFromImage(frame, blob, 1/255.0, cv::Size(640, 640), cv::Scalar(), true, false, CV_32F);
}

std::vector<Detection> YoloDetector::detect(const cv::Mat& image) {
    cv::Mat blob;
    preprocess(image, blob);

    net.setInput(blob);

    // Get the names of the output layers
    std::vector<std::string> output_names = net.getUnconnectedOutLayersNames();
    std::vector<cv::Mat> outputs;
    net.forward(outputs, output_names);

    return postprocess(image, outputs);
}

std::vector<Detection> YoloDetector::postprocess(const cv::Mat& frame, const std::vector<cv::Mat>& outputs) {
    std::vector<int> class_ids;
    std::vector<float> confidences;
    std::vector<cv::Rect> boxes;

    // Iterate through output detections
    for (const auto& output : outputs) {
        // Each row in output is a detection: [bbox_x, bbox_y, bbox_width, bbox_height, confidence, class_scores...]
        // For YOLOv5, output format might be slightly different.
        // Assuming (center_x, center_y, width, height, object_confidence, class_1_score, ..., class_N_score)
        // Need to check specific YOLOv5 ONNX export format.
        // This is a generic interpretation.
        float* data = (float*)output.data;
        for (int i = 0; i < output.rows; ++i) {
            float object_confidence = data[4];
            if (object_confidence >= conf_threshold) {
                cv::Mat scores = output.row(i).colRange(5, output.cols);
                cv::Point class_id_point;
                double max_class_score;
                cv::minMaxLoc(scores, 0, &max_class_score, 0, &class_id_point);

                if (max_class_score > conf_threshold) {
                    float center_x = data[0] * frame.cols;
                    float center_y = data[1] * frame.rows;
                    float width = data[2] * frame.cols;
                    float height = data[3] * frame.rows;
                    int x = static_cast<int>(center_x - width / 2);
                    int y = static_cast<int>(center_y - height / 2);

                    class_ids.push_back(class_id_point.x);
                    confidences.push_back(object_confidence * max_class_score); // Combined confidence
                    boxes.push_back(cv::Rect(x, y, static_cast<int>(width), static_cast<int>(height)));
                }
            }
            data += output.cols; // Move to next row
        }
    }

    std::vector<int> indices;
    cv::dnn::NMSBoxes(boxes, confidences, conf_threshold, nms_threshold, indices);

    std::vector<Detection> detections;
    for (int idx : indices) {
        Detection det;
        det.class_id = class_ids[idx];
        det.confidence = confidences[idx];
        det.box = boxes[idx];
        detections.push_back(det);
    }

    return detections;
}

void YoloDetector::loadClassNames(const std::string& names_file) {
    // This function can be implemented later if a names file is provided.
    // For now, it's a placeholder.
    // Example:
    // std::ifstream ifs(names_file.c_str());
    // if (!ifs.is_open()) {
    //     std::cerr << "Warning: Class names file not found: " << names_file << std::endl;
    //     return;
    // }
    // std::string line;
    // while (std::getline(ifs, line)) {
    //     class_names.push_back(line);
    // }
}

} // namespace Vision
} // namespace AutonomousForklift
