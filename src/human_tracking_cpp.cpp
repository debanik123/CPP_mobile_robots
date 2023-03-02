#include <iostream>
#include <vector>

#include <opencv2/opencv.hpp>

#include <torch/script.h>
#include <torch/torch.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <cv_bridge/cv_bridge.h>

using namespace std::chrono;

// Define a struct to store object detection results
struct DetectionResult {
    float x, y, w, h, confidence;
    int class_id;
};

// Define a class for YOLOv5 object detector
class ObjectDetector {
public:
    ObjectDetector(std::string model_path, float confidence_threshold = 0.5, int nms_threshold = 50, int input_size = 640) 
        : confidence_threshold_(confidence_threshold), nms_threshold_(nms_threshold), input_size_(input_size) {
        // Load the YOLOv5 model
        model_ = torch::jit::load(model_path);

        // Set the model to evaluation mode
        model_.eval();
    }

    // Detect objects in an image and return a vector of detection results
    std::vector<DetectionResult> detect(cv::Mat image) {
        // Preprocess the image
        cv::Mat resized_image;
        cv::resize(image, resized_image, cv::Size(input_size_, input_size_));
        cv::Mat float_image;
        resized_image.convertTo(float_image, CV_32FC3, 1.0/255.0);
        cv::Mat input_image = float_image.clone().reshape(1, input_size_, input_size_, 3);

        // Convert the image to a Torch tensor
        auto input_tensor = torch::from_blob(input_image.data, {1, input_size_, input_size_, 3}).to(torch::kCUDA);

        // Run the model inference
        auto output = model_.forward({input_tensor}).toTensor().squeeze();
        auto detections = output.index({output.index({torch::arange(0, output.size(0)), 5}).sigmoid() > confidence_threshold_});

        // Postprocess the detection results
        std::vector<DetectionResult> results;
        if (!detections.is_empty()) {
            detections.select(1, 0) = (detections.select(1, 0) + detections.select(1, 2)/2) * image.cols;
            detections.select(1, 1) = (detections.select(1, 1) + detections.select(1, 3)/2) * image.rows;
            detections.select(1, 2) *= image.cols;
            detections.select(1, 3) *= image.rows;
            detections.select(1, 0) -= detections.select(1, 2)/2;
            detections.select(1, 1) -= detections.select(1, 3)/2;
            detections.select(1, 2) += detections.select(1, 0);
            detections.select(1, 3) += detections.select(1, 1);
            detections.select(1, 4) = detections.select(1, 4) * detections.select(1, 5);

            detections = detections.index({detections.select(1, 4).argsort().reverse()});
            std::vector<int> keep;
            while (!detections.is_empty()) {
                auto det = detections[0];
                keep.push_back(det[4].item<int>());
                auto overlap = (detections.narrow(0, 1, detections.size(0)-1) * det).sum(1) / (detections.narrow(0, 1, detections.size(0)-1) + det).sum(1);
            detections = detections.index({overlap <= nms_threshold_/100.0});
        }

        for (int i : keep) {
            auto det = detections[i];
            DetectionResult result;
            result.x = det[0].item<float>();
            result.y = det[1].item<float>();
            result.w = det[2].item<float>() - det[0].item<float>();
            result.h = det[3].item<float>() - det[1].item<float>();
            result.confidence = det[4].item<float>();
            result.class_id = det[5].item<int>();
            results.push_back(result);
        }
    }

    return results;
}

private:
torch::jit::script::Module model_;
float confidence_threshold_;
int nms_threshold_;
int input_size_;
};

int main(int argc, char** argv) {
// Initialize the ROS node
rclcpp::init(argc, argv);
auto node = rclcpp::Node::make_shared("person_tracker");
// Create the object detector
ObjectDetector detector("yolov5s.pt");

// Open the video stream
cv::VideoCapture cap(0);
if (!cap.isOpened()) {
    std::cerr << "Failed to open video stream" << std::endl;
    return -1;
}

// Create the image publisher
auto image_pub = node->create_publisher<sensor_msgs::msg::Image>("image", 10);

// Create the detection result publisher
auto detection_pub = node->create_publisher<geometry_msgs::msg::PointStamped>("detection", 10);

// Create the loop rate object
rclcpp::WallRate loop_rate(30);

// Start the main loop
while (rclcpp::ok()) {
    // Capture a frame from the video stream
    cv::Mat frame;
    cap >> frame;

    // Detect objects in the frame
    auto results = detector.detect(frame);

    // Publish the image
    if (!frame.empty()) {
        sensor_msgs::msg::Image::SharedPtr msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
        image_pub->publish(*msg);
    }

    // Publish the detection results
    for (const auto& result : results) {
        if (result.class_id == 0) { // Only track persons
            geometry_msgs::msg::PointStamped msg;
            msg.header.stamp = node->get_clock()->now();
            msg.point.x = result.x + result.w/2;
            msg.point.y = result.y + result.h/2;
            detection_pub->publish(msg);
        }
    }

    // Sleep to maintain the loop rate
    loop_rate.sleep();
}

return 0;

}