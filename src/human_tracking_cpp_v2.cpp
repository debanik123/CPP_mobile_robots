#include <iostream>
#include <vector>

#include <opencv2/opencv.hpp>

#include <torch/script.h>
#include <torch/torch.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <cv_bridge/cv_bridge.h>

struct DetectionResult {
    float x;
    float y;
    float w;
    float h;
    float confidence;
    int class_id;
};

class ObjectDetector {
public:
    ObjectDetector(const std::string& model_path, float confidence_threshold=0.5, int nms_threshold=50, int input_size=640) :
        model_(torch::jit::load(model_path)),
        confidence_threshold_(confidence_threshold),
        nms_threshold_(nms_threshold),
        input_size_(input_size) {}

    std::vector<DetectionResult> detect(const cv::Mat& image) {
        // Resize the image
        cv::Mat resized_image;
        cv::resize(image, resized_image, cv::Size(input_size_, input_size_));

        // Convert the image to a tensor
        torch::Tensor tensor_image = torch::from_blob(resized_image.data, {1, input_size_, input_size_, 3}, torch::kByte);
        tensor_image = tensor_image.permute({0, 3, 1, 2}).toType(torch::kFloat32);
        tensor_image /= 255.0;

        // Run the model
        auto outputs = model_.forward({tensor_image}).toTuple();
        auto boxes = outputs->elements()[0].toTensor();
        auto scores = outputs->elements()[1].toTensor();
        auto labels = outputs->elements()[2].toTensor();

        // Postprocess the outputs
        std::vector<DetectionResult> results;
        for (int i = 0; i < scores.size(0); i++) {
            float score = scores[i].item<float>();
            if (score >= confidence_threshold_) {
                auto box = boxes[i];
                auto label = labels[i].item<int>();

                float x1 = box[0].item<float>() * image.cols;
                float y1 = box[1].item<float>() * image.rows;
                float x2 = box[2].item<float>() * image.cols;
                float y2 = box[3].item<float>() * image.rows;

                DetectionResult result;
                result.x = x1;
                result.y = y1;
                result.w = x2 - x1;
                result.h = y2 - y1;
                result.confidence = score;
                result.class_id = label;
                results.push_back(result);
            }
        }

        // Apply non-maximum suppression
        if (!results.empty()) {
            std::vector<int> keep(results.size());
            std::iota(keep.begin(), keep.end(), 0);

            std::vector<float> areas(results.size());
            for (int i = 0; i < results.size(); i++) {
                areas[i] = results[i].w * results[i].h;
            }

            for (int i = 0; i < results.size() - 1; i++) {
                auto box1 = results[i];
                for (int j = i + 1; j < results.size(); j++) {
                    auto box2 = results[j];
                    float overlap = std::max(0.0f, std::min(box1.x + box1.w, box2.x + box2.w) - std::max(box1.x, box2.x)) *
                                    std::max(0.0f, std::min(box1.y + box1.h, box2.y + box2.h) - std::max(box1.y, box2.y)) / (areas[i] + areas[j] - std::max(0.0f, overlap));
                    if (overlap >= nms_threshold_) {
                        if (results[j].confidence > results[i].confidence) {
                            keep[i] = -1;
                        } else {
                            keep[j] = -1;
                        }
                    }
                    }
                    }
            auto it = std::remove_if(results.begin(), results.end(), [&](const DetectionResult& result) { return keep[&result - &results[0]] == -1; });
            results.erase(it, results.end());
        }

        return results;
}

private:
torch::jit::script::Module model_;
float confidence_threshold_;
int nms_threshold_;
int input_size_;
};

class ObjectTracker : public rclcpp::Node {
public:
ObjectTracker() : Node("object_tracker") {
// Initialize the object detector
detector_ = std::make_shared<ObjectDetector>("/path/to/yolov5s.torchscript.pt");

    // Subscribe to the image topic
auto callback = std::bind(&ObjectTracker::image_callback, this, std::placeholders::_1);
image_sub_ = create_subscription<sensor_msgs::msg::Image>("image", callback);

// Advertise the detection topic
detection_pub_ = create_publisher<geometry_msgs::msg::PointStamped>("detection", 10);
}


private:
void image_callback(const sensor_msgs::msg::Image::SharedPtr image_msg) {
// Convert the ROS image message to an OpenCV image
cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(image_msg, "bgr8");
cv::Mat image = cv_ptr->image;

// Detect objects in the image
    auto detections = detector_->detect(image);

    // Publish the detections
    for (const auto& detection : detections) {
        geometry_msgs::msg::PointStamped detection_msg;
        detection_msg.header = image_msg->header;
        detection_msg.point.x = detection.x + detection.w / 2;
        detection_msg.point.y = detection.y + detection.h / 2;
        detection_msg.point.z = detection.confidence;
        detection_pub_->publish(detection_msg);
    }
}

rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr detection_pub_;
std::shared_ptr<ObjectDetector> detector_;
};

int main(int argc, char** argv) {
rclcpp::init(argc, argv);
rclcpp::spin(std::make_shared<ObjectTracker>());
rclcpp::shutdown();
return 0;
}




