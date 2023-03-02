#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

using namespace std::chrono_literals;

class CsvPublisherNode : public rclcpp::Node {
public:
    CsvPublisherNode() : Node("csv_publisher"), pub_(create_publisher<geometry_msgs::msg::PoseStamped>("pose", 10)) {
        auto timer_callback = [this]() -> void {
            publishNextPose();
        };
        timer_ = create_wall_timer(100ms, timer_callback);
        readCsvFile();
        current_pose_ = 0;
    }

private:
    void readCsvFile() {
        std::ifstream file("poses.csv");
        std::string line;
        while (std::getline(file, line)) {
            std::stringstream ss(line);
            std::string token;
            std::vector<double> values;
            while (std::getline(ss, token, ',')) {
                values.push_back(std::stod(token));
            }
            poses_.push_back(values);
        }
    }

    void publishNextPose() {
        auto pose = poses_[current_pose_];
        geometry_msgs::msg::PoseStamped pose_msg;
        pose_msg.header.stamp = this->get_clock()->now();
        pose_msg.header.frame_id = "map";
        pose_msg.pose.position.x = pose[0];
        pose_msg.pose.position.y = pose[1];
        pose_msg.pose.position.z = pose[2];
        pose_msg.pose.orientation.x = pose[3];
        pose_msg.pose.orientation.y = pose[4];
        pose_msg.pose.orientation.z = pose[5];
        pose_msg.pose.orientation.w = pose[6];
        pub_->publish(pose_msg);
        current_pose_ = (current_pose_ + 1) % poses_.size();
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_;
    std::vector<std::vector<double>> poses_;
    size_t current_pose_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CsvPublisherNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
