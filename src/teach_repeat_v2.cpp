#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_through_poses.hpp"
#include "nav2_util/simple_action_client.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

class TeachAndRepeatNavThroughPose
{
public:
  TeachAndRepeatNavThroughPose()
  : node_(rclcpp::Node::make_shared("teach_and_repeat_nav_through_pose")),
    tf_buffer_(node_->get_clock()),
    tf_listener_(tf_buffer_)
  {
  }

  void teachAndSavePath(const std::string& file_path, double record_duration)
  {
    std::vector<geometry_msgs::msg::PoseStamped> poses;
    rclcpp::Rate loop_rate(10);
    double elapsed_time = 0.0;

    while (elapsed_time < record_duration) {
      auto current_pose = getCurrentPose();
      poses.push_back(current_pose);
      elapsed_time += 1.0 / loop_rate.expected_cycle_time().seconds();
      loop_rate.sleep();
    }

    savePosesToFile(file_path, poses);
  }

  void repeatPathFromFile(const std::string& file_path)
  {
    std::vector<geometry_msgs::msg::PoseStamped> poses;
    if (!readPosesFromFile(file_path, poses)) {
      RCLCPP_ERROR(node_->get_logger(), "Failed to read poses from file: %s", file_path.c_str());
      return;
    }

    auto nav_goal = nav2_msgs::msg::NavigateThroughPosesGoal();
    nav_goal.poses = poses;

    nav2_util::SimpleActionClient<nav2_msgs::action::NavigateThroughPoses>::SharedPtr action_client;
    action_client = nav2_util::create_action_client<nav2_msgs::action::NavigateThroughPoses>(node_, "navigate_through_poses");
    if (!action_client->wait_for_action_server(std::chrono::seconds(10))) {
      RCLCPP_ERROR(node_->get_logger(), "Navigate Through Poses action server not available after waiting");
      return;
    }

    RCLCPP_INFO(node_->get_logger(), "Sending goals to Navigate Through Poses action server");

    action_client->async_send_goal(nav_goal);
  }

private:
  rclcpp::Node::SharedPtr node_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  geometry_msgs::msg::PoseStamped getCurrentPose()
  {
    geometry_msgs::msg::TransformStamped transform_stamped;
    try {
      transform_stamped = tf_buffer_.lookupTransform("map", "base_link", tf2::TimePointZero);
    } catch (tf2::TransformException& ex) {
      RCLCPP_ERROR(node_->get_logger(), "%s", ex.what());
      transform_stamped.header.stamp = node_->now();
      transform_stamped.header.frame_id = "map";
      transform_stamped.child_frame_id = "base_link";
      transform_stamped.transform.rotation.w = 1.0;
    }

    geometry_msgs::msg::PoseStamped current_pose;
    current_pose.header = transform_stamped.header;
    current_pose.pose.position.x = transform_stamped.transform.translation.x;
    current_pose.pose.position.y = transform_stamped.transform.translation.y;
    current_pose.pose.position.z = transform_stamped.transform.translation.z;
    current_pose.pose.orientation = transform_stamped
    return current_pose;
  }

  bool readPosesFromFile(const std::string& file_path, std::vector<geometry_msgs::msg::PoseStamped>& poses)
  {
    std::ifstream file(file_path);
    if (!file.is_open()) {
      return false;
    }

    std::string line;
    while (std::getline(file, line)) {
      std::istringstream iss(line);
      double x, y, z, qw, qx, qy, qz;
      if (!(iss >> x >> y >> z >> qw >> qx >> qy >> qz)) {
        return false;
      }

      geometry_msgs::msg::PoseStamped pose;
      pose.header.frame_id = "map";
      pose.pose.position.x = x;
      pose.pose.position.y = y;
      pose.pose.position.z = z;
      pose.pose.orientation.w = qw;
      pose.pose.orientation.x = qx;
      pose.pose.orientation.y = qy;
      pose.pose.orientation.z = qz;
      poses.push_back(pose);
    }

    return true;
  }

  void savePosesToFile(const std::string& file_path, const std::vector<geometry_msgs::msg::PoseStamped>& poses)
  {
    std::ofstream file(file_path);
    if (!file.is_open()) {
      RCLCPP_ERROR(node_->get_logger(), "Failed to open file: %s", file_path.c_str());
      return;
    }

    for (const auto& pose : poses) {
      file << pose.pose.position.x << " " << pose.pose.position.y << " " << pose.pose.position.z << " "
           << pose.pose.orientation.w << " " << pose.pose.orientation.x << " " << pose.pose.orientation.y << " "
           << pose.pose.orientation.z << "\n";
    }

    RCLCPP_INFO(node_->get_logger(), "Saved poses to file: %s", file_path.c_str());
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto teach_and_repeat_nav = TeachAndRepeatNavThroughPose();

  std::string file_path = "/path/to/file.csv";
  double record_duration = 30.0; // seconds
  teach_and_repeat_nav.teachAndSavePath(file_path, record_duration);

  // Wait for the user to press Enter before starting to repeat the path
  std::cout << "Press Enter to repeat the path...";
  std::cin.ignore();

  teach_and_repeat_nav.repeatPathFromFile(file_path);

  rclcpp::shutdown();
  return 0;
}
