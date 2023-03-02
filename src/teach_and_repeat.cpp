#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "nav2_msgs/action/navigate_through_poses.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

using namespace std::chrono_literals;

class TeachAndRepeat
{
public:
  TeachAndRepeat()
  {
    // Initialize ROS2 node
    node_ = rclcpp::Node::make_shared("teach_and_repeat");

    // Initialize transform listener
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(tf_buffer_);

    // Initialize action client
    action_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateThroughPoses>(
      node_,
      "navigate_through_poses"
    );

    // Subscribe to robot pose
    pose_sub_ = node_->create_subscription<geometry_msgs::msg::TransformStamped>(
      "robot_pose",
      rclcpp::QoS(10),
      std::bind(&TeachAndRepeat::recordPose, this, std::placeholders::_1)
    );
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<geometry_msgs::msg::TransformStamped>::SharedPtr pose_sub_;
  std::vector<geometry_msgs::msg::PoseStamped> poses_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  tf2_ros::Buffer tf_buffer_;
  rclcpp_action::Client<nav2_msgs::action::NavigateThroughPoses>::SharedPtr action_client_;

  void recordPose(const geometry_msgs::msg::TransformStamped::SharedPtr msg)
  {
    // Convert pose to PoseStamped
    geometry_msgs::msg::PoseStamped pose;
    pose.header = msg->header;
    pose.pose.position.x = msg->transform.translation.x;
    pose.pose.position.y = msg->transform.translation.y;
    pose.pose.position.z = msg->transform.translation.z;
    pose.pose.orientation = msg->transform.rotation;

    // Convert pose to global frame
    geometry_msgs::msg

::TransformStamped transform_stamped;
try {
  transform_stamped = tf_buffer_.lookupTransform("map", pose.header.frame_id, pose.header.stamp);
} catch (tf2::TransformException &ex) {
  RCLCPP_WARN(node_->get_logger(), "%s", ex.what());
  return;
}
tf2::doTransform(pose, pose, transform_stamped);

// Record pose
poses_.push_back(pose);
}

void savePosesToFile(const std::string &file_path)
{
std::ofstream file(file_path);
if (!file.is_open()) {
RCLCPP_ERROR(node_->get_logger(), "Failed to open file for writing");
return;
}

// Write header
file << "x,y,z,roll,pitch,yaw\n";

// Write poses
for (const auto &pose : poses_) {
  double roll, pitch, yaw;
  tf2::Quaternion quat(pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w);
  tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);

  file << pose.pose.position.x << ","
       << pose.pose.position.y << ","
       << pose.pose.position.z << ","
       << roll << ","
       << pitch << ","
       << yaw << "\n";
}

file.close();

}

std::vector<geometry_msgs::msg::PoseStamped> readPosesFromFile(const std::string &file_path)
{
std::vector<geometry_msgs::msg::PoseStamped> poses;

std::ifstream file(file_path);
if (!file.is_open()) {
  RCLCPP_ERROR(node_->get_logger(), "Failed to open file for reading");
  return poses;
}

// Read header
std::string line;
std::getline(file, line);

// Read poses
while (std::getline(file, line)) {
  std::stringstream ss(line);
  std::string cell;
  std::vector<double> values;
  while (std::getline(ss, cell, ',')) {
    values.push_back(std::stod(cell));
  }

  geometry_msgs::msg::PoseStamped pose;
  pose.pose.position.x = values[0];
  pose.pose.position.y = values[1];
  pose.pose.position.z = values[2];
  tf2::Quaternion quat;
  quat.setRPY(values[3], values[4], values[5]);
  pose.pose.orientation = tf2::toMsg(quat);

  poses.push_back(pose);
}

file.close();

return poses;
}

public:

void teachAndSavePath(const std::string &file_path, double duration)
{
// Wait for pose data
while (poses_.empty()) {
rclcpp::spin_some(node_);
}

// Record poses for duration
auto start_time = node_->now();
while ((node_->now() - start_time).seconds() < duration) {
  rclcpp::spin_some(node_);
}

// Save poses to file
savePosesToFile(file_path);

RCLCPP_INFO(node_->get_logger(), "Path saved to file: %s", file_path.c_str());

}

void repeatPathFromFile(const std::string &file_path)
{
// Read poses from file
auto poses = readPosesFromFile(file_path);
if (poses.empty()) {
RCLCPP_ERROR(node_->get_logger(), "No poses found in file: %s", file_path.c_str());
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
std::vector<geometry_msgs::msg::PoseStamped> poses_;
};

// int main(int argc, char *argv[])
// {
// rclcpp::init(argc, argv);
// auto node = std::make_shared<TeachAndRepeatNavThroughPose>("teach_and_repeat_nav_through_pose");
// rclcpp::spin(node);
// rclcpp::shutdown();
// return 0;
// }

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TeachAndRepeatNavThroughPose>("teach_and_repeat_nav_through_pose");

  // Teach and save path
  node->teachAndSavePath("/path/to/poses.csv", 60.0);  // Record for 60 seconds

  // Repeat path
  node->repeatPathFromFile("/path/to/poses.csv");

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
