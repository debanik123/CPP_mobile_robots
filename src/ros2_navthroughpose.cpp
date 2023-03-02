#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "nav2_msgs/action/navigate_through_poses.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

using namespace std::chrono_literals;

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  // Read CSV file
  std::ifstream file("goals.csv");
  std::string line;
  std::vector<geometry_msgs::msg::PoseStamped> goals;
  while (std::getline(file, line)) {
    std::istringstream iss(line);
    std::string x, y, z, qx, qy, qz, qw;
    if (!(iss >> x >> y >> z >> qx >> qy >> qz >> qw)) {
      break;
    }
    geometry_msgs::msg::PoseStamped pose;
    pose.pose.position.x = std::stod(x);
    pose.pose.position.y = std::stod(y);
    pose.pose.position.z = std::stod(z);
    pose.pose.orientation.x = std::stod(qx);
    pose.pose.orientation.y = std::stod(qy);
    pose.pose.orientation.z = std::stod(qz);
    pose.pose.orientation.w = std::stod(qw);
    goals.push_back(pose);
  }

  // Create action client
  auto client = rclcpp::Node::make_shared("nav_through_pose_client");
  auto action_client = rclcpp_action::create_client<nav2_msgs::action::NavigateThroughPoses>(client, "navigate_through_poses");

  // Wait for action server
  if (!action_client->wait_for_action_server(10s)) {
    RCLCPP_ERROR(client->get_logger(), "Action server not available after waiting");
    return 1;
  }

  // Send goals to action server
  for (auto goal : goals) {
    nav2_msgs::action::NavigateThroughPoses::Goal nav_goal;
    nav_goal.poses.push_back(goal);

    auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateThroughPoses>::SendGoalOptions();
    send_goal_options.result_callback = [](auto) {};

    auto future_goal_handle = action_client->async_send_goal(nav_goal, send_goal_options);
    auto status = future_goal_handle.wait_for(10s);
    if (status == std::future_status::timeout || !future_goal_handle.get()) {
      RCLCPP_ERROR
  (client->get_logger(), "Failed to send goal");
  return 1;
}

auto goal_handle = future_goal_handle.get();
if (!goal_handle) {
  RCLCPP_ERROR(client->get_logger(), "Goal was rejected by server");
  return 1;
}

auto result_future = action_client->async_get_result(goal_handle);
if (rclcpp::spin_until_future_complete(client, result_future) !=
    rclcpp::FutureReturnCode::SUCCESS)
{
  RCLCPP_ERROR(client->get_logger(), "Failed to receive result");
  return 1;
}

auto result = result_future.get();
if (result->total_elapsed_time.sec > 0) {
  RCLCPP_INFO(client->get_logger(), "Goal succeeded in %d seconds", result->total_elapsed_time.sec);
} else {
  RCLCPP_ERROR(client->get_logger(), "Goal failed with error code %d", result->error_code);
  return 1;
}
}

return 0;
}