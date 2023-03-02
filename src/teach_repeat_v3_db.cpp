#include <memory>
#include <vector>
#include <sqlite3.h>
#include "rclcpp/rclcpp.hpp"
#include "nav2_msgs/action/navigate_through_poses.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

using namespace std::chrono_literals;

class Navigation
{
public:
  Navigation()
  : action_client_(rclcpp_action::create_client<nav2_msgs::action::NavigateThroughPoses>("navigate_through_poses", this))
  {
    while (!action_client_->wait_for_action_server(1s)) {
      printf("Waiting for action server to start...\n");
    }
    printf("Action server started\n");
  }

  void teach_and_repeat()
  {
    load_poses_from_database();
    send_teach_goal();
    wait_for_teach_result();
    send_repeat_goal();
  }

  void load_poses_from_database()
  {
    sqlite3 *db;
    int rc = sqlite3_open("poses.db", &db);
    if (rc != SQLITE_OK) {
      fprintf(stderr, "Cannot open database: %s\n", sqlite3_errmsg(db));
      sqlite3_close(db);
      exit(1);
    }

    sqlite3_stmt *stmt;
    rc = sqlite3_prepare_v2(db, "SELECT x, y, z, qx, qy, qz, qw FROM poses ORDER BY id ASC", -1, &stmt, NULL);
    if (rc != SQLITE_OK) {
      fprintf(stderr, "Cannot prepare statement: %s\n", sqlite3_errmsg(db));
      sqlite3_close(db);
      exit(1);
    }

    poses_.clear();
    while (sqlite3_step(stmt) == SQLITE_ROW) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header.frame_id = "map";
      pose.pose.position.x = sqlite3_column_double(stmt, 0);
      pose.pose.position.y = sqlite3_column_double(stmt, 1);
      pose.pose.position.z = sqlite3_column_double(stmt, 2);
      pose.pose.orientation.x = sqlite3_column_double(stmt, 3);
      pose.pose.orientation.y = sqlite3_column_double(stmt, 4);
      pose.pose.orientation.z = sqlite3_column_double(stmt, 5);
      pose.pose.orientation.w = sqlite3_column_double(stmt, 6);
      poses_.push_back(pose);
    }

    sqlite3_finalize(stmt);
    sqlite3_close(db);

    printf("Loaded %d poses from database\n", poses_.size());
  }

  void send_teach_goal()
  {
    auto goal = nav2_msgs::action::NavigateThroughPoses::Goal();
    goal.poses = poses_;
    goal.controller_id = "dwb_controller";
    goal.planner_id = "GridBased";
    auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateThroughPoses>::SendGoalOptions();
    send_goal_options.goal_response_callback = std::bind(&Navigation::goal_response_callback, this, std::placeholders::_1);
    send_goal_options.result_callback = std::bind(&Navigation::result_callback, this, std::placeholders::_1);
    auto goal_handle_future = action_client_->async_send_goal(goal, send_goal_options);
    rclcpp::spin_until_future_completed(action_client_, goal_handle_future);
  }

  void wait_for_teach_result()
  {
    auto goal_handle_future = action_client_->async_get_result();
    while (rclcpp::ok()) {
      auto result = goal_handle->get_result();
    if (result) {
    if (result->result) {
    printf("Teach succeeded\n");
    return;
    } else {
    printf("Teach failed: %s\n", result->error_string.c_str());
    return;
    }
    }
    rclcpp::spin_some(action_client->get_node_base_interface());
    }
    }
   void send_repeat_goal()
    {
    auto goal = nav2_msgs::action::NavigateThroughPoses::Goal();
    goal.controller_id = "dwb_controller";
    goal.planner_id = "GridBased";
    auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateThroughPoses>::SendGoalOptions();
    send_goal_options.goal_response_callback = std::bind(&Navigation::goal_response_callback, this, std::placeholders::1);
    send_goal_options.result_callback = std::bind(&Navigation::result_callback, this, std::placeholders::1);
    auto goal_handle_future = action_client->async_send_goal(goal, send_goal_options);
    rclcpp::spin_until_future_completed(action_client, goal_handle_future);
    }

    void goal_response_callback(std::shared_futureGoalHandleNavigateThroughPoses::SharedPtr future)
    {
    auto goal_handle = future.get();
    if (!goal_handle) {
    printf("Goal was rejected by server\n");
    return;
    }
    printf("Goal accepted by server, ID=%lu\n", goal_handle->get_goal_id().id);
    }

    void result_callback(const GoalHandleNavigateThroughPoses::WrappedResult &result)
    {
    if (result.result) {
    printf("Navigation succeeded\n");
    } else {
    printf("Navigation failed: %s\n", result.error_string.c_str());
    }
    }

    private:
    rclcpp_action::Client<nav2_msgs::action::NavigateThroughPoses>::SharedPtr action_client_;
    std::vector<geometry_msgs::msg::PoseStamped> poses_;
    };

int main(int argc, char **argv)
{
rclcpp::init(argc, argv);
auto navigation = std::make_shared<Navigation>();
navigation->teach_and_repeat();
rclcpp::shutdown();
return 0;
}
