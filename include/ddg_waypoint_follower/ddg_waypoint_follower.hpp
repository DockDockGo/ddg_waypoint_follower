#ifndef DDG_WAYPOINT_FOLLOWER_HPP_
#define DDG_WAYPOINT_FOLLOWER_HPP_

#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <chrono>
#include <deque>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>

#include "ddg_multi_robot_srvs/srv/ddg_execute_waypoints.hpp"
#include "rclcpp/rclcpp.hpp"

namespace ddg_waypoint_follower {

enum class WaypointFollowerState {
  IDLE,
  EXECUTING_WAYPOINTS,
};

struct WaypointFollowerVars {
  WaypointFollowerState state;
  int waypoint_index;
  int prev_waypoint_index;
  nav_msgs::msg::Path waypoints;
};

class DDGWaypointFollower : public rclcpp::Node {
 public:
  DDGWaypointFollower();

 private:
  //  config params
  float GOAL_THREHSOLD = 0.5;  // in meters
  int WAYPOINT_WAIT = 5000;    // in milliseconds

  // vars for waiting
  bool pose_lock = false;
  bool waitstep_at_waypoint_timer_trigger = false;
  long int prev_time = 0;

  // config variables
  std::string namespace_;
  bool use_sim_;

  geometry_msgs::msg::PoseStamped curr_pose;

  // state variables
  WaypointFollowerVars vars_;

  std::string goal_pose_topic_name = "/goal_pose";
  std::string execute_waypoint_service_name = "/ddg_navigate_through_poses";

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr
      robot_goal_pose_publisher;
  rclcpp::Service<ddg_multi_robot_srvs::srv::DdgExecuteWaypoints>::SharedPtr
      ddg_execute_waypoint;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::TimerBase::SharedPtr execute_waypoint_timer_;
  rclcpp::TimerBase::SharedPtr update_robot_pose_timer_;

  // functions
  bool executeWaypoints(nav_msgs::msg::Path &path);
  bool isTargetReached(geometry_msgs::msg::PoseStamped &p1,
                       geometry_msgs::msg::PoseStamped &p2);
  double distBetweenPoses(geometry_msgs::msg::PoseStamped &p1,
                          geometry_msgs::msg::PoseStamped &p2);

  void handleExecuteWaypointsRequest(
      const std::shared_ptr<
          ddg_multi_robot_srvs::srv::DdgExecuteWaypoints::Request>
          request,
      std::shared_ptr<ddg_multi_robot_srvs::srv::DdgExecuteWaypoints::Response>
          response);
  void RobotPoseCallback();
  void updateRobotPose();
  void waypoint_executor_callback();
};

}  // namespace ddg_waypoint_follower

#endif  // DDG_WAYPOINT_FOLLOWER_HPP_
