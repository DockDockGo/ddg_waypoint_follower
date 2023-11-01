#include "ddg_waypoint_follower/ddg_waypoint_follower.hpp"

#include "rclcpp/rclcpp.hpp"

namespace ddg_waypoint_follower {

DDGWaypointFollower::DDGWaypointFollower() : Node("ddg_waypoint_follower") {
  // Initialize member functions, subscribers, publishers, and other private
  // members here.

  this->declare_parameter<bool>("use_sim", true);
  this->get_parameter("use_sim", use_sim_);

  this->declare_parameter<std::string>("namespace", "");
  this->get_parameter("namespace", namespace_);

  this->declare_parameter<float>("target_xy_threshold", 0.25);
  this->get_parameter("target_xy_threshold", GOAL_THREHSOLD);

  this->declare_parameter<int>("wait_time", 7500);
  this->get_parameter("wait_time", WAYPOINT_WAIT);
  // TODO @VineetTambe get namespace from the parameter server

  //  initialize the publisher to goal pose topic using namespace
  //  publisher to publish the goal pose for the custom waypoint
  // follower

  // initialize tf buffer and tf listener
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // set goal pose topic listener
  std::string final_goal_pose_topic_name = "";
  std::string final_execute_waypoint_service_name = "";

  if (namespace_ == "") {
    final_goal_pose_topic_name += goal_pose_topic_name;
    final_execute_waypoint_service_name += execute_waypoint_service_name;
  } else {
    final_goal_pose_topic_name += "/" + namespace_ + goal_pose_topic_name;
    final_execute_waypoint_service_name +=
        "/" + namespace_ + execute_waypoint_service_name;
  }

  RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"),
                     "The topic to publish goal pose for agent is: "
                         << final_goal_pose_topic_name);

  robot_goal_pose_publisher =
      this->create_publisher<geometry_msgs::msg::PoseStamped>(
          final_goal_pose_topic_name, 10);

  // initialize a service to accept a nav_msgs::msg::Path and start waypoint
  // following.

  RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"),
                     "The service to for ddg navigate through poses: "
                         << final_execute_waypoint_service_name);

  ddg_execute_waypoint =
      create_service<ddg_multi_robot_srvs::srv::DdgExecuteWaypoints>(
          final_execute_waypoint_service_name,
          [this](const std::shared_ptr<
                     ddg_multi_robot_srvs::srv::DdgExecuteWaypoints::Request>
                     request,
                 std::shared_ptr<
                     ddg_multi_robot_srvs::srv::DdgExecuteWaypoints::Response>
                     response) {
            handleExecuteWaypointsRequest(request, response);
          });

  // 10 Hz
  update_robot_pose_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&DDGWaypointFollower::updateRobotPose, this));

  execute_waypoint_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&DDGWaypointFollower::waypoint_executor_callback, this));
}

double DDGWaypointFollower::distBetweenPoses(
    geometry_msgs::msg::PoseStamped &p1, geometry_msgs::msg::PoseStamped &p2) {
  return sqrt(pow(abs(p1.pose.position.x - p2.pose.position.x), 2) +
              pow(abs(p1.pose.position.y - p2.pose.position.y), 2));
}

bool DDGWaypointFollower::isTargetReached(geometry_msgs::msg::PoseStamped &p1,
                                          geometry_msgs::msg::PoseStamped &p2) {
  // TODO @VineetTambe add yaw threshold
  if (distBetweenPoses(p1, p2) <= GOAL_THREHSOLD) {
    return true;
  }
  return false;
}

void DDGWaypointFollower::updateRobotPose() {
  std::string target_frame = "map";  // The frame you want the pose in
  std::string source_frame =
      namespace_ + ((use_sim_) ? "base_link" : "/base_link");

  geometry_msgs::msg::TransformStamped transform_stamped;
  try {
    transform_stamped = tf_buffer_->lookupTransform(target_frame, source_frame,
                                                    tf2::TimePointZero);
    pose_lock = true;
  } catch (tf2::TransformException &ex) {
    // RCLCPP_ERROR(get_logger(), "Failed to obtain robot pose: %s", ex.what());
    pose_lock = false;
    return;
  }

  // Create a PoseStamped message
  curr_pose.header.frame_id = target_frame;
  curr_pose.header.stamp = transform_stamped.header.stamp;
  curr_pose.pose.position.x = transform_stamped.transform.translation.x;
  curr_pose.pose.position.y = transform_stamped.transform.translation.y;
  curr_pose.pose.position.z = transform_stamped.transform.translation.z;
  curr_pose.pose.orientation = transform_stamped.transform.rotation;

  // RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"),
  //                    "Current pose of the robot is: "
  //                        << vars_.curr_pose.pose.position.x << " "
  //                        << vars_.curr_pose.pose.position.y << " "
  //                        << vars_.curr_pose.pose.position.z << " "
  //                        << vars_.curr_pose.pose.orientation.x << " "
  //                        << vars_.curr_pose.pose.orientation.y << " "
  //                        << vars_.curr_pose.pose.orientation.z << " "
  //                        << vars_.curr_pose.pose.orientation.w);
}

void DDGWaypointFollower::handleExecuteWaypointsRequest(
    const std::shared_ptr<
        ddg_multi_robot_srvs::srv::DdgExecuteWaypoints::Request>
        request,
    std::shared_ptr<ddg_multi_robot_srvs::srv::DdgExecuteWaypoints::Response>
        response) {
  vars_.state = WaypointFollowerState::IDLE;

  // TODO @VineetTambe implement this function
  vars_.waypoint_index = 0;
  vars_.prev_waypoint_index = 0;
  nav_msgs::msg::Path wp = request->waypoints;
  vars_.waypoints = wp;

  RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"),
                     "Received a request to execute waypoints");

  vars_.state = WaypointFollowerState::EXECUTING_WAYPOINTS;

  response->success = true;
}

void DDGWaypointFollower::waypoint_executor_callback() {
  // TODO @VineetTambe implement this function

  if (vars_.state == WaypointFollowerState::IDLE) return;

  if (!pose_lock) {
    RCLCPP_WARN(get_logger(),
                "Robot pose not available, can not follow waypoints");
    return;
  }

  // RCLCPP_INFO(get_logger(), "Here1");

  if ((size_t)vars_.waypoint_index >= vars_.waypoints.poses.size()) {
    RCLCPP_INFO(get_logger(), "Goal Reached");
    vars_.state = WaypointFollowerState::IDLE;
    // Done executing waypoints
    return;
  }

  // RCLCPP_INFO(get_logger(), "Here2");

  if (vars_.waypoint_index == 0 ||
      vars_.waypoint_index != vars_.prev_waypoint_index) {
    // publish waypoint
    vars_.prev_waypoint_index = vars_.waypoint_index;
    vars_.waypoints.poses[vars_.waypoint_index].header.frame_id = "map";
    vars_.waypoints.poses[vars_.waypoint_index].header.stamp = this->now();

    robot_goal_pose_publisher->publish(
        vars_.waypoints.poses[vars_.waypoint_index]);
  }

  if (vars_.waypoint_index > 0 && vars_.waypoints.poses.size() > 1) {
    if (vars_.waypoints.poses[vars_.waypoint_index].pose ==
        vars_.waypoints.poses[vars_.waypoint_index - 1].pose) {
      if (!waitstep_at_waypoint_timer_trigger) {
        waitstep_at_waypoint_timer_trigger = true;
        prev_time = this->now().nanoseconds() / 1000000;
        RCLCPP_INFO(get_logger(), "Starting wait for %d milliseconds",
                    WAYPOINT_WAIT);
        return;
      }

      if (((this->now().nanoseconds() / 1000000) - prev_time) >=
          WAYPOINT_WAIT) {
        waitstep_at_waypoint_timer_trigger = false;
        vars_.waypoint_index++;
      } else {
        return;
      }
    }
  }

  if (isTargetReached(curr_pose, vars_.waypoints.poses[vars_.waypoint_index])) {
    // reached the current waypoint
    // increment the waypoint index
    RCLCPP_INFO(get_logger(), "Waypoint %d Reached!", vars_.waypoint_index);
    vars_.waypoint_index++;

    return;
  }
}

}  // namespace ddg_waypoint_follower

int main(int argc, char *argv[]) {
  // Initialize the ROS2 node
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ddg_waypoint_follower::DDGWaypointFollower>());

  // Shutdown ROS2
  rclcpp::shutdown();
  return 0;
}
