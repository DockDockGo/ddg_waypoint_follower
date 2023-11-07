import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Get the path to the ROS2 package
    my_package_share_directory = get_package_share_directory("ddg_waypoint_follower")
    # Create the ROS2 launch description
    ld = LaunchDescription()
    namespace = DeclareLaunchArgument(
        "namespace",
        default_value="robot1",
        description="Namespace",
    )

    # TODO Yaw threshold - not needed as of now as planner does not plan for yaw
    target_xy_threshold = DeclareLaunchArgument(
        "target_xy_threshold",
        default_value="0.25",
        description="The threshold in meters at which the next waypoint is started.",
    )

    target_yaw_threshold = DeclareLaunchArgument(
        "target_yaw_threshold",
        default_value="5.0",
        description="The YAW threshold in degrees at which the next waypoint is started.",
    )

    wait_time = DeclareLaunchArgument(
        "wait_time",
        default_value="7500",
        description="The wait time in milliseconds when a duplicate waypoint is received.",
    )

    ld.add_action(namespace)
    ld.add_action(target_xy_threshold)
    ld.add_action(target_yaw_threshold)
    ld.add_action(wait_time)

    ddg_waypoint_follower_node = Node(
        package="ddg_waypoint_follower",
        executable="ddg_waypoint_follower_node",
        output="screen",
        parameters=[
            {"namespace": LaunchConfiguration("namespace")},
            {"target_xy_threshold": LaunchConfiguration("target_xy_threshold")},
            {"target_yaw_threshold": LaunchConfiguration("target_yaw_threshold")},
            {"wait_time": LaunchConfiguration("wait_time")},
        ],
    )

    # Add the node to the launch description
    ld.add_action(ddg_waypoint_follower_node)

    return ld
