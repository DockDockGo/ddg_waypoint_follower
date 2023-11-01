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
    use_sim = DeclareLaunchArgument(
        "use_sim",
        default_value="False",
        description="Simulation or Real)",
    )
    namespace = DeclareLaunchArgument(
        "namespace",
        default_value="robot2",
        description="Namespace",
    )

    ld.add_action(use_sim)
    ld.add_action(namespace)

    ddg_waypoint_follower_node = Node(
        package="ddg_waypoint_follower",
        executable="ddg_waypoint_follower_node",
        output="screen",
        parameters=[
            {"use_sim": LaunchConfiguration("use_sim")},
            {"namespace": LaunchConfiguration("namespace")},
        ],
    )

    # Add the node to the launch description
    ld.add_action(ddg_waypoint_follower_node)

    return ld
