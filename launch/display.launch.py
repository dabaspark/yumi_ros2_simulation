from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    pkg_share = FindPackageShare("yumi_description").find("yumi_description")
    urdf_dir = os.path.join(pkg_share, "urdf")
    
    # Use the yumi.rviz config
    rviz_config_file = os.path.join(pkg_share, "config", "yumi.rviz")
    
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "prefix",
            default_value="PositionJointInterface",
            description="Hardware interface type.",
        )
    )

    # Get URDF via xacro
    robot_description_content = Command(
        [
            FindExecutable(name="xacro"), " ",
            PathJoinSubstitution([urdf_dir, "yumi_pos.urdf.xacro"]),
            " prefix:=",
            LaunchConfiguration("prefix"),
        ]
    )

    robot_description = {"robot_description": robot_description_content}

    # Create a robot_state_publisher node
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    # Create a joint_state_publisher_gui node
    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        output="screen",
    )

    # Create an rviz2 node
    rviz_config_file = os.path.join(pkg_share, "config", "display.rviz")
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_file] if os.path.exists(rviz_config_file) else None,
    )

    # Create the launch description and populate
    ld = LaunchDescription(declared_arguments)
    
    # Add any actions
    ld.add_action(robot_state_publisher_node)
    ld.add_action(joint_state_publisher_gui_node)
    ld.add_action(rviz_node)

    return ld
