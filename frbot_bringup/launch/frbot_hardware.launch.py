import os
from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command

def generate_launch_description():
    pkg_share = get_package_share_directory("frbot_description")
    ros2_control_pkg_share = get_package_share_directory("frbot_controller")
    
    urdf_file = "robot_final.urdf.xacro"
    urdf_path = os.path.join(pkg_share, "urdf", urdf_file)
    controllers_path = os.path.join(ros2_control_pkg_share, "config", "frbot_controllers.yaml")
    rviz_config_path = os.path.join(pkg_share, "rviz", "frbot.rviz")

    robot_description_content = Command(['xacro ', urdf_path])
    robot_description = ParameterValue(robot_description_content, value_type=str)

    rsp_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            {'robot_description': robot_description, 'use_sim_time': False}
        ],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_path],
        parameters=[{'use_sim_time': True}],
    )
    
    control_node = Node(
    package="controller_manager",
    executable="ros2_control_node",
    output="screen",
    parameters=[
        controllers_path,
        {"robot_description": robot_description},
    ],
    )

    spawn_jsb = Node(
    package="controller_manager",
    executable="spawner",
    arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    output="screen",
    )
    spawn_diffdrive = Node(
    package="controller_manager",
    executable="spawner",
    arguments=["diff_drive_controller", "--controller-manager", "/controller_manager"],
    output="screen",
    )


    return LaunchDescription([
        rsp_node,
        control_node,
        rviz_node,
        spawn_jsb,
        spawn_diffdrive,
    ])
