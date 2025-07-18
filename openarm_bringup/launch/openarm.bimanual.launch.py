# Copyright 2025 Enactic, Inc.
# Copyright 2024 Stogl Robotics Consulting UG (haftungsbeschränkt)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
import xacro

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, TimerAction, OpaqueFunction
from launch.event_handlers import OnProcessExit
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_robot_description(context: LaunchContext, description_package, description_file,
                               arm_type, use_fake_hardware, right_can_interface, left_can_interface, arm_prefix):
    """Generate robot description using xacro processing."""

    # Substitute launch configuration values
    description_package_str = context.perform_substitution(description_package)
    description_file_str = context.perform_substitution(description_file)
    arm_type_str = context.perform_substitution(arm_type)
    use_fake_hardware_str = context.perform_substitution(use_fake_hardware)
    right_can_interface_str = context.perform_substitution(right_can_interface)
    left_can_interface_str = context.perform_substitution(left_can_interface)
    arm_prefix_str = context.perform_substitution(arm_prefix)

    # Build xacro file path
    xacro_path = os.path.join(
        get_package_share_directory(description_package_str),
        "urdf", "robot", description_file_str
    )

    # Process xacro with required arguments
    robot_description = xacro.process_file(
        xacro_path,
        mappings={
            "arm_type": arm_type_str,
            "bimanual": "true",
            "use_fake_hardware": use_fake_hardware_str,
            "ros2_control": "true",
            "right_can_interface": right_can_interface_str,
            "left_can_interface": left_can_interface_str,
        }
    ).toprettyxml(indent="  ")

    return robot_description


def robot_nodes_spawner(context: LaunchContext, description_package, description_file,
                        arm_type, use_fake_hardware, controllers_file, right_can_interface, left_can_interface, arm_prefix):
    """Spawn both robot state publisher and control nodes with shared robot description."""

    # Generate robot description once
    robot_description = generate_robot_description(
        context, description_package, description_file, arm_type, use_fake_hardware, right_can_interface, left_can_interface, arm_prefix
    )

    # Get controllers file path
    controllers_file_str = context.perform_substitution(controllers_file)
    robot_description_param = {"robot_description": robot_description}

    # Robot state publisher node
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[robot_description_param],
    )

    # Control node
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="both",
        parameters=[robot_description_param, controllers_file_str],
    )

    return [robot_state_pub_node, control_node]


def controller_spawner(context: LaunchContext, robot_controller):
    """Spawn controller based on robot_controller argument."""

    # Substitute launch configuration value
    robot_controller_str = context.perform_substitution(robot_controller)

    # Determine controller names based on robot_controller argument
    if robot_controller_str == "forward_position_controller":
        robot_controller_left = "left_forward_position_controller"
        robot_controller_right = "right_forward_position_controller"
    elif robot_controller_str == "joint_trajectory_controller":
        robot_controller_left = "left_joint_trajectory_controller"
        robot_controller_right = "right_joint_trajectory_controller"
    else:
        raise ValueError(f"Unknown robot_controller: {robot_controller_str}")

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[robot_controller_left,
                   robot_controller_right, "-c", "/controller_manager"],
    )

    return [robot_controller_spawner]


def generate_launch_description():
    """Generate launch description for OpenArm bimanual configuration."""

    # Declare launch arguments
    declared_arguments = [
        DeclareLaunchArgument(
            "description_package",
            default_value="openarm_description",
            description="Description package with robot URDF/xacro files.",
        ),
        DeclareLaunchArgument(
            "description_file",
            default_value="v10.urdf.xacro",
            description="URDF/XACRO description file with the robot.",
        ),
        DeclareLaunchArgument(
            "arm_type",
            default_value="v10",
            description="Type of arm (e.g., v10).",
        ),
        DeclareLaunchArgument(
            "use_fake_hardware",
            default_value="false",
            description="Use fake hardware instead of real hardware.",
        ),
        DeclareLaunchArgument(
            "robot_controller",
            default_value="joint_trajectory_controller",
            choices=["forward_position_controller",
                     "joint_trajectory_controller"],
            description="Robot controller to start.",
        ),
        DeclareLaunchArgument(
            "runtime_config_package",
            default_value="openarm_bringup",
            description="Package with the controller's configuration in config folder.",
        ),
        DeclareLaunchArgument(
            "arm_prefix",
            default_value="",
            description="Prefix for the arm.",
        ),
        DeclareLaunchArgument(
            "right_can_interface",
            default_value="can0",
            description="CAN interface to use for the right arm.",
        ),
        DeclareLaunchArgument(
            "left_can_interface",
            default_value="can1",
            description="CAN interface to use for the left arm.",
        ),
        DeclareLaunchArgument(
            "controllers_file",
            default_value="openarm_v10_bimanual_controllers.yaml",
            description="Controllers file(s) to use. Can be a single file or comma-separated list of files.",
        ),
    ]

    # Initialize launch configurations
    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    arm_type = LaunchConfiguration("arm_type")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    robot_controller = LaunchConfiguration("robot_controller")
    runtime_config_package = LaunchConfiguration("runtime_config_package")
    controllers_file = LaunchConfiguration("controllers_file")
    rightcan_interface = LaunchConfiguration("right_can_interface")
    left_can_interface = LaunchConfiguration("left_can_interface")
    arm_prefix = LaunchConfiguration("arm_prefix")
    # Configuration file paths
    controllers_file = PathJoinSubstitution(
        [FindPackageShare(runtime_config_package), "config",
         "v10_controllers", controllers_file]
    )

    # Robot nodes spawner (both state publisher and control)
    robot_nodes_spawner_func = OpaqueFunction(
        function=robot_nodes_spawner,
        args=[description_package, description_file, arm_type,
              use_fake_hardware, controllers_file, rightcan_interface, left_can_interface, arm_prefix]
    )
    # RViz configuration
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare(description_package), "rviz",
         "bimanual.rviz"]
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )

    # Joint state broadcaster spawner
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster",
                   "--controller-manager", "/controller_manager"],
    )

    # Controller spawners
    controller_spawner_func = OpaqueFunction(
        function=controller_spawner,
        args=[robot_controller]
    )

    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["left_gripper_controller",
                   "right_gripper_controller", "-c", "/controller_manager"],
    )

    # Timing and sequencing
    LAUNCH_DELAY_SECONDS = 1.0
    delayed_joint_state_broadcaster = TimerAction(
        period=LAUNCH_DELAY_SECONDS,
        actions=[joint_state_broadcaster_spawner],
    )

    delayed_robot_controller = TimerAction(
        period=LAUNCH_DELAY_SECONDS,
        actions=[controller_spawner_func],
    )
    delayed_gripper_controller = TimerAction(
        period=LAUNCH_DELAY_SECONDS,
        actions=[gripper_controller_spawner],
    )

    return LaunchDescription(
        declared_arguments + [
            robot_nodes_spawner_func,
            rviz_node,
        ] +
        [
            delayed_joint_state_broadcaster,
            delayed_robot_controller,
            delayed_gripper_controller,
        ]
    )
