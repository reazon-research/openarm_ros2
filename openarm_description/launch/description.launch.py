# Copyright 2025 Reazon Holdings, Inc.
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

from pathlib import Path

import launch
from launch.substitutions import Command
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

import launch_ros
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = Path(
        launch_ros.substitutions.FindPackageShare(package="openarm_description").find(
            "openarm_description"
        )
    )
    default_model_path = pkg_share / "urdf/openarm.urdf.xacro"

    model_arg = launch.actions.DeclareLaunchArgument(
        name="model",
        default_value=str(default_model_path),
        description="Absolute path to the robot's URDF file",
    )
    side_arg = DeclareLaunchArgument(
        name="side", default_value="right",  # Use "left" to test left arm.
        description="Select arm side: 'left' or 'right'"
    )
    zero_pos_arg = DeclareLaunchArgument(
        # Use "arm" to test alternative configuration.
        name="zero_pos", default_value="up",
        description="Specify zero position: 'up' or 'arm'"
    )
    prefix_arg = DeclareLaunchArgument(
        name="prefix", default_value="",
        description="Prefix for link and joint names (e.g., left_, right_)"
    )
    can_device_arg = DeclareLaunchArgument(
        name="can_device", default_value="can0",
        description="CAN device identifier to use"
    )

    use_sim_time = LaunchConfiguration("use_sim_time")
    use_sim_time_launch_arg = DeclareLaunchArgument(
        "use_sim_time", default_value="true")

    robot_description_command = Command([
        "xacro ", LaunchConfiguration("model"),
        " side:=", LaunchConfiguration("side"),
        " zero_pos:=", LaunchConfiguration("zero_pos"),
        " prefix:=", LaunchConfiguration("prefix"),
        " can_device:=", LaunchConfiguration("can_device")
    ])

    robot_state_publisher_node = launch_ros.actions.Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            {
                # ParameterValue is required to avoid being interpreted as YAML.
                "robot_description": ParameterValue(
                    robot_description_command, value_type=str
                ),
                "use_sim_time": use_sim_time,
            },
        ],
    )

    return launch.LaunchDescription(
        [
            model_arg,
            side_arg,
            zero_pos_arg,
            prefix_arg,
            can_device_arg,
            use_sim_time_launch_arg,
            robot_state_publisher_node,
        ]
    )
