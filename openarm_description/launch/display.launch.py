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
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

import launch_ros
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    pkg_share = Path(
        launch_ros.substitutions.FindPackageShare(package="openarm_description").find(
            "openarm_description"
        )
    )
    default_model_path = pkg_share / "urdf/openarm.urdf.xacro"
    default_rviz_config_path = pkg_share / "rviz/robot_description.rviz"

    use_sim_time = LaunchConfiguration("use_sim_time")

    robot_state_publisher_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("openarm_description"),
                        "launch",
                        "description.launch.py",
                    ]
                ),
            ]
        ),
        launch_arguments=dict(use_sim_time=use_sim_time).items(),
    )

    joint_state_publisher_node = launch_ros.actions.Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        condition=launch.conditions.UnlessCondition(
            LaunchConfiguration("gui")),
        parameters=[
            {
                "use_sim_time": use_sim_time,
            }
        ],
    )
    joint_state_publisher_gui_node = launch_ros.actions.Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        condition=launch.conditions.IfCondition(LaunchConfiguration("gui")),
        parameters=[
            {
                "use_sim_time": use_sim_time,
            }
        ],
    )
    rviz_node = launch_ros.actions.Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", LaunchConfiguration("rvizconfig")],
        parameters=[
            {
                "use_sim_time": use_sim_time,
            }
        ],
    )

    return launch.LaunchDescription(
        [
            launch.actions.DeclareLaunchArgument(
                name="use_sim_time",
                default_value="false",
                description="Flag to enable usage of simulation time",
            ),
            launch.actions.DeclareLaunchArgument(
                name="gui",
                default_value="True",
                description="Flag to enable joint_state_publisher_gui",
            ),
            launch.actions.DeclareLaunchArgument(
                name="model",
                default_value=str(default_model_path),
                description="Absolute path to robot urdf file",
            ),
            launch.actions.DeclareLaunchArgument(
                name="rvizconfig",
                default_value=str(default_rviz_config_path),
                description="Absolute path to rviz config file",
            ),
            joint_state_publisher_node,
            joint_state_publisher_gui_node,
            robot_state_publisher_node,
            rviz_node,
        ]
    )
