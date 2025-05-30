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

import launch
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from launch.launch_description_sources import PythonLaunchDescriptionSource
import pathlib


def generate_launch_description():
    pkg_share = FindPackageShare(package="openarm_bimanual_description")

    xacro_path = (
        pathlib.Path(pkg_share.find("openarm_bimanual_description"))
        / "urdf/openarm_bimanual.urdf.xacro"
    )

    use_sim_time = LaunchConfiguration("use_sim_time")
    use_sim_time_launch_arg = DeclareLaunchArgument(
        name="use_sim_time", default_value="false")

    robot_state_publisher_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        pkg_share,
                        "launch",
                        "description.launch.py",
                    ]
                ),
            ]
        ),
        launch_arguments=dict(use_sim_time=use_sim_time).items(),
    )

    robot_description_content = Command(
        ["xacro ", LaunchConfiguration("model")])

    robot_description_param = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    controller_params = PathJoinSubstitution(
        [FindPackageShare(package="openarm_bimanual_bringup"),
         "config", "controllers.yaml"]
    )

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[controller_params, robot_description_param],
    )

    delayed_controller_manager = TimerAction(
        period=3.0, actions=[controller_manager])

    joint_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broad"],
    )

    delayed_joint_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[joint_broadcaster_spawner],
        )
    )

    return launch.LaunchDescription(
        [
            DeclareLaunchArgument(
                name="model",
                default_value=str(xacro_path),
                description="Absolute path to the robot URDF or xacro file",
            ),
            use_sim_time_launch_arg,
            robot_state_publisher_node,
            delayed_controller_manager,
            delayed_joint_broadcaster_spawner,
        ]
    )
