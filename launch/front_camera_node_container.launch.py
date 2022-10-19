# Copyright 2020 Tier IV, Inc. All rights reserved.
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

from ament_index_python.packages import get_package_share_directory
import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import SetLaunchConfiguration
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.actions import LoadComposableNodes
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

from launch import LaunchContext

import yaml


def generate_launch_description():
    launch_arguments = []

    def add_launch_arg(name: str, default_value=None, description=None):
        # a default_value of None is equivalent to not passing that kwarg at all
        launch_arguments.append(
            DeclareLaunchArgument(name, default_value=default_value, description=description)
        )

    context = LaunchContext()
    use_intra_process = "True"
    use_multithread = "True"

    camera_param_path = "/home/leo/camera_ws/src/arena_camera/param/lucid_vision_camera.param.yaml"
    with open(camera_param_path, "r") as f:
        camera_yaml_param = yaml.safe_load(f)["/**"]["ros__parameters"]

    def create_parameter_dict(*args):
        result = {}
        for x in args:
            result[x] = LaunchConfiguration(x)
        return result

    container = ComposableNodeContainer(
        name="camera_node_right",
        namespace="/perception/object_detection",
        package="rclcpp_components",
        executable=LaunchConfiguration("container_executable"),
        composable_node_descriptions=[
            ComposableNode(
                package="arena_camera",
                plugin="ArenaCameraNode",
                name="arena_camera_node_right",
                parameters=[{"camera_name": camera_yaml_param['camera_name'],
                             "frame_id": camera_yaml_param['frame_id'],
                             "pixel_format": camera_yaml_param['pixel_format'],
                             "serial_no": camera_yaml_param['serial_no'],
                             "fps": camera_yaml_param['fps'],
                             "width": camera_yaml_param['width'],
                             "height": camera_yaml_param['height'],
                             "resize_image": camera_yaml_param['resize_image'],
                             "camera_info_url": camera_yaml_param['camera_info_url'],

                             }],
                remappings=[
                ],
                extra_arguments=[
                    {"use_intra_process_comms": bool(use_intra_process)}
                ],
            ),
        ],
        output="both",
    )

    set_container_executable = SetLaunchConfiguration(
        "container_executable",
        "component_container",
        condition=UnlessCondition(use_multithread),
    )

    set_container_mt_executable = SetLaunchConfiguration(
        "container_executable",
        "component_container_mt",
        condition=IfCondition(use_multithread),
    )

    return LaunchDescription(
        [
            *launch_arguments,
            set_container_executable,
            set_container_mt_executable,
            container,
        ]
    )
