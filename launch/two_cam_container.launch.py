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

def get_param_path(path):
    return os.path.join(
        FindPackageShare("arena_camera").perform(LaunchContext()),
        path
    )
def get_param_yaml(path):
    with open(path, "r") as f:
        camera_yaml_param = yaml.safe_load(f)["/**"]["ros__parameters"]
    return camera_yaml_param

def get_parameters(param):
    return [{"camera_name": param['camera_name'],
             "frame_id": param['frame_id'],
             "pixel_format": param['pixel_format'],
             "serial_no": param['serial_no'],
             "camera_info_url": param['camera_info_url'],
             "fps": param['fps'],
             "horizontal_binning": param['horizontal_binning'],
             "vertical_binning": param['vertical_binning'],
            #  "enable_rectifying": param['enable_rectifying'],
            #  "enable_compressing": param['enable_compressing'],
             "use_default_device_settings": param['use_default_device_settings'],
             "exposure_auto": param['exposure_auto'],
             "exposure_target": param['exposure_target'],
             "gain_auto": param['gain_auto'],
             "gain_target": param['gain_target'],
             "gamma_target": param['gamma_target'],
             }]
def generate_launch_description():
    launch_arguments = []

    # get camera param paths
    f_camera_param_path = get_param_path("param/front_camera.param.yaml")
    fr_camera_param_path = get_param_path("param/front_right_camera.param.yaml")


    # get camera yaml
    f_camera_yaml_param = get_param_yaml(f_camera_param_path)
    fr_camera_yaml_param = get_param_yaml(fr_camera_param_path)


    # camera containers
    container_f = ComposableNodeContainer(
        name="camera_node_f",
        namespace="/perception/object_detection",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[
            ComposableNode(
                package="arena_camera",
                plugin="ArenaCameraNode",
                name="arena_camera_node_f",
                parameters=get_parameters(f_camera_yaml_param),
                remappings=[
                ],
                extra_arguments=[
                    {"use_intra_process_comms": True}
                ],
            ),
        ],
        output="both",
    )
    container_fr = ComposableNodeContainer(
        name="camera_node_fr",
        namespace="/perception/object_detection",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[
            ComposableNode(
                package="arena_camera",
                plugin="ArenaCameraNode",
                name="arena_camera_node_fr",
                parameters=get_parameters(fr_camera_yaml_param),
                remappings=[
                ],
                extra_arguments=[
                    {"use_intra_process_comms": True}
                ],
            ),
        ],
        output="both",
    )
    
    return LaunchDescription(
        [
            *launch_arguments,
            container_f,
            container_fr,
        ]
    )