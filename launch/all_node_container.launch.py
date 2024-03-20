
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

def generate_launch_description():
    launch_arguments = []

    # get camera param paths
    fr_camera_param_path = get_param_path("param/camera0.param.yaml")
    ml_camera_param_path = get_param_path("param/camera1.param.yaml")
    f_camera_param_path = get_param_path("param/camera2.param.yaml")
    fl_camera_param_path = get_param_path("param/camera3.param.yaml")
    mr_camera_param_path = get_param_path("param/camera4.param.yaml")
    b_camera_param_path = get_param_path("param/camera5.param.yaml")
    br_camera_param_path = get_param_path("param/camera6.param.yaml")
    bl_camera_param_path = get_param_path("param/camera7.param.yaml")


    # camera containers
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
                parameters=[fr_camera_param_path],
                remappings=[
                ],
                extra_arguments=[
                    {"use_intra_process_comms": True}
                ],
            ),
        ],
        output="both",
    )
    container_ml = ComposableNodeContainer(
        name="camera_node_ml",
        namespace="/perception/object_detection",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[
            ComposableNode(
                package="arena_camera",
                plugin="ArenaCameraNode",
                name="arena_camera_node_ml",
                parameters=[ml_camera_param_path],
                remappings=[
                ],
                extra_arguments=[
                    {"use_intra_process_comms": True}
                ],
            ),
        ],
        output="both",
    )
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
                parameters=[f_camera_param_path],
                remappings=[
                ],
                extra_arguments=[
                    {"use_intra_process_comms": True}
                ],
            ),
        ],
        output="both",
    )
    container_fl = ComposableNodeContainer(
        name="camera_node_fl",
        namespace="/perception/object_detection",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[
            ComposableNode(
                package="arena_camera",
                plugin="ArenaCameraNode",
                name="arena_camera_node_fl",
                parameters=[fl_camera_param_path],
                remappings=[
                ],
                extra_arguments=[
                    {"use_intra_process_comms": True}
                ],
            ),
        ],
        output="both",
    )
    container_mr = ComposableNodeContainer(
        name="camera_node_mr",
        namespace="/perception/object_detection",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[
            ComposableNode(
                package="arena_camera",
                plugin="ArenaCameraNode",
                name="arena_camera_node_mr",
                parameters=[mr_camera_param_path],
                remappings=[
                ],
                extra_arguments=[
                    {"use_intra_process_comms": True}
                ],
            ),
        ],
        output="both",
    )
    container_b = ComposableNodeContainer(
        name="camera_node_b",
        namespace="/perception/object_detection",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[
            ComposableNode(
                package="arena_camera",
                plugin="ArenaCameraNode",
                name="arena_camera_node_b",
                parameters=[b_camera_param_path],
                remappings=[
                ],
                extra_arguments=[
                    {"use_intra_process_comms": True}
                ],
            ),
        ],
        output="both",
    )
    container_br = ComposableNodeContainer(
        name="camera_node_br",
        namespace="/perception/object_detection",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[
            ComposableNode(
                package="arena_camera",
                plugin="ArenaCameraNode",
                name="arena_camera_node_br",
                parameters=[br_camera_param_path],
                remappings=[
                ],
                extra_arguments=[
                    {"use_intra_process_comms": True}
                ],
            ),
        ],
        output="both",
    )
    container_bl = ComposableNodeContainer(
        name="camera_node_bl",
        namespace="/perception/object_detection",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[
            ComposableNode(
                package="arena_camera",
                plugin="ArenaCameraNode",
                name="arena_camera_node_bl",
                parameters=[bl_camera_param_path],
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
            container_fr,
            container_ml,
            container_f,
            container_fl,
            container_mr,
            container_b,
            container_br,
            container_bl,
        ]
    )
