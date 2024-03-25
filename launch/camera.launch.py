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
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare

from launch import LaunchContext


def get_param_path(path):
  return os.path.join(
    FindPackageShare("arena_camera").perform(LaunchContext()),
    path
  )


def generate_launch_description():
  f_camera_param_path = get_param_path("param/camera2.param.yaml")

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

  return LaunchDescription(
    [
      container_f,
    ]
  )
