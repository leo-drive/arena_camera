
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

    context = LaunchContext()

    # get camera param paths
    f_camera_param_path = os.path.join(
        FindPackageShare("arena_camera").perform(context),
        "param/f_camera.param.yaml"
    )
    fr_camera_param_path = os.path.join(
        FindPackageShare("arena_camera").perform(context),
        "param/fr_camera.param.yaml"
    )
    fl_camera_param_path = os.path.join(
        FindPackageShare("arena_camera").perform(context),
        "param/fl_camera.param.yaml"
    )
    mr_camera_param_path = os.path.join(
        FindPackageShare("arena_camera").perform(context),
        "param/mr_camera.param.yaml"
    )
    ml_camera_param_path = os.path.join(
        FindPackageShare("arena_camera").perform(context),
        "param/ml_camera.param.yaml"
    )
    br_camera_param_path = os.path.join(
        FindPackageShare("arena_camera").perform(context),
        "param/br_camera.param.yaml"
    )
    bl_camera_param_path = os.path.join(
        FindPackageShare("arena_camera").perform(context),
        "param/bl_camera.param.yaml"
    )
    b_camera_param_path = os.path.join(
        FindPackageShare("arena_camera").perform(context),
        "param/b_camera.param.yaml"
    )

    with open(f_camera_param_path, "r") as f:
        f_camera_yaml_param = yaml.safe_load(f)["/**"]["ros__parameters"]
    with open(fr_camera_param_path, "r") as f:
        fr_camera_yaml_param = yaml.safe_load(f)["/**"]["ros__parameters"]
    with open(fl_camera_param_path, "r") as f:
        fl_camera_yaml_param = yaml.safe_load(f)["/**"]["ros__parameters"]
    with open(mr_camera_param_path, "r") as f:
        mr_camera_yaml_param = yaml.safe_load(f)["/**"]["ros__parameters"]
    with open(ml_camera_param_path, "r") as f:
        ml_camera_yaml_param = yaml.safe_load(f)["/**"]["ros__parameters"]
    with open(br_camera_param_path, "r") as f:
        br_camera_yaml_param = yaml.safe_load(f)["/**"]["ros__parameters"]
    with open(bl_camera_param_path, "r") as f:
        bl_camera_yaml_param = yaml.safe_load(f)["/**"]["ros__parameters"]
    with open(b_camera_param_path, "r") as f:
        b_camera_yaml_param = yaml.safe_load(f)["/**"]["ros__parameters"]

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
                parameters=[{"camera_name": f_camera_yaml_param['camera_name'],
                             "frame_id": f_camera_yaml_param['frame_id'],
                             "pixel_format": f_camera_yaml_param['pixel_format'],
                             "serial_no": f_camera_yaml_param['serial_no'],
                             "camera_info_url": f_camera_yaml_param['camera_info_url'],
                             "fps": f_camera_yaml_param['fps'],
                             "horizontal_binning": f_camera_yaml_param['horizontal_binning'],
                             "vertical_binning": f_camera_yaml_param['vertical_binning'],
                             "use_default_device_settings": f_camera_yaml_param['use_default_device_settings'],
                             "exposure_auto": f_camera_yaml_param['exposure_auto'],
                             "exposure_target": f_camera_yaml_param['exposure_target'],
                             "gain_auto": f_camera_yaml_param['gain_auto'],
                             "gain_target": f_camera_yaml_param['gain_target'],
                             "gamma_target": f_camera_yaml_param['gamma_target'],
                             }],
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
                parameters=[{"camera_name": fr_camera_yaml_param['camera_name'],
                             "frame_id": fr_camera_yaml_param['frame_id'],
                             "pixel_format": fr_camera_yaml_param['pixel_format'],
                             "serial_no": fr_camera_yaml_param['serial_no'],
                             "camera_info_url": fr_camera_yaml_param['camera_info_url'],
                             "fps": fr_camera_yaml_param['fps'],
                             "horizontal_binning": fr_camera_yaml_param['horizontal_binning'],
                             "vertical_binning": fr_camera_yaml_param['vertical_binning'],
                             "use_default_device_settings": fr_camera_yaml_param['use_default_device_settings'],
                             "exposure_auto": fr_camera_yaml_param['exposure_auto'],
                             "exposure_target": fr_camera_yaml_param['exposure_target'],
                             "gain_auto": fr_camera_yaml_param['gain_auto'],
                             "gain_target": fr_camera_yaml_param['gain_target'],
                             "gamma_target": fr_camera_yaml_param['gamma_target'],
                             }],
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
                parameters=[{"camera_name": fl_camera_yaml_param['camera_name'],
                             "frame_id": fl_camera_yaml_param['frame_id'],
                             "pixel_format": fl_camera_yaml_param['pixel_format'],
                             "serial_no": fl_camera_yaml_param['serial_no'],
                             "camera_info_url": fl_camera_yaml_param['camera_info_url'],
                             "fps": fl_camera_yaml_param['fps'],
                             "horizontal_binning": fl_camera_yaml_param['horizontal_binning'],
                             "vertical_binning": fl_camera_yaml_param['vertical_binning'],
                             "use_default_device_settings": fl_camera_yaml_param['use_default_device_settings'],
                             "exposure_auto": fl_camera_yaml_param['exposure_auto'],
                             "exposure_target": fl_camera_yaml_param['exposure_target'],
                             "gain_auto": fl_camera_yaml_param['gain_auto'],
                             "gain_target": fl_camera_yaml_param['gain_target'],
                             "gamma_target": fl_camera_yaml_param['gamma_target'],
                             }],
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
                parameters=[{"camera_name": mr_camera_yaml_param['camera_name'],
                             "frame_id": mr_camera_yaml_param['frame_id'],
                             "pixel_format": mr_camera_yaml_param['pixel_format'],
                             "serial_no": mr_camera_yaml_param['serial_no'],
                             "camera_info_url": mr_camera_yaml_param['camera_info_url'],
                             "fps": mr_camera_yaml_param['fps'],
                             "horizontal_binning": mr_camera_yaml_param['horizontal_binning'],
                             "vertical_binning": mr_camera_yaml_param['vertical_binning'],
                             "use_default_device_settings": mr_camera_yaml_param['use_default_device_settings'],
                             "exposure_auto": mr_camera_yaml_param['exposure_auto'],
                             "exposure_target": mr_camera_yaml_param['exposure_target'],
                             "gain_auto": mr_camera_yaml_param['gain_auto'],
                             "gain_target": mr_camera_yaml_param['gain_target'],
                             "gamma_target": mr_camera_yaml_param['gamma_target'],
                             }],
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
                parameters=[{"camera_name": ml_camera_yaml_param['camera_name'],
                             "frame_id": ml_camera_yaml_param['frame_id'],
                             "pixel_format": ml_camera_yaml_param['pixel_format'],
                             "serial_no": ml_camera_yaml_param['serial_no'],
                             "camera_info_url": ml_camera_yaml_param['camera_info_url'],
                             "fps": ml_camera_yaml_param['fps'],
                             "horizontal_binning": ml_camera_yaml_param['horizontal_binning'],
                             "vertical_binning": ml_camera_yaml_param['vertical_binning'],
                             "use_default_device_settings": ml_camera_yaml_param['use_default_device_settings'],
                             "exposure_auto": ml_camera_yaml_param['exposure_auto'],
                             "exposure_target": ml_camera_yaml_param['exposure_target'],
                             "gain_auto": ml_camera_yaml_param['gain_auto'],
                             "gain_target": ml_camera_yaml_param['gain_target'],
                             "gamma_target": ml_camera_yaml_param['gamma_target'],
                             }],
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
                parameters=[{"camera_name": br_camera_yaml_param['camera_name'],
                             "frame_id": br_camera_yaml_param['frame_id'],
                             "pixel_format": br_camera_yaml_param['pixel_format'],
                             "serial_no": br_camera_yaml_param['serial_no'],
                             "camera_info_url": br_camera_yaml_param['camera_info_url'],
                             "fps": br_camera_yaml_param['fps'],
                             "horizontal_binning": br_camera_yaml_param['horizontal_binning'],
                             "vertical_binning": br_camera_yaml_param['vertical_binning'],
                             "use_default_device_settings": br_camera_yaml_param['use_default_device_settings'],
                             "exposure_auto": br_camera_yaml_param['exposure_auto'],
                             "exposure_target": br_camera_yaml_param['exposure_target'],
                             "gain_auto": br_camera_yaml_param['gain_auto'],
                             "gain_target": br_camera_yaml_param['gain_target'],
                             "gamma_target": br_camera_yaml_param['gamma_target'],
                             }],
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
                parameters=[{"camera_name": bl_camera_yaml_param['camera_name'],
                             "frame_id": bl_camera_yaml_param['frame_id'],
                             "pixel_format": bl_camera_yaml_param['pixel_format'],
                             "serial_no": bl_camera_yaml_param['serial_no'],
                             "camera_info_url": bl_camera_yaml_param['camera_info_url'],
                             "fps": bl_camera_yaml_param['fps'],
                             "horizontal_binning": bl_camera_yaml_param['horizontal_binning'],
                             "vertical_binning": bl_camera_yaml_param['vertical_binning'],
                             "use_default_device_settings": bl_camera_yaml_param['use_default_device_settings'],
                             "exposure_auto": bl_camera_yaml_param['exposure_auto'],
                             "exposure_target": bl_camera_yaml_param['exposure_target'],
                             "gain_auto": bl_camera_yaml_param['gain_auto'],
                             "gain_target": bl_camera_yaml_param['gain_target'],
                             "gamma_target": bl_camera_yaml_param['gamma_target'],
                             }],
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
                parameters=[{"camera_name": b_camera_yaml_param['camera_name'],
                             "frame_id": b_camera_yaml_param['frame_id'],
                             "pixel_format": b_camera_yaml_param['pixel_format'],
                             "serial_no": b_camera_yaml_param['serial_no'],
                             "camera_info_url": b_camera_yaml_param['camera_info_url'],
                             "fps": b_camera_yaml_param['fps'],
                             "horizontal_binning": b_camera_yaml_param['horizontal_binning'],
                             "vertical_binning": b_camera_yaml_param['vertical_binning'],
                             "use_default_device_settings": b_camera_yaml_param['use_default_device_settings'],
                             "exposure_auto": b_camera_yaml_param['exposure_auto'],
                             "exposure_target": b_camera_yaml_param['exposure_target'],
                             "gain_auto": b_camera_yaml_param['gain_auto'],
                             "gain_target": b_camera_yaml_param['gain_target'],
                             "gamma_target": b_camera_yaml_param['gamma_target'],
                             }],
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
            container_fl,
            container_mr,
            container_ml,
            container_br,
            container_bl,
            container_b,
        ]
    )
