# Copyright (c) 2024，D-Robotics.
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
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node


def declare_configurable_parameters(parameters):
    return [
        DeclareLaunchArgument(
            param["name"],
            default_value=param["default_value"],
            description=param["description"],
        )
        for param in parameters
    ]


def set_configurable_parameters(parameters):
    return dict(
        [(param["name"], LaunchConfiguration(param["name"])) for param in parameters]
    )


def generate_launch_description():
    stereo_calib_file_path = os.path.join(
        get_package_share_directory("hobot_zed_cam"), "config", "stereo_8_zed_mini.yaml"
    )

    node_params = [
        {
            "name": "need_rectify",
            "default_value": "false",
            "description": "need_rectify",
        },
        {
            "name": "show_raw_and_rectify",
            "default_value": "false",
            "description": "show_raw_and_rectify",
        },
        {
            "name": "save_image",
            "default_value": "false",
            "description": "save_image",
        },
        {
            "name": "user_rectify",
            "default_value": "false",
            "description": "user_rectify",
        },
        {
            "name": "resolution",
            "default_value": "720p",
            "description": "resolution",
        },
        {
            "name": "dst_width",
            "default_value": "-1",
            "description": "dst_width",
        },
        {
            "name": "dst_height",
            "default_value": "-1",
            "description": "dst_height",
        },
        {
            "name": "zed_pub_bgr",
            "default_value": "false",
            "description": "zed_pub_bgr",
        },
        {
            "name": "stereo_calib_file_path",
            "default_value": stereo_calib_file_path,
            "description": "stereo_calib_file_path",
        },
        {
            "name": "brightness",
            "default_value": "5",
            "description": "brightness",
        },
        {
            "name": "sharp",
            "default_value": "4",
            "description": "sharp",
        },
        {
            "name": "contrast",
            "default_value": "4",
            "description": "contrast",
        },
        {
            "name": "sat",
            "default_value": "4",
            "description": "sat",
        },
        {
            "name": "gamma",
            "default_value": "4",
            "description": "gamma",
        },
        {"name": "log_level", "default_value": "info", "description": "log_level"},
        {
            "name": "save_origin_image",
            "default_value": "false",
            "description": "save_origin_image",
        },
    ]

    launch = declare_configurable_parameters(node_params)
    launch.append(
        Node(
            package="hobot_zed_cam",
            executable="anypub_stereo_imgs",
            output="screen",
            parameters=[set_configurable_parameters(node_params)],
            arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
        )
    )

    return LaunchDescription(launch)
