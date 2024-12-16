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
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    need_rectify_arg = DeclareLaunchArgument(
        "need_rectify", default_value="false", description="need rectify"
    )

    show_raw_and_rectify_arg = DeclareLaunchArgument(
        "show_raw_and_rectify", default_value="false", description="show raw and rectify"
    )

    save_image_arg = DeclareLaunchArgument(
        "save_image", default_value="false", description="save image flag"
    )
    
    user_rectify_arg = DeclareLaunchArgument(
        "user_rectify", default_value="false", description="user_rectify"
    )

    stereo_calib_file_path =  os.path.join(
        get_package_share_directory('hobot_zed_cam'),
        'config',
        'stereo_8_zed_mini.yaml'
    )
    
    stereo_calib_file_path_arg = DeclareLaunchArgument(
        "stereo_calib_file_path", default_value=stereo_calib_file_path, description="stereo_calib_file_path"
    )

    resolution = DeclareLaunchArgument(
        "resolution", default_value='720', description="resolution"
    )

    zed_pub_bgr = DeclareLaunchArgument(
        "zed_pub_bgr", default_value='False', description="zed_pub_bgr"
    )

    brightness = DeclareLaunchArgument(
        "brightness", default_value='5', description="brightness"
    )

    sharp = DeclareLaunchArgument(
        "sharp", default_value='4', description="sharp"
    )
    sat = DeclareLaunchArgument(
        "sat", default_value='4', description="sat"
    )
    contrast = DeclareLaunchArgument(
        "contrast", default_value='4', description="contrast"
    )
    gamma = DeclareLaunchArgument(
        "gamma", default_value='5', description="gamma"
    )
    
    # 零拷贝环境配置
    shared_mem_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("hobot_shm"), "launch/hobot_shm.launch.py"
            )
        )
    )

    # zed双目相机
    zed_cam = Node(
        package="hobot_zed_cam",
        executable="anypub_stereo_imgs_nv12",
        output="screen",
        parameters=[
            {"need_rectify": LaunchConfiguration("need_rectify")},
            {"show_raw_and_rectify": LaunchConfiguration("show_raw_and_rectify")},
            {"save_image": LaunchConfiguration("save_image")},
            {"user_rectify": LaunchConfiguration("user_rectify")},
            {"stereo_calib_file_path": LaunchConfiguration("stereo_calib_file_path")},
            {"resolution": LaunchConfiguration("resolution")},
            {"zed_pub_bgr": LaunchConfiguration("zed_pub_bgr")},
            {"brightness": LaunchConfiguration("brightness")},
            {"sharp": LaunchConfiguration("sharp")},
            {"sat": LaunchConfiguration("sat")},
            {"contrast": LaunchConfiguration("contrast")},
            {"gamma": LaunchConfiguration("gamma")},
        ],
        arguments=["--ros-args", "--log-level", "info"],
    )

    # 编码节点
    codec_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("hobot_codec"),
                "launch/hobot_codec_encode.launch.py",
            )
        ),
        launch_arguments={
            "codec_in_mode": "ros",
            "codec_out_mode": "ros",
            # 左图和右图拼接后的图
            "codec_sub_topic": "/image_combine_raw",
            "codec_in_format": "nv12",
            "codec_pub_topic": "/image_jpeg",
            "codec_out_format": "jpeg",
            "log_level": "warn",
        }.items(),
    )

    # web展示节点
    web_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("websocket"), "launch/websocket.launch.py"
            )
        ),
        launch_arguments={
            "websocket_image_topic": "/image_jpeg",
            "websocket_only_show_image": "true",
            # 'websocket_smart_topic': '/detect_depth_result'
        }.items(),
    )

    return LaunchDescription(
        [
            need_rectify_arg,
            show_raw_and_rectify_arg,
            save_image_arg,
            user_rectify_arg,
            stereo_calib_file_path_arg,
            shared_mem_node,
            resolution,
            zed_pub_bgr,
            brightness,
            sharp,
            contrast,
            sat,
            gamma,
            zed_cam,
            codec_node,
            web_node,
        ]
    )
    
