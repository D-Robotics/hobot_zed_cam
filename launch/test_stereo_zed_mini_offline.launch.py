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

    visual_alpha_arg = DeclareLaunchArgument(
        "visual_alpha", default_value="2", description="visual alpha"
    )

    visual_beta_arg = DeclareLaunchArgument(
        "visual_beta", default_value="0", description="visual beta"
    )

    local_image_path_arg = DeclareLaunchArgument(
        "local_image_path", default_value="/root/zhikang.zeng/helloros_ws/zed-mini-meeting-room-20241009-img-split", description="local image path"
    )

    # 零拷贝环境配置
    shared_mem_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("hobot_shm"), "launch/hobot_shm.launch.py"
            )
        )
    )

    # ros2 launch stereonet_model stereonet_model_web_visual.launch.py \
    # need_rectify:="False" use_local_image:="True" local_image_path:=`pwd`/data/ \
    # camera_fx:=505.044342 camera_fy:=505.044342 camera_cx:=605.167053 camera_cy:=378.247009 base_line:=0.069046
    # 双目深度估计模型
    stereonet_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("hobot_stereonet"),
                "launch/stereonet_model.launch.py",
            )
        ),
        launch_arguments={
            "need_rectify": "false",
            "use_local_image": "true",
            "local_image_path": LaunchConfiguration("local_image_path"),
            "camera_fx": "765.9191691938968",
            "camera_fy": "765.9191691938968",
            "camera_cx": "624.0892486572266",
            "camera_cy": "324.9126722547743",
            "base_line": "0.0627251",
            "alpha": LaunchConfiguration("visual_alpha"),
            "beta": LaunchConfiguration("visual_beta"),
            "stereo_combine_mode": "1",
            "log_level": "warn",
        }.items(),
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
            # 左图和深度拼接后的图
            "codec_sub_topic": "/StereoNetNode/stereonet_visual",
            "codec_in_format": "bgr8",
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
            visual_alpha_arg,
            visual_beta_arg,
            local_image_path_arg,
            shared_mem_node,
            stereonet_node,
            codec_node,
            web_node,
        ]
    )
