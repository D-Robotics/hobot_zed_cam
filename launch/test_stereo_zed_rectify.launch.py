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

    camera_fx_arg = DeclareLaunchArgument(
        "camera_fx", default_value="0", description="camera fx"
    )

    camera_cx_arg = DeclareLaunchArgument(
        "camera_cx", default_value="0", description="camera cx"
    )

    camera_cy_arg = DeclareLaunchArgument(
        "camera_cy", default_value="0", description="camera cy"
    )

    camera_b_arg = DeclareLaunchArgument(
        "camera_b", default_value="0", description="camera b"
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
        package='hobot_zed_cam',
        executable='anypub_stereo_imgs_nv12',
        output='screen',
        parameters=[
            {"need_rectify": True},
        ],
        arguments=['--ros-args', '--log-level', 'info']
    )

    # 双目深度估计模型
    stereonet_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("hobot_stereonet"),
                "launch/stereonet_model.launch.py",
            )
        ),
        launch_arguments={
            "stereo_image_topic": "/image_combine_raw",
            "need_rectify": "false",
            "camera_fx": LaunchConfiguration("camera_fx"),
            "camera_fy": LaunchConfiguration("camera_fx"),
            "camera_cx": LaunchConfiguration("camera_cx"),
            "camera_cy": LaunchConfiguration("camera_cy"),
            "base_line": LaunchConfiguration("camera_b"),
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
            camera_fx_arg,
            camera_cx_arg,
            camera_cy_arg,
            camera_b_arg,
            shared_mem_node,
            zed_cam,
            stereonet_node,
            codec_node,
            web_node,
        ]
    )