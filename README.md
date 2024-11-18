# hobot_zed_cam

## 描述

启动zed相机，并发布分辨率为1280×640、格式为nv12的双目图像

## 编译

依赖：
- ZED相机SDK：[zed-open-capture](https://github.com/stereolabs/zed-open-capture)
- 地瓜双目算法：[hobot_stereonet](https://github.com/D-Robotics/hobot_stereonet)

1. 安装依赖的库文件，在RDK X5板端执行

```shell
# 如果RDK板端可以联网，可执行apt下载依赖的库文件
sudo apt install libusb-1.0-0-dev libhidapi-libusb0 libhidapi-dev

# 如果不能联网，则执行如下命令将库文件复制到对应目录
cp -rv ./hidapi/* /usr/
```

2. 编译，在RDK X5板端执行

```shell
source /opt/tros/humble/setup.bash
colcon build --packages-select hobot_zed_cam
```

3. 安装udev文件，在RDK X5板端执行（可选），该文件可以在zed相机通过usb接入RDK X5时定义一些设备规则

```shell
cd hobot_zed_cam/udev
bash install_udev_rule.sh
```

## 运行

### (1) 只启动zed相机，发布双目图像

```shell
source /opt/tros/humble/setup.bash
source ./install/setup.bash

# 发布矫正前的图像
ros2 launch hobot_zed_cam pub_stereo_imgs_nv12.launch.py

# 加载zed自带的标定参数，发布矫正后的图像
ros2 launch hobot_zed_cam pub_stereo_imgs_nv12.launch.py need_rectify:=true

# 同时发布矫正前和矫正后的图像
ros2 launch hobot_zed_cam pub_stereo_imgs_nv12.launch.py show_raw_and_rectify:=true
```

在浏览器输入[http://ip:8000](http://ip:8000)即可查看zed输出的双目图像

![](./doc/zed_stereo.png)

### (2) 启动zed相机+双目算法，读取本地的标定文件进行图像矫正

```shell
source /opt/tros/humble/setup.bash
source ./install/setup.bash

ros2 launch hobot_zed_cam test_stereo.launch.py stereo_calib_path:=stereo_8_zed_mini.yaml visual_alpha:=4
```

在浏览器输入[http://ip:8000](http://ip:8000)即可查看双目算法的结果

![](./doc/zed_stereo_custom_calib.png)

### (3) 启动zed相机+双目算法，读取zed自带的矫正参数进行矫正

```shell
source /opt/tros/humble/setup.bash
source ./install/setup.bash

# zed-mini
ros2 launch hobot_zed_cam test_stereo_zed_mini_rectify.launch.py visual_alpha:=4
# zed-2i
ros2 launch hobot_zed_cam test_stereo_zed_2i_rectify.launch.py visual_alpha:=4
```

在浏览器输入[http://ip:8000](http://ip:8000)即可查看双目算法的结果

![](./doc/zed-2i_stereo.png)

### (4) 加载离线图像进行推理保存视差图和深度图

```shell
source /opt/tros/humble/setup.bash
source ./install/setup.bash

# 以下launch文件会加载不同的标定参数，可通过local_image_path参数设置离线文件的路径
ros2 launch hobot_zed_cam test_stereo_zed_2i_offline.launch.py
ros2 launch hobot_zed_cam test_stereo_zed_mini_offline.launch.py
ros2 launch hobot_zed_cam test_stereo_zed_mini_custom_calib_offline.launch.py
```
