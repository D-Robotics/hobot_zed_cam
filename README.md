# hobot_zed_cam

## 描述

启动zed相机，并发布分辨率为1280×640、格式为nv12的双目图像

## 编译

参考：[zed-open-capture](https://github.com/stereolabs/zed-open-capture)

1. 安装依赖项，在RDK X5板端执行

```shell
sudo apt install libusb-1.0-0-dev libhidapi-libusb0 libhidapi-dev
```

2. 编译，在RDK X5板端执行

```shell
source /opt/tros/humble/setup.bash
colcon build --packages-select hobot_zed_cam
```

## 运行

### (1) 只启动zed相机，发布双目图像

```shell
source /opt/tros/humble/setup.bash
source ./install/setup.bash

ros2 launch hobot_zed_cam pub_stereo_imgs_nv12.launch.py
```

在浏览器输入[http://ip:8000](http://ip:8000)即可查看zed输出的双目图像

![](./doc/zed_stereo.png)

### (2) 启动zed相机+双目算法，读取本地的矫正文件进行图像矫正

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
