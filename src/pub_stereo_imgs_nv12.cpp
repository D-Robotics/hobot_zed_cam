// Copyright (c) 2024，D-Robotics.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "opencv2/opencv.hpp"
#include "cv_bridge/cv_bridge.h"
#include "videocapture.hpp"
#include "calibration.hpp"
#include <arm_neon.h>
#include <fstream>

class PubStereoImgsNv12Node : public rclcpp::Node
{
public:
    PubStereoImgsNv12Node() : Node("pub_stereo_imgs_nv12_node")
    {
        RCLCPP_INFO(this->get_logger(), "=> init pub_stereo_imgs_nv12_node!");
        // ======================================================================================================================================
        // param
        need_rectify_ = this->declare_parameter("need_rectify", false);
        show_raw_and_rectify_ = this->declare_parameter("show_raw_and_rectify", false);
        save_image_ = this->declare_parameter("save_image", false);
        if (show_raw_and_rectify_)
            need_rectify_ = true;
        RCLCPP_INFO_STREAM(this->get_logger(), "\033[31m" << std::endl
                                                          << "=> need_rectify: " << need_rectify_ << std::endl
                                                          << "=> show_raw_and_rectify: " << show_raw_and_rectify_ << std::endl
                                                          << "=> save_image: " << save_image_ << std::endl
                                                          << "\033[0m");

        // ======================================================================================================================================
        // sub & pub
        auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
        qos.reliability(rclcpp::ReliabilityPolicy::Reliable);
        qos.durability(rclcpp::DurabilityPolicy::TransientLocal);
        stereo_msg_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/image_combine_raw", qos);

        // start publishing thread
        RCLCPP_INFO(this->get_logger(), "\033[31m=> create pub thread\033[0m");

        // ======================================================================================================================================
        sl_oc::video::VideoParams params;
        params.res = sl_oc::video::RESOLUTION::HD720;
        params.fps = sl_oc::video::FPS::FPS_15;

        // ----> Create Video Capture
        sl_oc::video::VideoCapture cap_0(params);
        if (!cap_0.initializeVideo())
        {
            RCLCPP_ERROR(this->get_logger(), "=> cannot open camera video capture!");
        }
        RCLCPP_INFO_STREAM(this->get_logger(), "=> connected to camera sn: " << cap_0.getSerialNumber() << "[" << cap_0.getDeviceName() << "]");
        // <---- Create Video Capture

        // ----> Retrieve calibration file from Stereolabs server
        // ZED Calibration
        // Download camera calibration file
        std::string calibration_file;
        if (need_rectify_)
        {
            if (!sl_oc::tools::downloadCalibrationFile(cap_0.getSerialNumber(), calibration_file))
            {
                RCLCPP_ERROR(this->get_logger(), "=> could not load calibration file from Stereolabs servers");
            }
            RCLCPP_INFO(this->get_logger(), "=> calibration file found. Loading...");
        }

        // ----> Frame size
        int w, h;
        cap_0.getFrameSize(w, h);
        // <---- Frame size

        // ----> Initialize calibration
        cv::Mat map_left_x, map_left_y;
        cv::Mat map_right_x, map_right_y;
        cv::Mat cameraMatrix_left, cameraMatrix_right;
        double baseline = 0.0;
        if (need_rectify_)
        {
            sl_oc::tools::initCalibration(calibration_file, cv::Size(w / 2, h), map_left_x, map_left_y, map_right_x, map_right_y, cameraMatrix_left, cameraMatrix_right, cv::Size(1280, 640),
                                          &baseline);
            RCLCPP_INFO_STREAM(this->get_logger(), "=> camera Matrix L: \n" << cameraMatrix_left);
            RCLCPP_INFO_STREAM(this->get_logger(), "=> camera Matrix R: \n" << cameraMatrix_right);
            RCLCPP_INFO_STREAM(this->get_logger(), "=> baseline: " << baseline);
        }
        // ----> Initialize calibration

        // Infinite video grabbing loop
        cv::Mat frameBGR, left_raw, right_raw, combine_nv12, left_rect, right_rect;
        while (1)
        {
            // Get last available frame
            const sl_oc::video::Frame frame = cap_0.getLastFrame();

            // ----> If the frame is valid we can display it
            if (frame.data != nullptr && frame.timestamp != last_frame_timestamp_)
            {
                last_frame_timestamp_ = frame.timestamp;
                // ----> Conversion from YUV 4:2:2 to BGR for visualization
                cv::Mat frameYUV = cv::Mat(frame.height, frame.width, CV_8UC2, frame.data);
                // <---- Conversion from YUV 4:2:2 to BGR for visualization
                cv::cvtColor(frameYUV, frameBGR, cv::COLOR_YUV2BGR_YUYV);
                // ----> Extract left and right images from side-by-side
                left_raw = frameBGR(cv::Rect(0, 0, frameBGR.cols / 2, frameBGR.rows));
                right_raw = frameBGR(cv::Rect(frameBGR.cols / 2, 0, frameBGR.cols / 2, frameBGR.rows));
                RCLCPP_INFO_ONCE(this->get_logger(), "=> img size: [%d, %d]", left_raw.cols, left_raw.rows);

                if (show_raw_and_rectify_)
                {
                    cv::resize(left_raw, left_raw, cv::Size(1280, 640));
                    cv::resize(right_raw, right_raw, cv::Size(1280, 640));
                    cv::remap(left_raw, left_rect, map_left_x, map_left_y, cv::INTER_LINEAR);
                    cv::remap(right_raw, right_rect, map_right_x, map_right_y, cv::INTER_LINEAR);
                    cv::Mat frameBGR1, frameBGR2;
                    cv::hconcat(left_raw, right_raw, frameBGR1);
                    cv::hconcat(left_rect, right_rect, frameBGR2);
                    cv::vconcat(frameBGR1, frameBGR2, frameBGR);
                    RCLCPP_INFO_ONCE(this->get_logger(), "\033[31m=> show raw and rectify : [%d, %d]\033[0m", frameBGR.cols, frameBGR.rows);
                }
                else
                {
                    if (need_rectify_)
                    {
                        cv::remap(left_raw, left_rect, map_left_x, map_left_y, cv::INTER_LINEAR);
                        cv::remap(right_raw, right_rect, map_right_x, map_right_y, cv::INTER_LINEAR);
                        RCLCPP_INFO_ONCE(this->get_logger(), "\033[31m=> rectify and resize img: [%d, %d]\033[0m", left_rect.cols, left_rect.rows);
                        cv::vconcat(left_rect, right_rect, frameBGR);
                        if (save_image_)
                        {
                            cnt++;
                            if (cnt == 10)
                            {
                                cv::imwrite("left.png", left_rect);
                                cv::imwrite("right.png", right_rect);
                                cv::Mat left_rect_nv12, right_rect_nv12;
                                bgr_to_nv12(left_rect, left_rect_nv12);
                                bgr_to_nv12(right_rect, right_rect_nv12);
                                save_mat_to_bin(left_rect_nv12, "left.nv12");
                                save_mat_to_bin(right_rect_nv12, "right.nv12");
                                save_image_ = false;
                                RCLCPP_INFO_ONCE(this->get_logger(), "\033[31m=> save image!\033[0m");
                            }
                        }
                    }
                    else
                    {
                        cv::resize(left_raw, left_raw, cv::Size(1280, 640));
                        cv::resize(right_raw, right_raw, cv::Size(1280, 640));
                        RCLCPP_INFO_ONCE(this->get_logger(), "\033[31m=> resize img: [%d, %d]\033[0m", left_raw.cols, left_raw.rows);
                        cv::vconcat(left_raw, right_raw, frameBGR);
                    }
                }

                bgr_to_nv12(frameBGR, combine_nv12);

                // create ros msg
                auto stereo_msg = std::make_shared<sensor_msgs::msg::Image>();
                stereo_msg->header.stamp = this->now();
                stereo_msg->header.frame_id = "pcl_link";
                stereo_msg->height = frameBGR.rows;
                stereo_msg->width = frameBGR.cols;
                stereo_msg->encoding = "nv12";
                stereo_msg->is_bigendian = false;
                stereo_msg->step = frameBGR.cols;
                stereo_msg->data.assign(combine_nv12.data, combine_nv12.data + (combine_nv12.rows * combine_nv12.cols));

                RCLCPP_INFO_ONCE(this->get_logger(), "\033[31m=> zed frame pub\033[0m");
                stereo_msg_pub_->publish(*stereo_msg);
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "\033[31m=> zed frame error!\033[0m");
            }
        }
        // ======================================================================================================================================
    }

private:
    void bgr24_to_nv12_neon(uint8_t *bgr24, uint8_t *nv12, int width, int height)
    {
        int frameSize = width * height;
        int yIndex = 0;
        int uvIndex = frameSize;
        const uint16x8_t u16_rounding = vdupq_n_u16(128);
        const int16x8_t s16_rounding = vdupq_n_s16(128);
        const int8x8_t s8_rounding = vdup_n_s8(128);
        const uint8x16_t offset = vdupq_n_u8(16);
        const uint16x8_t mask = vdupq_n_u16(255);

        for (int j = 0; j < height; j++)
        {
            for (int i = 0; i < width >> 4; i++)
            {
                // Load rgb
                uint8x16x3_t pixel_rgb;
                pixel_rgb = vld3q_u8(bgr24);
                bgr24 += 48;

                uint8x8x2_t uint8_r;
                uint8x8x2_t uint8_g;
                uint8x8x2_t uint8_b;
                uint8_r.val[0] = vget_low_u8(pixel_rgb.val[2]);
                uint8_r.val[1] = vget_high_u8(pixel_rgb.val[2]);
                uint8_g.val[0] = vget_low_u8(pixel_rgb.val[1]);
                uint8_g.val[1] = vget_high_u8(pixel_rgb.val[1]);
                uint8_b.val[0] = vget_low_u8(pixel_rgb.val[0]);
                uint8_b.val[1] = vget_high_u8(pixel_rgb.val[0]);

                uint16x8x2_t uint16_y;
                uint8x8_t scalar = vdup_n_u8(66);
                uint8x16_t y;

                uint16_y.val[0] = vmull_u8(uint8_r.val[0], scalar);
                uint16_y.val[1] = vmull_u8(uint8_r.val[1], scalar);
                scalar = vdup_n_u8(129);
                uint16_y.val[0] = vmlal_u8(uint16_y.val[0], uint8_g.val[0], scalar);
                uint16_y.val[1] = vmlal_u8(uint16_y.val[1], uint8_g.val[1], scalar);
                scalar = vdup_n_u8(25);
                uint16_y.val[0] = vmlal_u8(uint16_y.val[0], uint8_b.val[0], scalar);
                uint16_y.val[1] = vmlal_u8(uint16_y.val[1], uint8_b.val[1], scalar);

                uint16_y.val[0] = vaddq_u16(uint16_y.val[0], u16_rounding);
                uint16_y.val[1] = vaddq_u16(uint16_y.val[1], u16_rounding);

                y = vcombine_u8(vqshrn_n_u16(uint16_y.val[0], 8), vqshrn_n_u16(uint16_y.val[1], 8));
                y = vaddq_u8(y, offset);

                vst1q_u8(nv12 + yIndex, y);
                yIndex += 16;

                // Compute u and v in the even row
                if (j % 2 == 0)
                {
                    int16x8_t u_scalar = vdupq_n_s16(-38);
                    int16x8_t v_scalar = vdupq_n_s16(112);

                    int16x8_t r = vreinterpretq_s16_u16(vandq_u16(vreinterpretq_u16_u8(pixel_rgb.val[2]), mask));
                    int16x8_t g = vreinterpretq_s16_u16(vandq_u16(vreinterpretq_u16_u8(pixel_rgb.val[1]), mask));
                    int16x8_t b = vreinterpretq_s16_u16(vandq_u16(vreinterpretq_u16_u8(pixel_rgb.val[0]), mask));

                    int16x8_t u;
                    int16x8_t v;
                    uint8x8x2_t uv;

                    u = vmulq_s16(r, u_scalar);
                    v = vmulq_s16(r, v_scalar);

                    u_scalar = vdupq_n_s16(-74);
                    v_scalar = vdupq_n_s16(-94);
                    u = vmlaq_s16(u, g, u_scalar);
                    v = vmlaq_s16(v, g, v_scalar);

                    u_scalar = vdupq_n_s16(112);
                    v_scalar = vdupq_n_s16(-18);
                    u = vmlaq_s16(u, b, u_scalar);
                    v = vmlaq_s16(v, b, v_scalar);

                    u = vaddq_s16(u, s16_rounding);
                    v = vaddq_s16(v, s16_rounding);

                    uv.val[0] = vreinterpret_u8_s8(vadd_s8(vqshrn_n_s16(u, 8), s8_rounding));
                    uv.val[1] = vreinterpret_u8_s8(vadd_s8(vqshrn_n_s16(v, 8), s8_rounding));

                    vst2_u8(nv12 + uvIndex, uv);

                    uvIndex += 16;
                }
            }
        }
    }

    void bgr_to_nv12(const cv::Mat &bgr, cv::Mat &nv12)
    {
        int width = bgr.cols;
        int height = bgr.rows;
        nv12 = cv::Mat(height * 3 / 2, width, CV_8UC1);
        bgr24_to_nv12_neon(bgr.data, nv12.data, width, height);
    }

    void save_mat_to_bin(const cv::Mat &mat, const std::string &filename)
    {
        // 检查矩阵是否为空
        if (mat.empty())
        {
            std::cerr << "The input matrix is empty!" << std::endl;
            return;
        }

        // 打开文件输出流，并以二进制模式写入
        std::ofstream ofs(filename, std::ios::binary);
        if (!ofs.is_open())
        {
            std::cerr << "Failed to open file for writing!" << std::endl;
            return;
        }

        // 获取数据指针及大小
        const uchar *dataPtr = mat.data;
        size_t dataSize = mat.total() * mat.elemSize();

        // 将数据写入文件
        ofs.write(reinterpret_cast<const char *>(dataPtr), dataSize);

        // 关闭文件
        ofs.close();
    }

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr stereo_msg_pub_; // stereo image publisher
    bool need_rectify_;
    bool show_raw_and_rectify_;
    bool save_image_;

    uint64_t last_frame_timestamp_ = 0;

    int cnt = 0;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PubStereoImgsNv12Node>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}