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
#include "stereo_rectify.h"
#include "blockqueue.h"

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
        save_origin_image_ = this->declare_parameter("save_origin_image", false);
        user_rectify_ = this->declare_parameter("user_rectify", false);
        stereo_calib_file_path_ = this->declare_parameter("stereo_calib_file_path", "stereo_8_zed_mini.yaml");
        resolution_ = this->declare_parameter("resolution", "720p");
        brightness_ = this->declare_parameter("brightness", 5);
        sharp_ = this->declare_parameter("sharp", 4);
        contrast_ = this->declare_parameter("contrast", 4);
        sat_ = this->declare_parameter("sat", 4);
        gamma_ = this->declare_parameter("gamma", 5);
        dst_width_ = this->declare_parameter("dst_width", 1280);
        dst_height_ = this->declare_parameter("dst_height", 640);
        zed_pub_bgr_ = this->declare_parameter("zed_pub_bgr", false);
        if (show_raw_and_rectify_)
            need_rectify_ = true;
        RCLCPP_INFO_STREAM(this->get_logger(), "\033[31m" << std::endl
                                                          << "=> need_rectify: " << need_rectify_ << std::endl
                                                          << "=> show_raw_and_rectify: " << show_raw_and_rectify_ << std::endl
                                                          << "=> save_image: " << save_image_ << std::endl
                                                          << "=> save_origin_image: " << save_origin_image_ << std::endl
                                                          << "=> user_rectify: " << user_rectify_ << std::endl
                                                          << "=> resolution: " << resolution_ << std::endl
                                                          << "=> brightness: " << brightness_ << std::endl
                                                          << "=> sharp: " << sharp_ << std::endl
                                                          << "=> contrast: " << contrast_ << std::endl
                                                          << "=> sat: " << sat_ << std::endl
                                                          << "=> gamma: " << gamma_ << std::endl
                                                          << "=> [dst_width, dst_height]: [" << dst_width_ << ", " << dst_height_ << "]" << std::endl
                                                          << "=> zed_pub_bgr: " << zed_pub_bgr_ << std::endl
                                                          << "\033[0m");
        if (user_rectify_)
        {
            RCLCPP_INFO_STREAM(this->get_logger(), "\033[31m" << "=> stereo_calib_file_path: " << stereo_calib_file_path_ << "\033[0m");
        }
        dst_size_ = cv::Size(dst_width_, dst_height_);

        // ======================================================================================================================================
        // sub & pub
        // auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
        // qos.reliability(rclcpp::ReliabilityPolicy::Reliable);
        // qos.durability(rclcpp::DurabilityPolicy::TransientLocal);
        stereo_msg_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/image_combine_raw", 10);

        // ======================================================================================================================================
        int result = init_zed_cam();
        if (result != 0)
        {
            rclcpp::shutdown();
            return;
        }

        if (need_rectify_)
        {
            result = load_calib_file();
            if (result != 0)
            {
                rclcpp::shutdown();
                return;
            }
        }

        pub_stereo_imgs();

        //  pub_thread_ = std::make_shared<std::thread>([this] { pub_func(); });
        // ======================================================================================================================================
    }

private:
    // ======================================================================================================================================
    /**
     * @brief init zed cam
     */
    int init_zed_cam()
    {
        // ======================================================================================================================================
        sl_oc::video::VideoParams params;
        if (resolution_ == "360p")
        {
            params.res = sl_oc::video::RESOLUTION::VGA;
        }
        else if (resolution_ == "1080p")
        {
            params.res = sl_oc::video::RESOLUTION::HD1080;
        }
        else if (resolution_ == "2K")
        {
            params.res = sl_oc::video::RESOLUTION::HD2K;
        }
        else
        {
            params.res = sl_oc::video::RESOLUTION::HD720;
        }
        params.fps = sl_oc::video::FPS::FPS_15;
        // ----> Create Video Capture
        cap_0_ = std::make_shared<sl_oc::video::VideoCapture>(params);
        if (!cap_0_->initializeVideo())
        {
            RCLCPP_ERROR(this->get_logger(), "\033[31m=> cannot open camera video capture!\033[0m");
            return -1;
        }
        int serial_num = cap_0_->getSerialNumber();
        if (serial_num == -1)
        {
            RCLCPP_ERROR(this->get_logger(), "\033[31m=> serial_num error!\033[0m");
            return -1;
        }
        RCLCPP_INFO_STREAM(this->get_logger(), "=> connected to camera sn: " << serial_num<< "[" << cap_0_->getDeviceName() << "]");
        // <---- Create Video Capture

        // ======================================================================================================================================
        cap_0_->setBrightness(brightness_);
        cap_0_->setSharpness(sharp_);
        cap_0_->setContrast(contrast_);
        cap_0_->setSaturation(sat_);
        cap_0_->setAutoWhiteBalance(true);
        cap_0_->setGamma(gamma_);
        cap_0_->setAECAGC(true);

        // cap_0_->setExposure((sl_oc::video::CAM_SENS_POS)0, 48);
        // cap_0_->setExposure((sl_oc::video::CAM_SENS_POS)1, 48);
        // cap_0_->setGain((sl_oc::video::CAM_SENS_POS)0, 12);
        // cap_0_->setGain((sl_oc::video::CAM_SENS_POS)1, 12);
        // cap_0_->setWhiteBalance();

        // ======================================================================================================================================
        return 0;
    }

    /**
     * @brief load calib file
     */
    int load_calib_file()
    {
        // ----> Initialize calibration
        if (user_rectify_)
        {
            cv::FileStorage fs(stereo_calib_file_path_, cv::FileStorage::READ);
            if (!fs.isOpened())
            {
                RCLCPP_WARN_STREAM(this->get_logger(), "=> failed to open " << stereo_calib_file_path_);
                return -1;
            }
            rectify_ = std::make_shared<stereonet::StereoRectify>(fs["stereo0"], dst_width_, dst_height_);
        }
        else
        {
            // ----> Retrieve calibration file from Stereolabs server
            // ZED Calibration
            // Download camera calibration file
            std::string calibration_file;
            if (!sl_oc::tools::downloadCalibrationFile(cap_0_->getSerialNumber(), calibration_file))
            {
                RCLCPP_ERROR(this->get_logger(), "=> could not load calibration file from Stereolabs servers");
                return -1;
            }
            RCLCPP_INFO(this->get_logger(), "=> calibration file found. Loading...");
            // ----> Frame size
            int w, h;
            cap_0_->getFrameSize(w, h);
            // <---- Frame size
            cv::Mat cameraMatrix_left, cameraMatrix_right;
            double baseline = 0.0;
            sl_oc::tools::initCalibration(calibration_file, cv::Size(w / 2, h), map_left_x_, map_left_y_, map_right_x_, map_right_y_, cameraMatrix_left, cameraMatrix_right, dst_size_, &baseline);
            RCLCPP_INFO_STREAM(this->get_logger(), "=> camera Matrix L: \n" << cameraMatrix_left);
            RCLCPP_INFO_STREAM(this->get_logger(), "=> camera Matrix R: \n" << cameraMatrix_right);
            RCLCPP_INFO(this->get_logger(), "\033[31m=> rectified fx: %f, fy: %f, cx: %f, cy: %f, base_line: %f\033[0m", cameraMatrix_left.at<double>(0, 0), cameraMatrix_left.at<double>(1, 1),
                        cameraMatrix_left.at<double>(0, 2), cameraMatrix_left.at<double>(1, 2), baseline * 1e-3);
            RCLCPP_INFO(this->get_logger(), "=> camera_fx:=%f camera_fy:=%f camera_cx:=%f camera_cy:=%f base_line:=%f", cameraMatrix_left.at<double>(0, 0), cameraMatrix_left.at<double>(1, 1),
                        cameraMatrix_left.at<double>(0, 2), cameraMatrix_left.at<double>(1, 2), baseline * 1e-3);
        }
        // ----> Initialize calibration
        return 0;
    }

    /**
     * @brief pub stereo imgs
     */
    void pub_stereo_imgs()
    {
        // Infinite video grabbing loop
        cv::Mat frameBGR, left_raw, right_raw, combine_nv12, left_rect, right_rect;
        while (rclcpp::ok())
        {
            // Get last available frame
            const sl_oc::video::Frame frame = cap_0_->getLastFrame();

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
                RCLCPP_INFO_ONCE(this->get_logger(), "\033[31m=> raw img size: [%d, %d]\033[0m", left_raw.cols, left_raw.rows);

                // ------------------------------------------------------- rectify imgs --------------------------------------------------
                if (need_rectify_)
                {
                    if (user_rectify_)
                    {
                        rectify_->Rectify(left_raw, right_raw, left_rect, right_rect);
                    }
                    else
                    {
                        cv::remap(left_raw, left_rect, map_left_x_, map_left_y_, cv::INTER_LINEAR);
                        cv::remap(right_raw, right_rect, map_right_x_, map_right_y_, cv::INTER_LINEAR);
                    }
                }

                // ------------------------------------------------------- build result --------------------------------------------------
                if (show_raw_and_rectify_)
                {
                    cv::resize(left_raw, left_raw, dst_size_);
                    cv::resize(right_raw, right_raw, dst_size_);
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
                        cv::vconcat(left_rect, right_rect, frameBGR);
                        RCLCPP_INFO_ONCE(this->get_logger(), "\033[31m=> rectify img size: [%d, %d]\033[0m", left_rect.cols, left_rect.rows);
                        if (save_image_ && !save_origin_image_)
                        {
                            save_images(left_rect, right_rect, frame.timestamp, "jpg");
                            RCLCPP_INFO(this->get_logger(), "=> save image: [%ld]", frame.timestamp);
                        }
                    }
                    else
                    {
                        cv::resize(left_raw, left_raw, dst_size_);
                        cv::resize(right_raw, right_raw, dst_size_);
                        cv::vconcat(left_raw, right_raw, frameBGR);
                        RCLCPP_INFO_ONCE(this->get_logger(), "\033[31m=> resize img: [%d, %d]\033[0m", left_raw.cols, left_raw.rows);
                    }
                }
                
                if (save_origin_image_) {
                    save_images(left_raw, right_raw, frame.timestamp, "jpg");
                    RCLCPP_INFO(this->get_logger(), "=> save origin image: [%ld]", frame.timestamp);
                }

                // ------------------------------------------------------- pub msg -------------------------------------------------------
                auto stereo_msg = std::make_shared<sensor_msgs::msg::Image>();
                stereo_msg->header.stamp = rclcpp::Time(frame.timestamp);
                stereo_msg->header.frame_id = "pcl_link";
                stereo_msg->height = frameBGR.rows;
                stereo_msg->width = frameBGR.cols;
                stereo_msg->is_bigendian = false;
                if (zed_pub_bgr_)
                {
                    stereo_msg->encoding = "bgr8";
                    stereo_msg->step = frameBGR.cols * 3;
                    stereo_msg->data.assign(frameBGR.data, frameBGR.data + (frameBGR.rows * frameBGR.cols) * 3);
                }
                else
                {
                    bgr_to_nv12(frameBGR, combine_nv12);
                    stereo_msg->encoding = "nv12";
                    stereo_msg->step = frameBGR.cols;
                    stereo_msg->data.assign(combine_nv12.data, combine_nv12.data + (combine_nv12.rows * combine_nv12.cols));
                }
                stereo_msg_pub_->publish(*stereo_msg);

                // if (pub_que_.size() > 5)
                // {
                //     RCLCPP_WARN(this->get_logger(), "\033[31m=> zed frame full, size: %d!\033[0m", pub_que_.size());
                // }
                // else
                // {
                //     stereo_msg_pub_->publish(*stereo_msg);
                //     // pub_que_.put(stereo_msg);
                // }
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "\033[31m=> zed frame error!\033[0m");
            }
        }
    }

    /**
     * @brief pub zed image
     */
    // void pub_func()
    // {
    //     while (rclcpp::ok())
    //     {
    //         std::shared_ptr<sensor_msgs::msg::Image> stereo_msg;
    //         if (pub_que_.get(stereo_msg))
    //         {
    //             RCLCPP_INFO_ONCE(this->get_logger(), "\033[31m=> zed frame pub\033[0m");
    //             stereo_msg_pub_->publish(*stereo_msg);
    //         }
    //     }
    // }

    // ======================================================================================================================================
    /**
     * @brief convert bgr arr to nv12 arr
     */
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

    /**
     * @brief convert bgr mat to nv12 mat
     */
    void bgr_to_nv12(const cv::Mat &bgr, cv::Mat &nv12)
    {
        int width = bgr.cols;
        int height = bgr.rows;
        nv12 = cv::Mat(height * 3 / 2, width, CV_8UC1);
        bgr24_to_nv12_neon(bgr.data, nv12.data, width, height);
    }

    /**
     * @brief save mat to bin file
     */
    void save_mat_to_bin(const cv::Mat &mat, const std::string &filename)
    {
        if (mat.empty())
        {
            RCLCPP_ERROR(this->get_logger(), "=> the input matrix is empty!");
            return;
        }

        std::ofstream ofs(filename, std::ios::binary);
        if (!ofs.is_open())
        {
            RCLCPP_ERROR(this->get_logger(), "=> failed to open file for writing!");
            return;
        }

        const uchar *dataPtr = mat.data;
        size_t dataSize = mat.total() * mat.elemSize();

        ofs.write(reinterpret_cast<const char *>(dataPtr), dataSize);

        ofs.close();
    }

    /**
     * @brief save images
     */
    void save_images(cv::Mat &left_img, cv::Mat &right_img, uint64_t ts, const std::string &image_format)
    {
        static std::atomic_bool directory_created{false};
        static std::atomic_int i{0};
        std::stringstream iss;
        cv::Mat image_combine;
        if (!directory_created)
        {
            directory_created = true;
            system("mkdir -p"
                   " ./images/cam0/data/"
                   " ./images/cam1/data/"
                   " ./images/cam_combine/data/");
        }
        iss << std::setw(3) << std::setfill('0') << i++;
        auto image_seq = iss.str();
        cv::imwrite("./images/cam0/data/" + std::to_string(ts) + "." + image_format, left_img);
        cv::imwrite("./images/cam1/data/" + std::to_string(ts) + "." + image_format, right_img);
        // cv::vconcat(left_img, right_img, image_combine);
        // cv::imwrite("./images/cam_combine/data/combine_" + image_seq + image_format, image_combine);
    }
    // ======================================================================================================================================
    // zed cam cap
    std::shared_ptr<sl_oc::video::VideoCapture> cap_0_ = nullptr;

    // zed param
    std::string resolution_ = "720p";
    int brightness_ = 5;
    int sharp_ = 4;
    int contrast_ = 4;
    int sat_ = 4;
    int gamma_ = 5;

    // destination size
    int dst_width_ = 1280;
    int dst_height_ = 640;
    cv::Size dst_size_;

    // stereo image publisher
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr stereo_msg_pub_;
    // false: pub nv12 data, true: pub bgr data
    bool zed_pub_bgr_ = false;
    // blockqueue<std::shared_ptr<sensor_msgs::msg::Image>> pub_que_;
    // std::shared_ptr<std::thread> pub_thread_ = nullptr;

    // image rectify flag
    bool need_rectify_ = false;

    // if user_rectify_==false,
    cv::Mat map_left_x_, map_left_y_;
    cv::Mat map_right_x_, map_right_y_;

    // false: loading ZED rectify parameters, true: loading user-calibrated rectify parameters
    bool user_rectify_ = false;
    // if user_rectify_==true, loading user-calibrated rectify parameters from `stereo_calib_file_path_`
    std::string stereo_calib_file_path_ = "stereo_8_zed_mini.yaml";
    // if user_rectify_==true, `rectify_` need init
    std::shared_ptr<stereonet::StereoRectify> rectify_ = nullptr;

    // pub raw stereo imgs and rectify stereo imgs
    bool show_raw_and_rectify_;

    // save stereo image flag
    bool save_image_;
    
    // save stereo image that before rectify flag
    bool save_origin_image_;

    // last frame timestamp
    uint64_t last_frame_timestamp_ = 0;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PubStereoImgsNv12Node>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
