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
#include <chrono>
#include <fstream>
#include <iostream>
#include <string>

#include "hobotcv_imgproc/hobotcv_imgproc.h"
#include "include/utils.h"
#include "opencv2/core/mat.hpp"
#include "opencv2/core/types.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/opencv.hpp"
#include "rclcpp/rclcpp.hpp"

void writeImg(cv::Mat &mat, std::string imgfile) {
  cv::Mat img_bgr;
  cv::cvtColor(mat, img_bgr, cv::COLOR_YUV2BGR_NV12);
  cv::imwrite(imgfile, img_bgr);
}

int main() {
  // 1.读入图片
  std::string image_file = "./config/test.jpg";
  cv::Mat bgr_mat = cv::imread(image_file, cv::IMREAD_COLOR);
  auto src_height = bgr_mat.rows;
  auto src_width = bgr_mat.cols;

  // 2.转为nv12格式
  cv::Mat srcmat_nv12;
  BGRToNv12(bgr_mat, srcmat_nv12);

  // 3.rotate图像、测试耗时，输入图像为Mat格式
  {
    RCLCPP_INFO(rclcpp::get_logger("example"), "%s", "Test hobotcv_rotate: Mat Input, VPS Process");
    auto before_rotate = std::chrono::system_clock::now();
    // =============================================
    cv::Mat dstmat_nv12(src_height * 3 / 2, src_width, CV_8UC1);
    auto ret = hobot_cv::hobotcv_rotate(srcmat_nv12, dstmat_nv12, hobot_cv::ROTATION_180);
    // =============================================
    auto after_rotate = std::chrono::system_clock::now();
    auto interval = std::chrono::duration_cast<std::chrono::milliseconds>(after_rotate - before_rotate).count();
    std::stringstream ss;
    ss << "source image " << image_file << " is " << src_width << "x" << src_height << " pixels";
    RCLCPP_INFO(rclcpp::get_logger("example"), "%s", ss.str().c_str());

    if (ret == 0) {
      std::stringstream ss_rotate;
      ss_rotate << "rotate image 180°, mat input, time cost: " << interval << " ms";
      RCLCPP_INFO(rclcpp::get_logger("example"), "%s", ss_rotate.str().c_str());
      // 保存结果
      writeImg(dstmat_nv12, "./rotate_mat_input_vps_process.jpg");
    }
  }

  // 4.rotate图像、测试耗时，输入图像为数组格式
  {
    RCLCPP_INFO(rclcpp::get_logger("example"), "%s", "Test hobotcv_rotate: Arr Input, VPS Process");
    auto before_rotate = std::chrono::system_clock::now();
    // =============================================
    auto imageInfo = hobot_cv::hobotcv_rotate(reinterpret_cast<const char *>(srcmat_nv12.data), src_height, src_width, hobot_cv::ROTATION_180);
    auto after_rotate = std::chrono::system_clock::now();
    // =============================================
    auto interval = std::chrono::duration_cast<std::chrono::milliseconds>(after_rotate - before_rotate).count();
    std::stringstream ss;
    ss << "source image " << image_file << " is " << src_width << "x" << src_height << " pixels";
    RCLCPP_INFO(rclcpp::get_logger("example"), "%s", ss.str().c_str());
    if (imageInfo != nullptr) {
      std::stringstream ss_rotate;
      ss_rotate << "rotate image 180°, arr input, time cost: " << interval << " ms";
      RCLCPP_INFO(rclcpp::get_logger("example"), "%s", ss_rotate.str().c_str());
      cv::Mat dst_mat(imageInfo->height * 3 / 2, imageInfo->width, CV_8UC1, imageInfo->imageAddr);
      // 保存结果
      writeImg(dst_mat, "./rotate_arr_input_vps_process.jpg");
    }
  }

  return 0;
}
