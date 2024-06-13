// Copyright (c) 2022，Horizon Robotics.
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

  // 3.crop原图中心部分
  auto dst_height = src_height / 2;
  auto dst_width = src_width / 2;

  // 4.crop图像、测试耗时，输入图像为Mat格式
  {
    RCLCPP_INFO(rclcpp::get_logger("example"), "%s", "Test hobotcv_crop: Mat Input, VPS Process");
    auto before_crop = std::chrono::system_clock::now();
    // =============================================
    auto cropmat = hobot_cv::hobotcv_crop(srcmat_nv12, src_height, src_width, dst_height, dst_width, cv::Range(0, dst_height), cv::Range(0, dst_width), hobot_cv::HOBOTCV_VPS);
    // =============================================
    auto after_crop = std::chrono::system_clock::now();
    auto interval = std::chrono::duration_cast<std::chrono::milliseconds>(after_crop - before_crop).count();
    std::stringstream ss;
    ss << "source image " << image_file << " is " << src_width << "x" << src_height << " pixels";
    RCLCPP_INFO(rclcpp::get_logger("example"), "%s", ss.str().c_str());

    std::stringstream ss_crop;
    ss_crop << "crop image to " << dst_width << "x" << dst_height << " pixels" << ", mat input, time cost: " << interval << " ms";
    RCLCPP_INFO(rclcpp::get_logger("example"), "%s", ss_crop.str().c_str());
    // 保存结果
    writeImg(cropmat, "./crop_mat_input_vps_process_1.jpg");
  }

  // 4.crop图像、测试耗时，输入图像为数组格式
  {
    RCLCPP_INFO(rclcpp::get_logger("example"), "%s", "Test hobotcv_crop: Arr Input, VPS Process");
    auto before_crop = std::chrono::system_clock::now();
    // =============================================
    auto imageInfo = hobot_cv::hobotcv_crop(reinterpret_cast<const char *>(srcmat_nv12.data), src_height, src_width, dst_height, dst_width, cv::Range(0, dst_height), cv::Range(0, dst_width), hobot_cv::HOBOTCV_VPS);
    auto after_crop = std::chrono::system_clock::now();
    // =============================================
    auto interval = std::chrono::duration_cast<std::chrono::milliseconds>(after_crop - before_crop).count();
    std::stringstream ss;
    ss << "source image " << image_file << " is " << src_width << "x" << src_height << " pixels";
    RCLCPP_INFO(rclcpp::get_logger("example"), "%s", ss.str().c_str());
    if (imageInfo != nullptr) {
      std::stringstream ss_crop;
      ss_crop << "crop image to " << dst_width << "x"<< dst_height << " pixels"<< ", arr input, time cost: " << interval << " ms";
      RCLCPP_INFO(rclcpp::get_logger("example"), "%s", ss_crop.str().c_str());
      cv::Mat dst_mat(imageInfo->height * 3 / 2, imageInfo->width, CV_8UC1, imageInfo->imageAddr);
      // 保存结果
      writeImg(dst_mat, "./crop_arr_input_vps_process_1.jpg");
    }
  }

  // 4.crop图像、测试耗时，输入图像为Mat格式，会缩放再截取
  {
    RCLCPP_INFO(rclcpp::get_logger("example"), "%s", "Test hobotcv_crop: Mat Input, VPS Process");
    auto before_crop = std::chrono::system_clock::now();
    // =============================================
    auto cropmat = hobot_cv::hobotcv_crop(srcmat_nv12, src_height, src_width, dst_height, dst_width, cv::Range(0, dst_height+10), cv::Range(0, dst_width+10), hobot_cv::HOBOTCV_VPS);
    // =============================================
    auto after_crop = std::chrono::system_clock::now();
    auto interval = std::chrono::duration_cast<std::chrono::milliseconds>(after_crop - before_crop).count();
    std::stringstream ss;
    ss << "source image " << image_file << " is " << src_width << "x" << src_height << " pixels";
    RCLCPP_INFO(rclcpp::get_logger("example"), "%s", ss.str().c_str());

    std::stringstream ss_crop;
    ss_crop << "crop image to " << dst_width << "x" << dst_height << " pixels" << ", mat input, time cost: " << interval << " ms";
    RCLCPP_INFO(rclcpp::get_logger("example"), "%s", ss_crop.str().c_str());
    // 保存结果
    writeImg(cropmat, "./crop_mat_input_vps_process_2.jpg");
  }

  // 4.crop图像、测试耗时，输入图像为数组格式，会缩放再截取
  {
    RCLCPP_INFO(rclcpp::get_logger("example"), "%s", "Test hobotcv_crop: Arr Input, VPS Process");
    auto before_crop = std::chrono::system_clock::now();
    // =============================================
    auto imageInfo = hobot_cv::hobotcv_crop(reinterpret_cast<const char *>(srcmat_nv12.data), src_height, src_width, dst_height, dst_width, cv::Range(0, dst_height+10), cv::Range(0, dst_width+10), hobot_cv::HOBOTCV_VPS);
    auto after_crop = std::chrono::system_clock::now();
    // =============================================
    auto interval = std::chrono::duration_cast<std::chrono::milliseconds>(after_crop - before_crop).count();
    std::stringstream ss;
    ss << "source image " << image_file << " is " << src_width << "x" << src_height << " pixels";
    RCLCPP_INFO(rclcpp::get_logger("example"), "%s", ss.str().c_str());
    if (imageInfo != nullptr) {
      std::stringstream ss_crop;
      ss_crop << "crop image to " << dst_width << "x"<< dst_height << " pixels"<< ", arr input, time cost: " << interval << " ms";
      RCLCPP_INFO(rclcpp::get_logger("example"), "%s", ss_crop.str().c_str());
      cv::Mat dst_mat(imageInfo->height * 3 / 2, imageInfo->width, CV_8UC1, imageInfo->imageAddr);
      // 保存结果
      writeImg(dst_mat, "./crop_arr_input_vps_process_2.jpg");
    }
  }

  return 0;
}
