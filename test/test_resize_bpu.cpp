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
#include "opencv2/core/mat.hpp"
#include "opencv2/core/types.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/opencv.hpp"
#include "rclcpp/rclcpp.hpp"
#include "utils.h"

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

  // 3.目标图像缩小为原图一半
  auto dst_height = src_height / 2;
  auto dst_width = src_width / 2;

  // 4.缩放图像、测试耗时，输入图像为Mat格式
  {
    RCLCPP_INFO(rclcpp::get_logger("example"), "%s", "Test hobotcv_resize: Mat Input, BPU Process");
    auto before_resize = std::chrono::system_clock::now();
    // =============================================
    cv::Mat dstmat_nv12;
    auto ret = hobot_cv::hobotcv_resize(srcmat_nv12, src_height, src_width, dstmat_nv12, dst_height, dst_width, hobot_cv::HOBOTCV_BPU);
    // =============================================
    auto after_resize = std::chrono::system_clock::now();
    auto interval = std::chrono::duration_cast<std::chrono::milliseconds>(after_resize - before_resize).count();
    std::stringstream ss;
    ss << "\n" << "source image " << image_file << " is " << src_width << "x" << src_height << " pixels";
    RCLCPP_INFO(rclcpp::get_logger("example"), "%s", ss.str().c_str());
    if (0 == ret) {
      std::stringstream ss_resize;
      ss_resize << "resize image to " << dst_width << "x" << dst_height << " pixels" << ", time cost: " << interval << " ms";
      RCLCPP_INFO(rclcpp::get_logger("example"), "%s", ss_resize.str().c_str());
      // 保存结果
      writeImg(dstmat_nv12, "./resize_mat_input_bpu_process.jpg");
    }
  }

  // 5.缩放图像、测试耗时，输入图像为数组格式
  {
    RCLCPP_INFO(rclcpp::get_logger("example"), "%s", "Test hobotcv_resize: Arr Input, BPU Process");
    auto before_resize = std::chrono::system_clock::now();
    // =============================================
    auto imageInfo = hobot_cv::hobotcv_resize(reinterpret_cast<const char *>(srcmat_nv12.data), src_height, src_width, dst_height, dst_width, hobot_cv::HOBOTCV_BPU);
    // =============================================
    auto after_resize = std::chrono::system_clock::now();
    auto interval = std::chrono::duration_cast<std::chrono::milliseconds>(after_resize - before_resize).count();
    std::stringstream ss;
    ss << "\n" << "source image " << image_file << " is " << src_width << "x" << src_height << " pixels";
    RCLCPP_INFO(rclcpp::get_logger("example"), "%s", ss.str().c_str());
    if (imageInfo != nullptr) {
      std::stringstream ss_resize;
      ss_resize << "resize image to " << dst_width << "x" << dst_height << " pixels" << ", time cost: " << interval << " ms";
      RCLCPP_INFO(rclcpp::get_logger("example"), "%s", ss_resize.str().c_str());
      // 保存结果
      cv::Mat dst_mat(imageInfo->height * 3 / 2, imageInfo->width, CV_8UC1, imageInfo->imageAddr);
      writeImg(dst_mat, "./resize_arr_input_bpu_process.jpg");
    }
  }

  return 0;
}
