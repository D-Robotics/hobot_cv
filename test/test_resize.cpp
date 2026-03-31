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

#include "opencv2/opencv.hpp"
#include "opencv2/imgcodecs.hpp"
#include "rclcpp/rclcpp.hpp"

#include "hobotcv_imgproc/hobotcv_imgproc.h"
#include "utils.h"

void writeImg(cv::Mat &mat, std::string imgfile) {
  cv::Mat img_bgr;
  cv::cvtColor(mat, img_bgr, cv::COLOR_YUV2BGR_NV12);
  cv::imwrite(imgfile, img_bgr);
}

int main() {
  std::string image_file = "config/test.jpg";
  cv::Mat bgr_mat = cv::imread(image_file, cv::IMREAD_COLOR);
  auto src_height = bgr_mat.rows;
  auto src_width = bgr_mat.cols;

  cv::Mat srcmat_nv12;
  BGRToNv12(bgr_mat, srcmat_nv12);
  auto dst_height = src_height / 2;
  auto dst_width = src_width / 2;
  auto dst_height_2 = 544;
  auto dst_width_2 = 960;
  cv::Mat dstmat_nv12(dst_height * 3 / 2, dst_width, CV_8UC1);
  cv::Mat dstmat_nv12_2(dst_height_2 * 3 / 2, dst_width_2, CV_8UC1);
  int loop = 0;
  do {
    {  // resize first
      auto before_resize = std::chrono::system_clock::now();
      auto ret = hobot_cv::hobotcv_resize(
          srcmat_nv12, src_height, src_width, dstmat_nv12, dst_height, dst_width);
      auto after_resize = std::chrono::system_clock::now();
      auto interval = std::chrono::duration_cast<std::chrono::milliseconds>(
                          after_resize - before_resize)
                          .count();
      std::stringstream ss;
      ss << "\n"
        << "source image " << image_file << " is " << src_width << "x"
        << src_height << " pixels";
      RCLCPP_INFO(rclcpp::get_logger("example"), "%s", ss.str().c_str());
      if (0 == ret) {
        std::stringstream ss_resize;
        ss_resize << "resize image to " << dst_width << "x" << dst_height
                  << " pixels"
                  << ", time cost: " << interval << " ms";
        RCLCPP_INFO(rclcpp::get_logger("example"), "%s", ss_resize.str().c_str());
        writeImg(dstmat_nv12, "./resize.jpg");
      }
    }

    {  // resieze second
      auto before_resize = std::chrono::system_clock::now();
      auto ret = hobot_cv::hobotcv_resize(
          srcmat_nv12, src_height, src_width, dstmat_nv12_2, dst_height_2, dst_width_2);
      auto after_resize = std::chrono::system_clock::now();
      auto interval = std::chrono::duration_cast<std::chrono::milliseconds>(
                          after_resize - before_resize)
                          .count();
      if (0 == ret) {
        std::stringstream ss_resize;
        ss_resize << "resize image to " << dst_width_2 << "x" << dst_height_2
                  << " pixels"
                  << ", time cost: " << interval << " ms";
        RCLCPP_INFO(rclcpp::get_logger("example"), "%s", ss_resize.str().c_str());
        writeImg(dstmat_nv12_2, "./resize_2.jpg");
      }
    }

    {
      // 选择一组触发条件的目标分辨率：src_w > dst_h && src_h < dst_w
      auto dst_height_neon = 400;
      auto dst_width_neon = 1200;

      RCLCPP_INFO(rclcpp::get_logger("example"),
                  "\n=== NEON fallback test ===\n"
                  "src(%d x %d) -> dst(%d x %d)\n"
                  "src_w(%d) > dst_h(%d) ? %s\n"
                  "src_h(%d) < dst_w(%d) ? %s",
                  src_width,
                  src_height,
                  dst_width_neon,
                  dst_height_neon,
                  src_width,
                  dst_height_neon,
                  (src_width > dst_height_neon) ? "YES -> NEON" : "NO",
                  src_height,
                  dst_width_neon,
                  (src_height < dst_width_neon) ? "YES -> NEON" : "NO");

      cv::Mat dstmat_neon(dst_height_neon * 3 / 2, dst_width_neon, CV_8UC1);

      auto before_resize = std::chrono::system_clock::now();
      auto ret = hobot_cv::hobotcv_resize(srcmat_nv12,
                                          src_height,
                                          src_width,
                                          dstmat_neon,
                                          dst_height_neon,
                                          dst_width_neon);
      auto after_resize = std::chrono::system_clock::now();
      auto interval = std::chrono::duration_cast<std::chrono::milliseconds>(
                          after_resize - before_resize)
                          .count();

      if (0 == ret) {
        std::stringstream ss_resize;
        ss_resize << "[NEON] resize image to " << dst_width_neon << "x"
                  << dst_height_neon << " pixels, time cost: " << interval
                  << " ms";
        RCLCPP_INFO(
            rclcpp::get_logger("example"), "%s", ss_resize.str().c_str());
        writeImg(dstmat_neon, "./resize_neon_fallback.jpg");
      } else {
        RCLCPP_ERROR(rclcpp::get_logger("example"),
                     "NEON fallback resize failed! ret=%d",
                     ret);
      }
    }

    {  // nv12 interface resieze
      auto before_resize = std::chrono::system_clock::now();
      auto imageInfo = hobot_cv::hobotcv_resize(
          reinterpret_cast<const char *>(srcmat_nv12.data),
          src_height,
          src_width,
          dst_height_2,
          dst_width_2);
      auto after_resize = std::chrono::system_clock::now();
      auto interval = std::chrono::duration_cast<std::chrono::milliseconds>(
                          after_resize - before_resize)
                          .count();
      if (imageInfo != nullptr) {
        std::stringstream ss_resize;
        ss_resize << "nv12 interface resize image to " << dst_width_2 << "x"
                  << dst_height_2 << " pixels"
                  << ", time cost: " << interval << " ms";
        RCLCPP_INFO(rclcpp::get_logger("example"), "%s", ss_resize.str().c_str());
        cv::Mat dst_mat(imageInfo->height * 3 / 2,
                        imageInfo->width,
                        CV_8UC1,
                        imageInfo->imageAddr);
        writeImg(dst_mat, "./nv12_interface_resize.jpg");
      }
    }
    usleep(10*1000);
  } while(loop--);
   
  usleep(1000*1000);
  return 0;
}
