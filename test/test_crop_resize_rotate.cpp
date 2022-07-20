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
  std::string image_file = "config/test.jpg";
  cv::Mat bgr_mat = cv::imread(image_file, cv::IMREAD_COLOR);
  auto src_height = bgr_mat.rows;
  auto src_width = bgr_mat.cols;

  cv::Mat srcmat_nv12;
  BGRToNv12(bgr_mat, srcmat_nv12);
  auto dst_height = src_height / 2;
  auto dst_width = src_width / 2;
  cv::Mat dstmat_nv12(dst_height * 3 / 2, dst_width, CV_8UC1);

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
  }
  writeImg(dstmat_nv12, "./resize.jpg");

  auto before_crop = std::chrono::system_clock::now();
  auto cropmat = hobot_cv::hobotcv_crop(srcmat_nv12,
                                        src_height,
                                        src_width,
                                        dst_height,
                                        dst_width,
                                        cv::Range(0, dst_height),
                                        cv::Range(0, dst_width));
  auto after_crop = std::chrono::system_clock::now();
  interval = std::chrono::duration_cast<std::chrono::milliseconds>(after_crop -
                                                                   before_crop)
                 .count();
  std::stringstream ss_crop;
  ss_crop << "crop image to " << dst_width << "x" << dst_height << " pixels"
          << ", time cost: " << interval << " ms";
  RCLCPP_INFO(rclcpp::get_logger("example"), "%s", ss_crop.str().c_str());
  writeImg(cropmat, "./crop.jpg");

  auto before_cropResize = std::chrono::system_clock::now();
  auto cropResizemat = hobot_cv::hobotcv_crop(srcmat_nv12,
                                              src_height,
                                              src_width,
                                              src_height,
                                              src_width,
                                              cv::Range(0, dst_height),
                                              cv::Range(0, dst_width));
  auto after_cropResize = std::chrono::system_clock::now();
  interval = std::chrono::duration_cast<std::chrono::milliseconds>(
                 after_cropResize - before_cropResize)
                 .count();
  std::stringstream ss_cropResize;
  ss_cropResize << "crop image to " << dst_width << "x" << dst_height
                << " pixels"
                << " and resize image to " << src_width << "x" << src_height
                << " pixels"
                << ", time cost: " << interval << " ms"
                << "\n";
  RCLCPP_INFO(rclcpp::get_logger("example"), "%s", ss_cropResize.str().c_str());
  writeImg(cropResizemat, "./cropResize.jpg");

  {  // rotate
    cv::Mat rotate_nv12;
    auto before_rotate = std::chrono::system_clock::now();
    auto ret = hobot_cv::hobotcv_rotate(
        srcmat_nv12, rotate_nv12, hobot_cv::ROTATION_180);
    auto after_rotate = std::chrono::system_clock::now();
    auto interval = std::chrono::duration_cast<std::chrono::milliseconds>(
                        after_rotate - before_rotate)
                        .count();
    if (ret == 0) {
      std::stringstream ss_rotate;
      ss_rotate << "rotate image 180 "
                << ", time cost: " << interval << " ms"
                << "\n";
      RCLCPP_INFO(rclcpp::get_logger("example"), "%s", ss_rotate.str().c_str());
    }
    writeImg(rotate_nv12, "./rotate.jpg");
  }

  {  // crop & resize & rotate
    cv::Mat cropResizeRotate_nv12;
    auto before_cropResizeRotate = std::chrono::system_clock::now();
    auto ret = hobot_cv::hobotcv_imgproc(srcmat_nv12,
                                         cropResizeRotate_nv12,
                                         800,
                                         1440,
                                         hobot_cv::ROTATION_90,
                                         cv::Range(540, 1080),
                                         cv::Range(960, 1920));
    auto after_cropResizeRotate = std::chrono::system_clock::now();
    auto interval = std::chrono::duration_cast<std::chrono::milliseconds>(
                        after_cropResizeRotate - before_cropResizeRotate)
                        .count();

    if (ret == 0) {
      std::stringstream ss_cropResizeRotate;
      ss_cropResizeRotate << "crop image to " << 960 << "x" << 540 << " pixels"
                          << " and resize image to " << 1440 << "x" << 800
                          << " pixels"
                          << " and rotate 90"
                          << ", time cost: " << interval << " ms"
                          << "\n";
      RCLCPP_INFO(rclcpp::get_logger("example"),
                  "%s",
                  ss_cropResizeRotate.str().c_str());
    }
    writeImg(cropResizeRotate_nv12, "./cropResizeRotate.jpg");
  }

  return 0;
}