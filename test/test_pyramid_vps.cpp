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

  // 3.图像金字塔，测试耗时，输入图像为Mat格式
  {
    RCLCPP_INFO(rclcpp::get_logger("example"), "%s", "Test hobotcv_pymscale: Mat Input, VPS Process");
    hobot_cv::OutputPyramid *pymout = new hobot_cv::OutputPyramid;
    hobot_cv::PyramidAttr attr;
    memset(&attr, 0, sizeof(attr));
    attr.timeout = 2000;
    attr.ds_info[0].factor = 1;
    attr.ds_info[4].factor = 1;
    attr.ds_info[8].factor = 1;
    attr.ds_info[12].factor = 1;
    attr.ds_info[16].factor = 1;
    attr.ds_info[20].factor = 1;

    auto before_pyramid = std::chrono::system_clock::now();
    // =============================================
    auto ret = hobot_cv::hobotcv_pymscale(srcmat_nv12, pymout, attr);
    // =============================================
    auto after_pyramid = std::chrono::system_clock::now();
    auto interval = std::chrono::duration_cast<std::chrono::milliseconds>(after_pyramid - before_pyramid).count();
    std::stringstream ss;
    ss << "source image " << image_file << " is " << src_width << "x" << src_height << " pixels";
    RCLCPP_INFO(rclcpp::get_logger("example"), "%s", ss.str().c_str());
    if (ret == 0) {
      std::stringstream ss_pyramid;
      ss_pyramid << "pyramid image, mat input, time cost: " << interval << " ms"<< "\n";
      RCLCPP_INFO(rclcpp::get_logger("example"), "%s", ss_pyramid.str().c_str());

      for (int i = 0; i < 24; ++i) {
        int width = pymout->pym_out[i].width;
        int height = pymout->pym_out[i].height;
        if (width != 0 || height != 0) {
          cv::Mat dstmat(height * 3 / 2, width, CV_8UC1);
          memcpy(dstmat.data, &(pymout->pym_out[i].img[0]), width * height * 3 / 2);
          std::stringstream ss;
          ss << "./pyramid_" << i << "_mat_input_vps_process.jpg";
          writeImg(dstmat, ss.str().c_str());
        }
      }
    }
    delete pymout;
  }

  // 3.图像金字塔，测试耗时，输入图像为Arr格式
  {
    RCLCPP_INFO(rclcpp::get_logger("example"), "%s", "Test hobotcv_pymscale: Arr Input, VPS Process");
    hobot_cv::OutputPyramid *pymout = new hobot_cv::OutputPyramid;
    hobot_cv::PyramidAttr attr;
    memset(&attr, 0, sizeof(attr));
    attr.timeout = 2000;
    attr.ds_info[0].factor = 1;
    attr.ds_info[4].factor = 1;
    attr.ds_info[8].factor = 1;
    attr.ds_info[12].factor = 1;
    attr.ds_info[16].factor = 1;
    attr.ds_info[20].factor = 1;

    auto before_pyramid = std::chrono::system_clock::now();
    // =============================================
    auto ret = hobot_cv::hobotcv_pymscale(reinterpret_cast<const char *>(srcmat_nv12.data), src_height, src_width, pymout, attr);
    // =============================================
    auto after_pyramid = std::chrono::system_clock::now();
    auto interval = std::chrono::duration_cast<std::chrono::milliseconds>(after_pyramid - before_pyramid).count();
    std::stringstream ss;
    ss << "source image " << image_file << " is " << src_width << "x" << src_height << " pixels";
    RCLCPP_INFO(rclcpp::get_logger("example"), "%s", ss.str().c_str());
    if (ret == 0) {
      std::stringstream ss_pyramid;
      ss_pyramid << "pyramid image, mat input, time cost: " << interval << " ms"<< "\n";
      RCLCPP_INFO(rclcpp::get_logger("example"), "%s", ss_pyramid.str().c_str());

      for (int i = 0; i < 24; ++i) {
        int width = pymout->pym_out[i].width;
        int height = pymout->pym_out[i].height;
        if (width != 0 || height != 0) {
          cv::Mat dstmat(height * 3 / 2, width, CV_8UC1);
          memcpy(dstmat.data, &(pymout->pym_out[i].img[0]), width * height * 3 / 2);
          std::stringstream ss;
          ss << "./pyramid_" << i << "_mat_input_vps_process.jpg";
          writeImg(dstmat, ss.str().c_str());
        }
      }
    }
    delete pymout;
  }

  return 0;
}
