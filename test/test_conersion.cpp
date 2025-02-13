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
  auto nh = std::make_shared<hobot_cv::calculate_time>("nv12_to_bgr24 opencv");
  cv::cvtColor(mat, img_bgr, cv::COLOR_YUV2BGR_NV12);
  nh.reset();
  cv::imwrite(imgfile, img_bgr);
}

int main() {
  cv::setNumThreads(1);
  std::string image_file = "config/test.jpg";
  cv::Mat bgr_mat = cv::imread(image_file, cv::IMREAD_COLOR);
  auto src_height = bgr_mat.rows;
  auto src_width = bgr_mat.cols;
  auto nh = std::make_shared<hobot_cv::calculate_time>("bgr24_to_nv12 opencv");
  cv::Mat srcmat_nv12;
  BGRToNv12(bgr_mat, srcmat_nv12);
  nh.reset();
  int loop = 0;
  do {
    {  // resize first
      auto nh_1 = std::make_shared<hobot_cv::calculate_time>("nv12_to_bgr24 neon 1");
      cv::Mat dest_bgr;
      //auto ret = hobot_cv::hobotcv_Nv12ToBgr24(srcmat_nv12, dest_bgr);
      auto ret = hobot_cv::hobotcv_color(srcmat_nv12, dest_bgr, hobot_cv::DCOLOR_YUV2BGR_NV12);
      nh_1.reset();
      if (0 == ret) {
        cv::imwrite("./conersion_nv12_to_bgr_1.jpg", dest_bgr);
      }
    }

    {  // nv12 interface 
      auto nh_1 = std::make_shared<hobot_cv::calculate_time>("nv12_to_bgr24 neon 2");
      auto imageInfo = hobot_cv::hobotcv_color(
          reinterpret_cast<const char *>(srcmat_nv12.data),
          src_height,
          src_width,
          hobot_cv::DCOLOR_YUV2BGR_NV12);
      nh_1.reset();
      if (imageInfo != nullptr) {
        cv::Mat dst_mat(imageInfo->height,
                        imageInfo->width,
                        CV_8UC3,
                        imageInfo->imageAddr);
        cv::imwrite("./conersion_nv12_to_bgr_2.jpg", dst_mat);
      }
    }

    {  
      cv::Mat dest_nv12;
      auto nh_1 = std::make_shared<hobot_cv::calculate_time>("bgr24_to_nv12 neon 1");
      auto ret = hobot_cv::hobotcv_color(bgr_mat, dest_nv12, hobot_cv::DCOLOR_BGR2YUV_NV12);
      nh_1.reset();
      if (0 == ret) {
        writeImg(dest_nv12, "./conersion_bgr_to_nv12_1.jpg");
      }
    }

    {  // nv12 interface 
      auto nh_1 = std::make_shared<hobot_cv::calculate_time>("bgr24_to_nv12 neon 2");
      auto imageInfo = hobot_cv::hobotcv_color(
          reinterpret_cast<const char *>(bgr_mat.data),
          src_height,
          src_width,
          hobot_cv::DCOLOR_BGR2YUV_NV12);
      nh_1.reset();
      if (imageInfo != nullptr) {
        cv::Mat dst_mat(imageInfo->height * 3 / 2,
                        imageInfo->width,
                        CV_8UC1,
                        imageInfo->imageAddr);
        writeImg(dst_mat, "./conersion_bgr_to_nv12_2.jpg");
      }
    }
    usleep(10*1000);
  } while(loop--);
  return 0;
}
