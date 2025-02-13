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

#ifndef UTILS_H
#define UTILS_H

#include <memory>
#include <string>

#include "dnn/hb_dnn.h"
#include "opencv2/opencv.hpp"
#include "rclcpp/rclcpp.hpp"

#define ALIGNED_2E(w, alignment) \
  ((static_cast<uint32_t>(w) + (alignment - 1U)) & (~(alignment - 1U)))
#define ALIGN_4(w) ALIGNED_2E(w, 4U)
#define ALIGN_8(w) ALIGNED_2E(w, 8U)
#define ALIGN_16(w) ALIGNED_2E(w, 16U)
#define ALIGN_64(w) ALIGNED_2E(w, 64U)

int prepareBpuResizeParam(int src_w, int src_h, int dst_w, int dst_h);

void prepare_nv12_tensor_without_padding(const char *image_data,
                                         int image_height,
                                         int image_width,
                                         hbDNNTensor *tensor);

void prepare_nv12_tensor_without_padding(int image_height,
                                         int image_width,
                                         hbDNNTensor *tensor);

int32_t BGRToNv12(cv::Mat &bgr_mat, cv::Mat &img_nv12);

uint64_t currentMicroseconds();

namespace hobot_cv {
  class calculate_time {
   public:
    calculate_time(std::string name):name_(name) {
      start_ = std::chrono::system_clock::now();
    }
    ~calculate_time() {
      auto end = std::chrono::system_clock::now();
      auto interval = std::chrono::duration_cast<std::chrono::milliseconds>(end - start_).count();
      std::stringstream ss;
      ss << name_ << " time cost: " << interval << " ms";
      RCLCPP_INFO(rclcpp::get_logger("hobot_cv"), "%s", ss.str().c_str());
    }
   private:
    std::string name_;
    std::chrono::system_clock::time_point start_;
  };

}// namespace hobot_cv

#endif  // UTILS_H
