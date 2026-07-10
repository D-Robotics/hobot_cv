// Copyright (c) 2025，D-Robotics.
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

#ifndef IMAGE_PROCESSOR_NODE__H_
#define IMAGE_PROCESSOR_NODE__H_

#ifdef CV_BRIDGE_CPP
#include <cv_bridge/cv_bridge.hpp>
#else
#include <cv_bridge/cv_bridge.h>
#endif

#include <cstdint>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <string>

#include "hobotcv_imgproc/hobotcv_imgproc.h"

namespace hobot_cv {

class ImageOperation {
 public:
  ImageOperation() = default;
  ImageOperation(int dst_height, int dst_width);
  virtual bool apply(const cv::Mat &src_img,
                     int src_height,
                     int src_width,
                     cv::Mat &dst_img) = 0;
  virtual ~ImageOperation() = default;

  int get_dst_height() { return dst_height_; }
  int get_dst_width() { return dst_width_; }
  std::string get_dst_encoding() { return dst_encoding_; }

 protected:
  int dst_height_{0};
  int dst_width_{0};
  std::string dst_encoding_{"nv12"};
};

class ConvertOperation : public ImageOperation {
 public:
  explicit ConvertOperation(COLOR_E convert_type);
  bool apply(const cv::Mat &src_img,
             int src_height,
             int src_width,
             cv::Mat &dst_img) override;

 private:
  COLOR_E convert_type_{hobot_cv::COLOR_E::DCOLOR_MAX};
};

class ResizeOperation : public ImageOperation {
 public:
  ResizeOperation(int dst_height, int dst_width);
  bool apply(const cv::Mat &src_img,
             int src_height,
             int src_width,
             cv::Mat &dst_img) override;
};

class CropResizeOperation : public ImageOperation {
 public:
  CropResizeOperation(const cv::Range &row_range,
                      const cv::Range &col_range,
                      int dst_height,
                      int dst_width);
  bool apply(const cv::Mat &src_img,
             int src_height,
             int src_width,
             cv::Mat &dst_img) override;

 private:
  cv::Range row_range_;
  cv::Range col_range_;
};

class ImageProcessorNode : public rclcpp::Node {
 public:
  explicit ImageProcessorNode(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions(),
      const std::string &node_name = "ImageProcessorNode");

  ~ImageProcessorNode();

 private:
  COLOR_E get_color_conver_code(const std::string &name);

  void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr &msg);

 private:
  // Support chained operations
  std::vector<std::shared_ptr<ImageOperation>> operations_;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;

  // dump params
  bool need_dump_{false};
  std::string dump_pre_name_{""};
  uint64_t dump_index_{0u};
  std::string dump_dir_{"."};
};
}  // namespace hobot_cv

#endif  // IMAGE_PROCESSOR_NODE__H_
