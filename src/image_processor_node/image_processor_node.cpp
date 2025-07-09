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

#include "image_processor_node/image_processor_node.h"

#include "hobotcv_imgproc/hobotcv_imgproc.h"

namespace hobot_cv {

ImageOperation::ImageOperation(int dst_height, int dst_width)
    : dst_height_(dst_height), dst_width_(dst_width) {}

ConvertOperation::ConvertOperation(COLOR_E convert_type)
    : convert_type_(convert_type) {
  if (convert_type_ == COLOR_E::DCOLOR_YUV2BGR_NV12) {
    dst_encoding_ = "rgb8";
  }
}

bool ConvertOperation::apply(const cv::Mat &src_img,
                             int src_height,
                             int src_width,
                             cv::Mat &dst_img) {
  (void)src_height;
  (void)src_width;
  if (convert_type_ != COLOR_E::DCOLOR_MAX) {
    auto ret = hobot_cv::hobotcv_color(src_img, dst_img, convert_type_);
    if (ret == 0) {
      dst_height_ = src_height;
      dst_width_ = src_width;
      return true;
    } else {
      RCLCPP_ERROR(rclcpp::get_logger("ConvertOperation"),
                   "ConvertOperation failed, error code: %d",
                   ret);
    }
  }
  RCLCPP_ERROR(rclcpp::get_logger("ConvertOperation"),
               "ConvertOperation failed, convert_type_ is DCOLOR_MAX");
  return false;
}

ResizeOperation::ResizeOperation(int dst_height, int dst_width)
    : ImageOperation(dst_height, dst_width) {}

bool ResizeOperation::apply(const cv::Mat &src_img,
                            int src_height,
                            int src_width,
                            cv::Mat &dst_img) {
  if (src_height <= 0 || src_width <= 0 || dst_height_ <= 0 ||
      dst_width_ <= 0) {
    RCLCPP_ERROR(rclcpp::get_logger("ResizeOperation"),
                 "ResizeOperation failed, src_height: %d, src_width: %d, "
                 "dst_height_: %d, dst_width_: %d",
                 src_height,
                 src_width,
                 dst_height_,
                 dst_width_);
    return false;
  }
  auto ret = hobot_cv::hobotcv_resize(
      src_img, src_height, src_width, dst_img, dst_height_, dst_width_);
  if (ret == 0) {
    return true;
  }
  RCLCPP_ERROR(rclcpp::get_logger("ResizeOperation"),
               "ResizeOperation failed, error code: %d",
               ret);
  return false;
}

CropResizeOperation::CropResizeOperation(const cv::Range &row_range,
                                         const cv::Range &col_range,
                                         int dst_height = 0,
                                         int dst_width = 0)
    : ImageOperation(dst_height, dst_width),
      row_range_(row_range),
      col_range_(col_range) {}

bool CropResizeOperation::apply(const cv::Mat &src_img,
                                int src_height,
                                int src_width,
                                cv::Mat &dst_img) {
  // check src img param
  if (src_height <= 0 || src_width <= 0) {
    RCLCPP_ERROR(rclcpp::get_logger("CropResizeOperation"),
                 "CropResizeOperation failed, src_height: %d, src_width: %d",
                 src_height,
                 src_width);
    return false;
  }
  // check crop param
  if (row_range_.start >= src_height || row_range_.start < 0 ||
      row_range_.end > src_height || row_range_.end <= 0 ||
      col_range_.start >= src_width || col_range_.start < 0 ||
      col_range_.end > src_width || col_range_.end <= 0 ||
      row_range_.end - row_range_.start <= 0 ||
      col_range_.end - col_range_.start <= 0) {
    RCLCPP_ERROR(rclcpp::get_logger("CropResizeOperation"),
                 "CropResizeOperation failed, src_height: %d, src_width: %d, "
                 "row_range.start: %d, row_range.end: %d, col_range.start: %d, "
                 "col_range.end: %d",
                 src_height,
                 src_width,
                 row_range_.start,
                 row_range_.end,
                 col_range_.start,
                 col_range_.end);
    return false;
  }
  // check resize param
  if (dst_width_ == 0) {
    dst_width_ = col_range_.end - col_range_.start;
  }
  if (dst_height_ == 0) {
    dst_height_ = row_range_.end - row_range_.start;
  }
  dst_img = hobot_cv::hobotcv_crop(src_img,
                                   src_height,
                                   src_width,
                                   dst_height_,
                                   dst_width_,
                                   row_range_,
                                   col_range_);
  if (dst_img.empty()) {
    return false;
  } else {
    return true;
  }
}

ImageProcessorNode::ImageProcessorNode(const rclcpp::NodeOptions &options,
                                       const std::string &node_name)
    : Node(node_name, options) {
  declare_parameter<bool>("need_dump", need_dump_);
  declare_parameter<std::string>("dump_pre_name", dump_pre_name_);
  declare_parameter<std::string>("dump_dir", dump_dir_);
  declare_parameter<std::vector<std::string>>("pipeline",
                                              std::vector<std::string>{});

  get_parameter("need_dump", need_dump_);
  RCLCPP_INFO(
      this->get_logger(), "need_dump: %s", need_dump_ ? "true" : "false");
  get_parameter("dump_pre_name", dump_pre_name_);
  RCLCPP_INFO(this->get_logger(), "dump_pre_name: %s", dump_pre_name_.c_str());
  get_parameter("dump_dir", dump_dir_);
  RCLCPP_INFO(this->get_logger(), "dump_dir: %s", dump_dir_.c_str());
  auto pipeline = get_parameter("pipeline").as_string_array();
  for (const auto &op : pipeline) {
    if (op == "convert") {
      declare_parameter<std::string>("convert.code", "DCOLOR_YUV2BGR_NV12");
      std::string code = get_parameter("convert.code").as_string();
      auto cv_code = get_color_conver_code(code);
      operations_.emplace_back(std::make_shared<ConvertOperation>(cv_code));
    } else if (op == "crop_resize") {
      declare_parameter<int>("crop.x", 0);
      declare_parameter<int>("crop.y", 0);
      declare_parameter<int>("crop.width", 0);
      declare_parameter<int>("crop.height", 0);
      declare_parameter<int>("crop.resize_dst_height", 0);
      declare_parameter<int>("crop.resize_dst_width", 0);
      auto crop_x = get_parameter("crop.x").as_int();
      auto crop_y = get_parameter("crop.y").as_int();
      auto crop_height = get_parameter("crop.height").as_int();
      auto crop_width = get_parameter("crop.width").as_int();
      auto crop_resize_dst_height =
          get_parameter("crop.resize_dst_height").as_int();
      auto crop_resize_dst_width =
          get_parameter("crop.resize_dst_width").as_int();
      operations_.emplace_back(std::make_shared<CropResizeOperation>(
          cv::Range(crop_y, crop_y + crop_height),
          cv::Range(crop_x, crop_x + crop_width),
          crop_resize_dst_height,
          crop_resize_dst_width));
    } else if (op == "resize") {
      declare_parameter<int>("resize.dst_height", 0);
      declare_parameter<int>("resize.dst_width", 0);
      operations_.emplace_back(std::make_shared<ResizeOperation>(
          get_parameter("resize.dst_height").as_int(),
          get_parameter("resize.dst_width").as_int()));
    } else {
      RCLCPP_WARN(get_logger(), "Unknown operation: %s", op.c_str());
    }
  }

  sub_ = create_subscription<sensor_msgs::msg::Image>(
      "/image_in",
      10,
      std::bind(
          &ImageProcessorNode::image_callback, this, std::placeholders::_1));

  pub_ = create_publisher<sensor_msgs::msg::Image>("/image_out", 10);
}

COLOR_E ImageProcessorNode::get_color_conver_code(const std::string &name) {
  static const std::map<std::string, COLOR_E> map = {
      {"DCOLOR_YUV2BGR_NV12", COLOR_E::DCOLOR_YUV2BGR_NV12},
      {"DCOLOR_BGR2YUV_NV12", COLOR_E::DCOLOR_BGR2YUV_NV12},
  };
  auto it = map.find(name);
  return it != map.end() ? it->second : COLOR_E::DCOLOR_MAX;
}

void ImageProcessorNode::image_callback(
    const sensor_msgs::msg::Image::ConstSharedPtr &msg) {
  int src_height = msg->height;
  int src_width = msg->width;
  std::string encoding = msg->encoding;
  cv::Mat img;
  if (encoding == "nv12") {
    img = cv::Mat(src_height + src_height / 2,
                  src_width,
                  CV_8UC1,
                  const_cast<uint8_t *>(msg->data.data()));
  } else {
    cv_bridge::CvImagePtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
    } catch (cv_bridge::Exception &e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }
    img = cv_ptr->image;
  }

  for (auto &op : operations_) {
    cv::Mat dst;
    op->apply(img, src_height, src_width, dst);
    src_height = op->get_dst_height();
    src_width = op->get_dst_width();
    encoding = op->get_dst_encoding();
    img = dst;
  }
  if (need_dump_) {
    std::string file_name = dump_pre_name_ + std::to_string(dump_index_++) + ".jpg";
    if (encoding == "rgb8") {
      cv::imwrite(dump_dir_ + "/" + file_name, img);
    } else if (encoding == "nv12") {
      cv::Mat rgb_image;
      cv::cvtColor(img, rgb_image, cv::COLOR_YUV2RGB_NV12);
      cv::imwrite(dump_dir_ + "/" + file_name, rgb_image);
    }
  }

  sensor_msgs::msg::Image::SharedPtr out_msg =
      cv_bridge::CvImage(msg->header, encoding, img).toImageMsg();
  pub_->publish(*out_msg);
}
}  // namespace hobot_cv

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(hobot_cv::ImageProcessorNode)
