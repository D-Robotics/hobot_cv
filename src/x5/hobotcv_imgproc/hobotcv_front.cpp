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

#include <errno.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#include <chrono>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

#include "hobotcv_imgproc/hobotcv_front.h"
#include "hobotcv_imgproc/hobotcv_imgproc.h"
#include "rclcpp/rclcpp.hpp"
#include "utils.h"

namespace hobot_cv {

#define ERR_CON_EQ(ret, a) do {\
		if ((ret) != (a)) {\
			printf("%s(%d) failed, ret %d\n", __func__, __LINE__, (int32_t)(ret));\
			return (ret);\
		}\
	} while(0)\


#define ERR_CON_NE(ret, a) do {\
		if ((ret) == (a)) {\
			printf("%s(%d) failed, ret %ld\n", __func__, __LINE__, (ret));\
			return (ret);\
		}\
	} while(0)\


bool check_padding_area(uint32_t top,
                        uint32_t bottom,
                        uint32_t left,
                        uint32_t right,
                        const int &src_h,
                        const int &src_w,
                        int padding_type) {
  if (padding_type == (int)(HobotcvPaddingType::HOBOTCV_REFLECT)) {
    // HOBOTCV_REFLECT方式，padding尺寸如果超过原图尺寸会发生越界
    if ((int)top > src_h) {
      RCLCPP_ERROR(rclcpp::get_logger("hobot_cv padding"),
                   "Invalid top size: %d! src_h: %d , padding top must be less "
                   "than src height!",
                   top,
                   src_h);
      return false;
    } else if ((int)bottom > src_h) {
      RCLCPP_ERROR(
          rclcpp::get_logger("hobot_cv padding"),
          "Invalid bottom size: %d! src_h: %d , padding bottom must be less "
          "than src height!",
          bottom,
          src_h);
      return false;
    } else if ((int)left > src_w) {
      RCLCPP_ERROR(
          rclcpp::get_logger("hobot_cv padding"),
          "Invalid left size: %d! src_w: %d , padding left must be less "
          "than src width!",
          left,
          src_w);
      return false;
    } else if ((int)right > src_w) {
      RCLCPP_ERROR(
          rclcpp::get_logger("hobot_cv padding"),
          "Invalid right size: %d! src_w: %d , padding right must be less "
          "than src width!",
          right,
          src_w);
      return false;
    }
  }
  if (top == 0 && bottom == 0 && left == 0 && right == 0) {
    RCLCPP_ERROR(rclcpp::get_logger("hobot_cv padding"), "No padding area!");
    return false;
  } else if (top % 2 != 0) {
    RCLCPP_ERROR(rclcpp::get_logger("hobot_cv padding"),
                 "Invalid top size: %d! Padding size must be even",
                 top);
    return false;
  } else if (bottom % 2 != 0) {
    RCLCPP_ERROR(rclcpp::get_logger("hobot_cv padding"),
                 "Invalid bottom size: %d! Padding size must be even",
                 bottom);
    return false;
  } else if (left % 2 != 0) {
    RCLCPP_ERROR(rclcpp::get_logger("hobot_cv padding"),
                 "Invalid left size: %d! Padding size must be even",
                 left);
    return false;
  } else if (right % 2 != 0) {
    RCLCPP_ERROR(rclcpp::get_logger("hobot_cv padding"),
                 "Invalid right size: %d! Padding size must be even",
                 right);
    return false;
  }
  return true;
}

std::unique_ptr<char[]> hobotcv_constant_padding(const char *src,
                                                 const int &src_h,
                                                 const int &src_w,
                                                 uint32_t top,
                                                 uint32_t bottom,
                                                 uint32_t left,
                                                 uint32_t right,
                                                 uint8_t value) {
  // value转成yuv值
  uint8_t value_y = value;
  uint8_t value_u = 128;
  uint8_t value_v = 128;

  uint32_t dst_w = src_w + left + right;
  uint32_t dst_h = src_h + top + bottom;
  uint32_t dst_y_size = dst_w * dst_h;
  uint32_t dst_uv_size = dst_w * dst_h / 2;
  size_t dst_size = dst_y_size + dst_uv_size;
  std::unique_ptr<char[]> unique(new char[dst_size]);
  auto dst = unique.get();
  char *dst_y = dst;
  char *dst_uv = dst + dst_y_size;

  // padding y
  for (uint32_t h = 0; h < dst_h; ++h) {
    if (h < top || h >= src_h + top) {
      auto *raw = dst_y + h * dst_w;
      memset(raw, value_y, dst_w);
    } else {
      // padding left
      auto *raw = dst_y + h * dst_w;
      memset(raw, value_y, left);
      // copy src
      raw = dst_y + h * dst_w + left;
      auto *src_y_raw = src + (h - top) * src_w;
      memcpy(raw, src_y_raw, src_w);
      // padding right
      raw = dst_y + h * dst_w + left + src_w;
      memset(raw, value_y, right);
    }
  }

  // padding uv
  auto *src_uv_data = src + src_h * src_w;
  for (uint32_t h = 0; h < dst_h / 2; ++h) {
    auto *raw = dst_uv + h * dst_w;
    if ((h < (top / 2)) || (h >= (src_h + top) / 2)) {
      for (uint32_t w = 0; w < dst_w; w += 2) {
        *(raw + w) = value_u;
        *(raw + w + 1) = value_v;
      }
    } else {
      auto *raw = dst_uv + h * dst_w;
      for (uint32_t w = 0; w < left; w += 2) {
        *(raw + w) = value_u;
        *(raw + w + 1) = value_v;
      }
      raw = dst_uv + h * dst_w + left;
      auto *src_uv_raw = src_uv_data + (h - (top / 2)) * src_w;
      memcpy(raw, src_uv_raw, src_w);
      raw = dst_uv + h * dst_w + left + src_w;
      for (uint32_t w = 0; w < right; w += 2) {
        *(raw + w) = value_u;
        *(raw + w + 1) = value_v;
      }
    }
  }

  return unique;
}

std::unique_ptr<char[]> hobotcv_replicate_padding(const char *src,
                                                  const int &src_h,
                                                  const int &src_w,
                                                  uint32_t top,
                                                  uint32_t bottom,
                                                  uint32_t left,
                                                  uint32_t right) {
  uint32_t dst_w = src_w + left + right;
  uint32_t dst_h = src_h + top + bottom;
  int dst_y_size = dst_w * dst_h;
  size_t dst_size = dst_h * dst_w * 3 / 2;
  std::unique_ptr<char[]> unique(new char[dst_size]);
  auto dst = unique.get();
  char *dst_y = dst;
  char *dst_uv = dst + dst_y_size;
  auto *src_uv = src + src_h * src_w;
  uint32_t dst_bottom_start_line = src_h + top;
  // padding top and bottom
  for (uint32_t h = 0; h < dst_h; ++h) {  // padding y
    auto *raw = dst_y + h * dst_w + left;
    if (h < top) {  // padding top
      memcpy(raw, src, src_w);
    } else if (h >= dst_bottom_start_line) {  // padding bottom
      auto *src_y_raw = src + (src_h - 1) * src_w;
      memcpy(raw, src_y_raw, src_w);
    } else {  // copy src
      auto *src_y_raw = src + (h - top) * src_w;
      memcpy(raw, src_y_raw, src_w);
    }
  }

  for (uint32_t h = 0; h < dst_h / 2; ++h) {  // padding uv
    auto *raw = dst_uv + h * dst_w + left;
    if (h < (top / 2)) {  // top
      memcpy(raw, src_uv, src_w);
    } else if (h >= (dst_bottom_start_line / 2)) {  // bottom
      auto *src_uv_raw = src_uv + ((src_h / 2) - 1) * src_w;
      memcpy(raw, src_uv_raw, src_w);
    } else {  // copy src
      auto *src_uv_raw = src_uv + (h - (top / 2)) * src_w;
      memcpy(raw, src_uv_raw, src_w);
    }
  }

  // padding left and right
  for (uint32_t h = 0; h < dst_h; ++h) {
    // padding left
    auto *dst_left_y = dst_y + h * dst_w;
    auto *dst_left_uv = dst_uv + (h / 2) * dst_w;
    auto *src_left_y = dst_y + h * dst_w + left;
    auto *src_left_uv = dst_uv + (h / 2) * dst_w + left;
    uint16_t *src_left_uv_16 = (uint16_t *)(src_left_uv);
    memset(dst_left_y, *src_left_y, left);
    for (uint32_t w = 0; w < left; w += 2) {
      uint16_t *dst_16 = (uint16_t *)(dst_left_uv + w);
      *dst_16 = *src_left_uv_16;
    }
    // padding right
    auto *dst_right_y = dst_y + h * dst_w + left + src_w;
    auto *dst_right_uv = dst_uv + (h / 2) * dst_w + left + src_w;
    auto *src_right_y = dst_y + h * dst_w + left + src_w - 1;
    auto *src_right_uv = dst_uv + (h / 2) * dst_w + left + src_w - 2;

    uint16_t *src_right_uv_16 = (uint16_t *)(src_right_uv);
    memset(dst_right_y, *src_right_y, right);
    for (uint32_t w = 0; w < right; w += 2) {
      uint16_t *dst_16 = (uint16_t *)(dst_right_uv + w);
      *dst_16 = *src_right_uv_16;
    }
  }

  return unique;
}

std::unique_ptr<char[]> hobotcv_reflect_padding(const char *src,
                                                const int &src_h,
                                                const int &src_w,
                                                uint32_t top,
                                                uint32_t bottom,
                                                uint32_t left,
                                                uint32_t right) {
  uint32_t dst_w = src_w + left + right;
  uint32_t dst_h = src_h + top + bottom;
  int dst_y_size = dst_w * dst_h;
  size_t dst_size = dst_h * dst_w * 3 / 2;
  std::unique_ptr<char[]> unique(new char[dst_size]);
  auto dst = unique.get();
  char *dst_y = dst;
  char *dst_uv = dst + dst_y_size;
  auto *src_uv = src + src_h * src_w;
  uint32_t dst_bottom_start_line = src_h + top;
  // padding top and bottom
  int index_top = top, index_bottom = bottom;
  for (uint32_t h = 0; h < dst_h; ++h) {  // padding y
    auto *raw = dst_y + h * dst_w + left;
    if (h < top) {  // padding top
      auto *src_y_raw = src + index_top * src_w;
      memcpy(raw, src_y_raw, src_w);
      index_top--;
    } else if (h >= dst_bottom_start_line) {  // padding bottom
      auto *src_y_raw = src + (src_h - bottom + index_bottom - 1) * src_w;
      memcpy(raw, src_y_raw, src_w);
      index_bottom--;
    } else {  // copy src
      auto *src_y_raw = src + (h - top) * src_w;
      memcpy(raw, src_y_raw, src_w);
    }
  }

  index_top = top / 2;
  index_bottom = bottom / 2;
  for (uint32_t h = 0; h < dst_h / 2; ++h) {  // padding uv
    auto *raw = dst_uv + h * dst_w + left;
    if (h < (top / 2)) {  // top
      auto *src_uv_raw = src_uv + index_top * src_w;
      memcpy(raw, src_uv_raw, src_w);
      index_top--;
    } else if (h >= (dst_bottom_start_line / 2)) {  // bottom
      auto *src_uv_raw =
          src_uv + ((src_h / 2) - (bottom / 2) + index_bottom - 1) * src_w;
      memcpy(raw, src_uv_raw, src_w);
      index_bottom--;
    } else {  // copy src
      auto *src_uv_raw = src_uv + (h - (top / 2)) * src_w;
      memcpy(raw, src_uv_raw, src_w);
    }
  }

  // padding left and right
  for (uint32_t h = 0; h < dst_h; ++h) {
    // padding left
    auto *dst_left_y = dst_y + h * dst_w;
    auto *dst_left_uv = dst_uv + (h / 2) * dst_w;
    auto *src_left_y = dst_y + h * dst_w + left;
    auto *src_left_uv = dst_uv + (h / 2) * dst_w + left;
    for (uint32_t w = 0; w < left; w++) {
      *(dst_left_y + w) = *(src_left_y + left - w);
      if (w % 2 == 0) {
        uint16_t *dst_16 = (uint16_t *)(dst_left_uv + w);
        uint16_t *src_left_uv_16 = (uint16_t *)(src_left_uv + left - w);
        *dst_16 = *src_left_uv_16;
      }
    }
    // padding right
    auto *dst_right_y = dst_y + h * dst_w + left + src_w;
    auto *dst_right_uv = dst_uv + (h / 2) * dst_w + left + src_w;
    auto *src_right_y = dst_y + h * dst_w + left + src_w - 1;
    auto *src_right_uv = dst_uv + (h / 2) * dst_w + left + src_w - 2;
    for (uint32_t w = 0; w < right; w++) {
      *(dst_right_y + w) = *(src_right_y - w);
      if (w % 2 == 0) {
        uint16_t *dst_16 = (uint16_t *)(dst_right_uv + w);
        uint16_t *src_right_uv_16 = (uint16_t *)(src_right_uv - w);
        *dst_16 = *src_right_uv_16;
      }
    }
  }

  return unique;
}


/*当加速方式为AUTO时，先验证vps条件，此时printLog为false。
若不适用vps加速，不输出error log，直接采用bpu加速方式。*/
int hobotcv_front::prepareParam(int src_width,
                                      int src_height,
                                      int dst_width,
                                      int dst_height,
                                      cv::Range rowRange,
                                      cv::Range colRange,
                                      bool printLog) {
  int ret;
  if (dst_width % 16 != 0) {
    int remain = dst_width % 16;
    int recommend_dst_width = dst_width + 16 - remain;
    if (printLog) {
      /*当加速方式为AUTO时，先验证vps条件，此时printLog为false。
      若不适用vps加速，不输出error log，直接采用bpu加速方式。*/
      RCLCPP_ERROR(rclcpp::get_logger("hobot_cv resize"),
                   "unsupported dst width %d! The dst width must "
                   "be a multiple of 16! The recommended dst width is %d ",
                   dst_width,
                   recommend_dst_width);
    }
    return -1;
  }
  if (dst_height % 2 != 0) {
    if (printLog) {
      RCLCPP_ERROR(rclcpp::get_logger("hobot_cv resize"),
                   "unsupported dst height %d! The dst height must be even!",
                   dst_height);
    }
    return -1;
  }
  if (dst_height > 2160 || dst_width > 4096 || dst_height < 32 ||
      dst_width < 32) {
    if (printLog) {
      RCLCPP_ERROR(
          rclcpp::get_logger("hobot_cv resize"),
          "unsupported dst resolution %d x %d! The supported dst resolution "
          "is 32 x 32 to 4096 x 2160!",
          dst_width,
          dst_height);
    }
    return -1;
  }
  if ((rowRange.start == 0) && (rowRange.end == 0) && (colRange.start == 0) && (colRange.end == 0)) {
    rowRange.end = src_height;
    colRange.end = src_width;
  }
  if (colRange.end - colRange.start <= 0 || rowRange.end - rowRange.start <= 0 ||
      rowRange.start < 0 || colRange.start < 0 || rowRange.end > src_height ||
        colRange.end > src_width) {
    if (printLog) {
      RCLCPP_ERROR(
            rclcpp::get_logger("hobot_cv crop"),
            "Invalid Range data, rowRange.start:%d rowRange.end:%d "
            "colRange.start: %d colRange.end: %d"
            "rowRange should be in [0, %d) and colRange should be in [0, %d)",
            rowRange.start,
            rowRange.end,
            colRange.start,
            colRange.end,
            src_height,
            src_width);
    }
    return -1;
  }

  if ((src_h != src_height) || (src_w != src_width) || (dst_h != dst_height) ||
      (dst_w != dst_width) || (roi_x != colRange.start) || (roi_y != rowRange.start) ||
      (roi_w != (colRange.end - colRange.start)) || (roi_h != (rowRange.end - rowRange.start))) {

    this->src_h = src_height;
    this->src_w = src_width;
    this->dst_h = dst_height;
    this->dst_w = dst_width;
    this->roi_x = colRange.start;
    this->roi_y = rowRange.start;
    this->roi_w = colRange.end - colRange.start;
    this->roi_h = rowRange.end - rowRange.start;
    if (start_ == true) {
      ret = stop_vse_node();
      ERR_CON_EQ(ret, 0);
    }
    if (m_inited_ == true) {
      ret = destroy_vse_node();
      ERR_CON_EQ(ret, 0);
    }
    ret = creat_vflow_node();
    ERR_CON_EQ(ret, 0);
    ret = start_vse_node();
    ERR_CON_EQ(ret, 0);
  } else {
    if (m_inited_ == false) {
      ret = creat_vflow_node();
      ERR_CON_EQ(ret, 0);
    }
    if (start_ == false) {
      ret = start_vse_node();
      ERR_CON_EQ(ret, 0);
    }
  }
  return 0;
}


//int hobotcv_front::processFrame(const cv::Mat &src, cv::Mat &dst) {
int hobotcv_front::processFrame(const char *src, int input_w, int input_h, char *dst, int dst_size) {
  //int input_w = src.cols;
  //int input_h = src.rows * 2 / 3;
  hbn_vnode_image_t img;
  int64_t alloc_flags = 0;
  int ret;
  memset(&img, 0, sizeof(hbn_vnode_image_t));


  alloc_flags = HB_MEM_USAGE_MAP_INITIALIZED |
				HB_MEM_USAGE_PRIV_HEAP_2_RESERVERD |
				HB_MEM_USAGE_CPU_READ_OFTEN |
				HB_MEM_USAGE_CPU_WRITE_OFTEN |
				HB_MEM_USAGE_CACHED |
				HB_MEM_USAGE_GRAPHIC_CONTIGUOUS_BUF;
	ret = hb_mem_alloc_graph_buf(input_w,
								input_h,
								MEM_PIX_FMT_NV12,
								alloc_flags,
								input_w,
								input_h,
								&img.buffer);
  ERR_CON_EQ(ret, 0);

  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  img.info.timestamps = ts.tv_sec * 1e9 + ts.tv_nsec;

  memcpy(img.buffer.virt_addr[0], src, input_w * input_h);
  memcpy(img.buffer.virt_addr[1], src + input_w * input_h, input_w * input_h / 2);

  ret = hbn_vnode_sendframe(vse_node_handle, chn_id, &img);
  if (ret != 0) {
    printf("hbn_vnode_sendframe VSE channel  = %d,ret = %d failed\n", chn_id,ret);
    hb_mem_free_buf(img.buffer.fd[0]);
    return -1;
  }

	hbn_vnode_image_t out_img;
	ret = hbn_vnode_getframe(vse_node_handle, chn_id, 1000, &out_img);
	if (ret != 0) {
		printf("hbn_vnode_getframe VSE channel  = %d,ret = %d failed\n", chn_id,ret);
    hb_mem_free_buf(img.buffer.fd[0]);
		return -1;
	}
	//hb_mem_invalidate_buf_with_vaddr((uint64_t)out_img.buffer.virt_addr[0],out_img.buffer.size[0]);

	//hb_mem_invalidate_buf_with_vaddr((uint64_t)out_img.buffer.virt_addr[1],out_img.buffer.size[1]);
  int stride = out_img.buffer.stride;
  int width = out_img.buffer.width;
  int height = out_img.buffer.height;

  //dst = cv::Mat(height * 3 / 2, width, CV_8UC1);
  if (dst_size < (out_img.buffer.size[0] + out_img.buffer.size[1])) {
    hb_mem_free_buf(img.buffer.fd[0]);
    hbn_vnode_releaseframe(vse_node_handle, chn_id, &out_img);
    return -1;
  }
	memcpy(dst, out_img.buffer.virt_addr[0], out_img.buffer.size[0]);
  memcpy(dst + out_img.buffer.size[0], out_img.buffer.virt_addr[1], out_img.buffer.size[1]);
  hb_mem_free_buf(img.buffer.fd[0]);
	hbn_vnode_releaseframe(vse_node_handle, chn_id, &out_img);
	return 0;
}



int hobotcv_front::creat_vflow_node() {
  int hw_id = 0;
  int ret;
  if (m_inited_) 
    return -1;
  hb_mem_module_open();
  ret = creat_vse_node();
  ERR_CON_EQ(ret, 0);
  	// 创建HBN flow
	ret = hbn_vflow_create(&vflow_fd);
	ERR_CON_EQ(ret, 0);
	ret = hbn_vflow_add_vnode(vflow_fd, vse_node_handle);
	ERR_CON_EQ(ret, 0);
  m_inited_ = true;
  ret = set_vse_attr();
  ERR_CON_EQ(ret, 0);
  return 0;
}

int hobotcv_front::start_vse_node() {
  if ((m_inited_ == false) || (start_ == true)) {
    return -1;
  }
  int ret;
	ret = hbn_vflow_start(vflow_fd);
  ERR_CON_EQ(ret, 0);
  start_ = true;
  return 0;
}

int hobotcv_front::stop_vse_node() {
  int ret;
  if (start_) {
    ret = hbn_vflow_stop(vflow_fd);
    start_ = false;
    ERR_CON_EQ(ret, 0);
    return 0;
  } else {
    return -1;
  }

}

int hobotcv_front::destroy_vse_node() {
  int i = 0;
  if (start_) {
    stop_vse_node();
  }
  hbn_vflow_destroy(vflow_fd);
	hb_mem_module_close();
  m_inited_ = false;
  return 0;
}


int hobotcv_front::creat_vse_node() {
  if (m_inited_ == false) {
    int ret = 0;
    uint32_t hw_id = 0;
    ret = hbn_vnode_open(HB_VSE, hw_id, AUTO_ALLOC_ID, &vse_node_handle);
	  ERR_CON_EQ(ret, 0);
    return 0;
  } else {
    return -1;
  }
}

int hobotcv_front::set_vse_attr() {
  if ((m_inited_ == true) && (start_ == false)) {
    int ret = 0;
    uint32_t hw_id = 0;
    hbn_buf_alloc_attr_t alloc_attr = {0};
    vse_attr_t vse_attr = {0};
    vse_ichn_attr_t vse_ichn_attr;
    vse_ochn_attr_t vse_ochn_attr;
    ret = hbn_vnode_set_attr(vse_node_handle, &vse_attr);
    ERR_CON_EQ(ret, 0);

    ret = hbn_vnode_get_ichn_attr(vse_node_handle, chn_id, &vse_ichn_attr);
    ERR_CON_EQ(ret, 0);

    vse_ichn_attr.width = src_w;
    vse_ichn_attr.height = src_h;
    vse_ichn_attr.fmt = FRM_FMT_NV12;
    vse_ichn_attr.bit_width = 8;

    ret = hbn_vnode_set_ichn_attr(vse_node_handle, chn_id, &vse_ichn_attr);
    ERR_CON_EQ(ret, 0);

    vse_ochn_attr.chn_en = CAM_TRUE;
    vse_ochn_attr.roi.x = roi_x;
    vse_ochn_attr.roi.y = roi_y;
    vse_ochn_attr.roi.w = roi_w;
    vse_ochn_attr.roi.h = roi_h;
    vse_ochn_attr.fmt = FRM_FMT_NV12;
    vse_ochn_attr.bit_width = 8;
    vse_ochn_attr.target_w = dst_w;
    vse_ochn_attr.target_h = dst_h;

    ret = hbn_vnode_set_ochn_attr(vse_node_handle, 0, &vse_ochn_attr);
    ERR_CON_EQ(ret, 0);
    alloc_attr.buffers_num = 3;
    alloc_attr.is_contig = 1;
    alloc_attr.flags = HB_MEM_USAGE_CPU_READ_OFTEN
              | HB_MEM_USAGE_CPU_WRITE_OFTEN
              | HB_MEM_USAGE_CACHED
              | HB_MEM_USAGE_GRAPHIC_CONTIGUOUS_BUF;
    ret = hbn_vnode_set_ochn_buf_attr(vse_node_handle, 0, &alloc_attr);
    ERR_CON_EQ(ret, 0);
    return 0;
  } else {
    return -1;
  }
}

}  // namespace hobot_cv
