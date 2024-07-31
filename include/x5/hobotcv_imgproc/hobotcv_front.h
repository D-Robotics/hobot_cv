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
#ifndef HOBOTCV_FRONT_H
#define HOBOTCV_FRONT_H

#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <iostream>
#include <sys/timerfd.h>
#include <unistd.h>
#include <fcntl.h>
#include <cstring>
#include <system_error>
#include <functional>
#include <signal.h>

#include "opencv2/core/mat.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"
#include "hbn_api.h"
#include "vse_cfg.h"

namespace hobot_cv {

// hobotcv padding功能填充区域合规检查
bool check_padding_area(uint32_t top,
                        uint32_t bottom,
                        uint32_t left,
                        uint32_t right,
                        const int &src_h,
                        const int &src_w,
                        int padding_type);

/* hobotcv constant填充方式，用传入的value填充，
成功返回填充后图片数据，失败返回nullptr*/
std::unique_ptr<char[]> hobotcv_constant_padding(const char *src,
                                                 const int &src_h,
                                                 const int &src_w,
                                                 uint32_t top,
                                                 uint32_t bottom,
                                                 uint32_t left,
                                                 uint32_t right,
                                                 uint8_t value);

/* hobotcv replicate填充方式，复制最边界像素填充
成功返回填充后图片数据，失败返回nullptr*/
std::unique_ptr<char[]> hobotcv_replicate_padding(const char *src,
                                                  const int &src_h,
                                                  const int &src_w,
                                                  uint32_t top,
                                                  uint32_t bottom,
                                                  uint32_t left,
                                                  uint32_t right);

/* hobotcv reflect填充方式，已原图边界为轴镜像填充
成功返回填充后图片数据，失败返回nullptr*/
std::unique_ptr<char[]> hobotcv_reflect_padding(const char *src,
                                                const int &src_h,
                                                const int &src_w,
                                                uint32_t top,
                                                uint32_t bottom,
                                                uint32_t left,
                                                uint32_t right);

class hobotcv_front {
 public:
  // 获取单体实例的静态方法
  static hobotcv_front& getInstance() {
      static hobotcv_front instance;  // 静态局部变量，只初始化一次
      return instance;
  }
  // 禁止复制构造函数和赋值操作符
  hobotcv_front(const hobotcv_front&) = delete;
  hobotcv_front& operator=(const hobotcv_front&) = delete;


  int prepareParam(int src_height,
                     int src_width,
                     int dst_width,
                     int dst_height,
                     cv::Range rowRange,
                     cv::Range colRange,
                     bool printLog = true);

  int processFrame(const char *src, int input_w, int input_h, char *dst, int dst_size);

 private:
  explicit hobotcv_front() {}
  // 防止外部删除
  ~hobotcv_front() {}

  int creat_vse_node();
  int start_vse_node();
  int stop_vse_node();
  int destroy_vse_node();
  int creat_vflow_node();
  int set_vse_attr();

 public:
  int src_w;
  int src_h;
  int dst_w;
  int dst_h;
  int roi_x;
  int roi_y;
  int roi_w;
  int roi_h;


 private:
  int processId = 0;
  int ds_layer_en = 0;
  hbn_vnode_handle_t vse_node_handle;
  hbn_vflow_handle_t vflow_fd;
  uint32_t chn_id = 0;
  uint32_t ochn_id = 0;
  bool m_inited_ = false;
  bool start_ = 0;

};

}  // namespace hobot_cv

#endif  // HOBOTCV_FRONT_H
