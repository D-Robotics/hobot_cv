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

#include <chrono>
#include <functional>
#include <thread>
#include <atomic>

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


class Timer {
 public:
    Timer() : running(false) {}

    // 设置定时器
    void setTimer(std::chrono::milliseconds duration, std::function<void()> func) {
        callback_func_ = func;
        duration_ = duration;
    }

    // 启动定时器
    void start() {
        if (!running.exchange(true)) {
            thread_ = std::thread([this]() {
              while (this->running.load()) {
                std::this_thread::sleep_for(this->duration_);
                this->callback_func_();
              }
            });
        }
    }

    // 取消定时器
    void cancel() {
        if (running.exchange(false)) {
            if (thread_.joinable()) {
                thread_.detach(); // 如果线程在sleep中，这可能导致不确定的行为
            }
        }
    }

    ~Timer() {
        cancel();
    }

 private:
    std::atomic<bool> running;
    std::chrono::milliseconds duration_;
    std::thread thread_;
    std::function<void()> callback_func_;
};

class hobotcv_front {
 public:

  hobotcv_front() {run_time = std::chrono::system_clock::now();};
  ~hobotcv_front() {destroy_vse_node();}

  int prepareParam(int src_height,
                     int src_width,
                     int dst_width,
                     int dst_height,
                     cv::Range rowRange,
                     cv::Range colRange,
                     bool printLog = true);

  int processFrame(const char *src, int input_w, int input_h, char *dst, int dst_size);

  std::chrono::system_clock::time_point run_time;

 private:
  
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

class hobotcv_front_group {
 public:
  // 获取单体实例的静态方法
  static hobotcv_front_group& getInstance() {
      static hobotcv_front_group instance;  // 静态局部变量，只初始化一次
      return instance;
  }
  // 禁止复制构造函数和赋值操作符
  hobotcv_front_group(const hobotcv_front_group&) = delete;
  hobotcv_front_group& operator=(const hobotcv_front_group&) = delete;


  std::shared_ptr<hobotcv_front> getHobotcvFront(int src_height,
                     int src_width,
                     int dst_width,
                     int dst_height,
                     cv::Range rowRange,
                     cv::Range colRange,
                     bool printLog = true);

  void process_front_timeout();

 private:
  explicit hobotcv_front_group() {
    timer_.setTimer(std::chrono::milliseconds(1000),
              std::bind(&hobotcv_front_group::process_front_timeout,this));
    timer_.start();
  }
  // 防止外部删除
  ~hobotcv_front_group() {timer_.cancel();}


 private:
  std::vector<std::shared_ptr<hobotcv_front>> hobotcv_group;
  Timer timer_;
  std::mutex v_mtx_;

};

}  // namespace hobot_cv

#endif  // HOBOTCV_FRONT_H
