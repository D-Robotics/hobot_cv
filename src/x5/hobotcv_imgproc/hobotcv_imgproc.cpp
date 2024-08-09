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

#include <iostream>
#include <fstream>

#include "rclcpp/rclcpp.hpp"

#include "hobotcv_imgproc/hobotcv_imgproc.h"
#include "hobotcv_imgproc/hobotcv_front.h"
#include "utils.h"
#include "GC820/nano2D.h"
#include "GC820/nano2D_util.h"


namespace hobot_cv {

void save_yuv(std::string name, struct timespec stamp, int w, int h,
     void *data, int data_size) {
  std::string yuv_path = "./yuv/";
  uint64_t time_stamp = (stamp.tv_sec * 1000 + stamp.tv_nsec / 1000000);;
  if (access(yuv_path.c_str(), F_OK) == 0) {

    std::string yuv_file = "./yuv/" + name + std::to_string(time_stamp) + "_w" + std::to_string(w) + "_h" + std::to_string(h) + ".yuv";
    RCLCPP_INFO(rclcpp::get_logger("mipi_node"),
      "save yuv image: %s", yuv_file.c_str());
    std::ofstream out(yuv_file, std::ios::out|std::ios::binary);
    out.write(reinterpret_cast<char*>(data), data_size);
    out.close();
  }
}

int hobotcv_resize(const cv::Mat &src,
                   int src_h,
                   int src_w,
                   cv::Mat &dst,
                   int dst_h,
                   int dst_w) {
#if 1
  int ret;
  int64_t msStart = 0, msEnd = 0;
  {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    msStart = (ts.tv_sec * 1000 + ts.tv_nsec / 1000000);
  }

  hobotcv_front& hobotcv = hobotcv_front::getInstance();

  ret = hobotcv.prepareParam(src_w, src_h, dst_w, dst_h, cv::Range(0, 0), cv::Range(0, 0));
  if (ret != 0) {
    return -1;
  }
  
  dst = cv::Mat(dst_h * 3 / 2, dst_w, CV_8UC1);
  int dst_size = dst_w * dst_h * 1.5;
  ret = hobotcv.processFrame((const char*)src.data, src_w, src_h, (char*)dst.data, dst_size);
  if (ret != 0) {
    return -1;
  }

  //ret = hobotcv_vps_resize(
  //      src, dst, dst_h, dst_w, cv::Range(0, 0), cv::Range(0, 0));
  {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    msEnd = (ts.tv_sec * 1000 + ts.tv_nsec / 1000000);
  }
    RCLCPP_INFO(rclcpp::get_logger("hobot_cv"),
            "hobotcv_resize vps laps ms= %d", (msEnd - msStart));  
#else
  n2d_buffer_format_t format = N2D_NV12;
  float data_rate = 1.5;
  switch (format) {
    case N2D_NV12:
      data_rate = 1.5;
      break;
    case N2D_RGB888:
      data_rate = 3;
      break;
    default:
      return -1;
  }
  std::shared_ptr<n2d_buffer_t> src_ptr = std::make_shared<n2d_buffer_t>();
  std::shared_ptr<n2d_buffer_t> dst_ptr = std::make_shared<n2d_buffer_t>();
  do {
    struct timespec stamp;
    clock_gettime(CLOCK_MONOTONIC, &stamp);
    save_yuv("raw",stamp,src_w,src_h,src.data,src_w * src_h * 1.5);
    int64_t msStart = 0, msEnd = 0;
    {
      struct timespec ts;
      clock_gettime(CLOCK_MONOTONIC, &ts);
      msStart = (ts.tv_sec * 1000 + ts.tv_nsec / 1000000);
    }
    int error = n2d_open();
    if (N2D_IS_ERROR(error)) {
      printf("open context failed! error=%d.\n", error);
      //n2d_close();
      return -1;
    }
    /* switch to default device and core */
    error = n2d_switch_device(N2D_DEVICE_0);
    if (N2D_IS_ERROR(error)) {
      printf("switch device context failed! error=%d.\n", error);
      n2d_close();
      return -1;
    }
    error = n2d_switch_core(N2D_CORE_0);
    if (N2D_IS_ERROR(error)) {
      printf("switch core context failed! error=%d.\n", error);
      //n2d_close();
      return -1;
    }

    error = n2d_util_allocate_buffer(src_w, src_h, format, N2D_0, N2D_LINEAR, N2D_TSC_DISABLE, src_ptr.get());
    if (N2D_IS_ERROR(error))  {
      printf("n2d_util_allocate_buffer, src error=%d.\n", error);
      //n2d_close();
      return -1;
    }

    error = n2d_util_allocate_buffer(dst_w, dst_h, format, N2D_0, N2D_LINEAR, N2D_TSC_DISABLE, dst_ptr.get());
    if (N2D_IS_ERROR(error)) {
      printf("n2d_util_allocate_buffer, dst error=%d.\n", error);
      n2d_free(src_ptr.get());
      return -1;
    }
    
    memcpy(src_ptr->memory, src.data, src_w * src_h * data_rate);

    error = n2d_blit(dst_ptr.get(), N2D_NULL, src_ptr.get(), N2D_NULL, N2D_BLEND_NONE);
    if (N2D_IS_ERROR(error)) {
      printf("blit error, error=%d.\n", error);
      break;
    }
    error = n2d_commit();
    if (N2D_IS_ERROR(error)) {
        printf("blit error, error=%d.\n", error);
        break;
    }
    if (format == N2D_RGB888) {
      dst.create(dst_h, dst_w, CV_8UC3);
      memcpy(dst.data, dst_ptr->memory, dst_w * dst_h * 3);
    } else {
      dst.create(dst_h * 1.5, dst_w, CV_8UC1);
      memcpy(dst.data, dst_ptr->memory, dst_w * dst_h * 1.5);
    }
    {
      struct timespec ts;
      clock_gettime(CLOCK_MONOTONIC, &ts);
      msEnd = (ts.tv_sec * 1000 + ts.tv_nsec / 1000000);
    }
    save_yuv("n2d",stamp,dst_w,dst_h,dst.data,dst_w * dst_h * data_rate);
    RCLCPP_INFO(rclcpp::get_logger("hobot_cv"),
            "hobotcv_resize laps ms= %d", (msEnd - msStart));

  } while(0);
  n2d_free(src_ptr.get());
  n2d_free(dst_ptr.get());
  //n2d_close();
#endif
  return 0;

}

cv::Mat hobotcv_crop(const cv::Mat &src,
                     int src_h,
                     int src_w,
                     int dst_h,
                     int dst_w,
                     const cv::Range &rowRange,
                     const cv::Range &colRange,
                     HobotcvSpeedUpType type) {
  
#if 1
  cv::Mat dst_null;
  int ret;
  int64_t msStart = 0, msEnd = 0;
  {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    msStart = (ts.tv_sec * 1000 + ts.tv_nsec / 1000000);
  }

  hobotcv_front& hobotcv = hobotcv_front::getInstance();

  ret = hobotcv.prepareParam(src_w, src_h, dst_w, dst_h, rowRange, colRange);
  if (ret != 0) {
    return dst_null;
  }
  
  cv::Mat dst(dst_h * 3 / 2, dst_w, CV_8UC1);
  int dst_size = dst_w * dst_h * 1.5;
  ret = hobotcv.processFrame((const char*)src.data, src_w, src_h, (char*)dst.data, dst_size);
  if (ret != 0) {
    return dst_null;
  }

  {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    msEnd = (ts.tv_sec * 1000 + ts.tv_nsec / 1000000);
  }
    RCLCPP_INFO(rclcpp::get_logger("hobot_cv"),
            "hobotcv_resize vps laps ms= %d", (msEnd - msStart));  
#else
  cv::Mat dst;
  if (rowRange.end > src_h || colRange.end > src_w || rowRange.start < 0 || colRange.start < 0) {  // crop区域要在原图范围内
    RCLCPP_ERROR(
        rclcpp::get_logger("hobot_cv crop"),
        "Invalid Range data, rowRange.start:%d rowRange.end:%d "
        "colRange.start: %d colRange.end: %d"
        "rowRange should be in [0, %d) and colRange should be in [0, %d)",
        rowRange.start,
        rowRange.end,
        colRange.start,
        colRange.end,
        src_h,
        src_w);
    return dst;
  }
  n2d_buffer_format_t format = N2D_NV12;
  float data_rate = 1.5;
  switch (format) {
    case N2D_NV12:
      data_rate = 1.5;
      break;
    case N2D_RGB888:
      data_rate = 3;
      break;
    default:
      return dst;
  }
  std::shared_ptr<n2d_buffer_t> src_ptr = std::make_shared<n2d_buffer_t>();
  std::shared_ptr<n2d_buffer_t> dst_ptr = std::make_shared<n2d_buffer_t>();
  do {
    int64_t msStart = 0, msEnd = 0;
    {
      struct timespec ts;
      clock_gettime(CLOCK_MONOTONIC, &ts);
      msStart = (ts.tv_sec * 1000 + ts.tv_nsec / 1000000);
    }
    int error = n2d_open();
    if (N2D_IS_ERROR(error)) {
      printf("open context failed! error=%d.\n", error);
      //n2d_close();
      return dst;
    }
    /* switch to default device and core */
    error = n2d_switch_device(N2D_DEVICE_0);
    if (N2D_IS_ERROR(error)) {
      printf("switch device context failed! error=%d.\n", error);
      n2d_close();
      return dst;
    }
    error = n2d_switch_core(N2D_CORE_0);
    if (N2D_IS_ERROR(error)) {
      printf("switch core context failed! error=%d.\n", error);
      //n2d_close();
      return dst;
    }

    error = n2d_util_allocate_buffer(src_w, src_h, format, N2D_0, N2D_LINEAR, N2D_TSC_DISABLE, src_ptr.get());
    if (N2D_IS_ERROR(error))  {
      printf("n2d_util_allocate_buffer, src error=%d.\n", error);
      //n2d_close();
      return dst;
    }

    error = n2d_util_allocate_buffer(dst_w, dst_h, format, N2D_0, N2D_LINEAR, N2D_TSC_DISABLE, dst_ptr.get());
    if (N2D_IS_ERROR(error)) {
      printf("n2d_util_allocate_buffer, dst error=%d.\n", error);
      n2d_free(src_ptr.get());
      return dst;
    }
    
    memcpy(src_ptr->memory, src.data, src_w * src_h * data_rate);


    n2d_rectangle_t dst_src;
    dst_src.x = 0;
    dst_src.y = 0;
    dst_src.width  = dst_w;
    dst_src.height = dst_h;


    n2d_rectangle_t src_rect;
    src_rect.x = colRange.start;
    src_rect.y = rowRange.start;
    src_rect.width  = colRange.end - colRange.start;
    src_rect.height = rowRange.end - rowRange.start;

    error = n2d_blit(dst_ptr.get(), &dst_src, src_ptr.get(), &src_rect, N2D_BLEND_NONE);
    if (N2D_IS_ERROR(error)) {
      printf("blit error, error=%d.\n", error);
      break;
    }
    error = n2d_commit();
    if (N2D_IS_ERROR(error)) {
        printf("blit error, error=%d.\n", error);
        break;
    }
    if (format == N2D_RGB888) {
      dst.create(dst_h, dst_w, CV_8UC3);
      memcpy(dst.data, dst_ptr->memory, dst_w * dst_h * 3);
    } else {
      dst.create(dst_h * 1.5, dst_w, CV_8UC1);
      memcpy(dst.data, dst_ptr->memory, dst_w * dst_h * 1.5);
    }
    {
      struct timespec ts;
      clock_gettime(CLOCK_MONOTONIC, &ts);
      msEnd = (ts.tv_sec * 1000 + ts.tv_nsec / 1000000);
    }
    RCLCPP_INFO(rclcpp::get_logger("hobot_cv"),
            "hobotcv_resize laps ms= %d", (msEnd - msStart));

  } while(0);
  n2d_free(src_ptr.get());
  n2d_free(dst_ptr.get());
  //n2d_close();
#endif
  return dst;
}

int hobotcv_rotate(const cv::Mat &src, cv::Mat &dst, ROTATION_E rotation) {
  return -1;
}

int hobotcv_imgproc(const cv::Mat &src,
                    cv::Mat &dst,
                    int dst_h,
                    int dst_w,
                    ROTATION_E rotate,
                    const cv::Range &rowRange,
                    const cv::Range &colRange) {
  return -1;
}

HobotcvImagePtr hobotcv_BorderPadding(const char *src,
                                      const int &src_h,
                                      const int &src_w,
                                      const HobotcvPaddingType type,
                                      const PaddingArea &area,
                                      const uint8_t value) {
  if (!check_padding_area(area.top,
                          area.bottom,
                          area.left,
                          area.right,
                          src_h,
                          src_w,
                          (int)type)) {
    return nullptr;
  }
  if (type == HobotcvPaddingType::HOBOTCV_CONSTANT) {
    return hobotcv_constant_padding(
        src, src_h, src_w, area.top, area.bottom, area.left, area.right, value);
  } else if (type == HobotcvPaddingType::HOBOTCV_REPLICATE) {
    return hobotcv_replicate_padding(
        src, src_h, src_w, area.top, area.bottom, area.left, area.right);
  } else if (type == HobotcvPaddingType::HOBOTCV_REFLECT) {
    return hobotcv_reflect_padding(
        src, src_h, src_w, area.top, area.bottom, area.left, area.right);
  }
  return nullptr;
}

std::shared_ptr<ImageInfo> hobotcv_resize(const char *src,
                                          int src_h,
                                          int src_w,
                                          int dst_h,
                                          int dst_w) {
#if 1
  int ret;
  int64_t msStart = 0, msEnd = 0;
  {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    msStart = (ts.tv_sec * 1000 + ts.tv_nsec / 1000000);
  }

  hobotcv_front& hobotcv = hobotcv_front::getInstance();

  ret = hobotcv.prepareParam(src_w, src_h, dst_w, dst_h, cv::Range(0, 0), cv::Range(0, 0));
  if (ret != 0) {
    return nullptr;
  }
  
  //dst = cv::Mat(dst_h * 3 / 2, dst_w, CV_8UC1);

  int dst_size = dst_w * dst_h * 1.5;
  char* dst = (char *)malloc(dst_size);
  ret = hobotcv.processFrame(src, src_w, src_h, dst, dst_size);
  if (ret != 0) {
    return nullptr;
  }
  {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    msEnd = (ts.tv_sec * 1000 + ts.tv_nsec / 1000000);
  }
    RCLCPP_INFO(rclcpp::get_logger("hobot_cv"),
            "hobotcv_resize vps laps ms= %d", (msEnd - msStart));  
  auto imageInfo = new ImageInfo;
  imageInfo->width = dst_w;
  imageInfo->height = dst_h;
  imageInfo->imageAddr = dst;
  return std::shared_ptr<ImageInfo>(imageInfo,
                                    [dst](ImageInfo *imageInfo) {
                                      free(dst);
                                      delete imageInfo;
                                    });
#else
  n2d_buffer_format_t format = N2D_NV12;
  float data_rate = 1.5;
  switch (format) {
    case N2D_NV12:
      data_rate = 1.5;
      break;
    case N2D_RGB888:
      data_rate = 3;
      break;
    default:
      return nullptr;
  }
  std::shared_ptr<ImageInfo> image_ptr = nullptr;
  std::shared_ptr<n2d_buffer_t> src_ptr = std::make_shared<n2d_buffer_t>();
  std::shared_ptr<n2d_buffer_t> dst_ptr = std::make_shared<n2d_buffer_t>();
  do {
    int64_t msStart = 0, msEnd = 0;
    {
      struct timespec ts;
      clock_gettime(CLOCK_MONOTONIC, &ts);
      msStart = (ts.tv_sec * 1000 + ts.tv_nsec / 1000000);
    }
    int error = n2d_open();
    if (N2D_IS_ERROR(error)) {
      printf("open context failed! error=%d.\n", error);
      //n2d_close();
      return image_ptr;
    }
    /* switch to default device and core */
    error = n2d_switch_device(N2D_DEVICE_0);
    if (N2D_IS_ERROR(error)) {
      printf("switch device context failed! error=%d.\n", error);
      //n2d_close();
      return image_ptr;
    }
    error = n2d_switch_core(N2D_CORE_0);
    if (N2D_IS_ERROR(error)) {
      printf("switch core context failed! error=%d.\n", error);
      //n2d_close();
      return image_ptr;
    }

    error = n2d_util_allocate_buffer(src_w, src_h, format, N2D_0, N2D_LINEAR, N2D_TSC_DISABLE, src_ptr.get());
    if (N2D_IS_ERROR(error))  {
      printf("n2d_util_allocate_buffer, src error=%d.\n", error);
      //n2d_close();
      return image_ptr;
    }

    error = n2d_util_allocate_buffer(dst_w, dst_h, format, N2D_0, N2D_LINEAR, N2D_TSC_DISABLE, dst_ptr.get());
    if (N2D_IS_ERROR(error)) {
      printf("n2d_util_allocate_buffer, dst error=%d.\n", error);
      n2d_free(src_ptr.get());
      return image_ptr;
    }
    
    memcpy(src_ptr->memory, src, src_w * src_h * data_rate);

    error = n2d_blit(dst_ptr.get(), N2D_NULL, src_ptr.get(), N2D_NULL, N2D_BLEND_NONE);
    if (N2D_IS_ERROR(error)) {
      printf("blit error, error=%d.\n", error);
      break;
    }
    error = n2d_commit();
    if (N2D_IS_ERROR(error)) {
        printf("blit error, error=%d.\n", error);
        break;
    }
    {
      struct timespec ts;
      clock_gettime(CLOCK_MONOTONIC, &ts);
      msEnd = (ts.tv_sec * 1000 + ts.tv_nsec / 1000000);
    }
    RCLCPP_INFO(rclcpp::get_logger("hobot_cv"),
            "hobotcv_resize laps ms= %d", (msEnd - msStart));
    auto imageInfo = new ImageInfo;
    imageInfo->width = dst_w;
    imageInfo->height = dst_h;
    imageInfo->imageAddr = dst_ptr->memory;
    n2d_free(src_ptr.get());
    return std::shared_ptr<ImageInfo>(imageInfo,
                                      [dst_ptr](ImageInfo *imageInfo) {
                                        n2d_free(dst_ptr.get());
                                        delete imageInfo;
                                      });
  } while(0);
  n2d_free(src_ptr.get());
  n2d_free(dst_ptr.get());
  //n2d_close();
  return image_ptr;
#endif
}

std::shared_ptr<ImageInfo> hobotcv_crop(const char *src,
                                        int src_h,
                                        int src_w,
                                        int dst_h,
                                        int dst_w,
                                        const cv::Range &rowRange,
                                        const cv::Range &colRange,
                                        HobotcvSpeedUpType type) {
#if 1
  int ret;
  int64_t msStart = 0, msEnd = 0;
  {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    msStart = (ts.tv_sec * 1000 + ts.tv_nsec / 1000000);
  }

  hobotcv_front& hobotcv = hobotcv_front::getInstance();

  ret = hobotcv.prepareParam(src_w, src_h, dst_w, dst_h, rowRange, colRange);
  if (ret != 0) {
    return nullptr;
  }
  
  //dst = cv::Mat(dst_h * 3 / 2, dst_w, CV_8UC1);

  int dst_size = dst_w * dst_h * 1.5;
  char* dst = (char *)malloc(dst_size);
  ret = hobotcv.processFrame(src, src_w, src_h, dst, dst_size);
  if (ret != 0) {
    return nullptr;
  }
  {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    msEnd = (ts.tv_sec * 1000 + ts.tv_nsec / 1000000);
  }
    RCLCPP_INFO(rclcpp::get_logger("hobot_cv"),
            "hobotcv_resize vps laps ms= %d", (msEnd - msStart));  
  auto imageInfo = new ImageInfo;
  imageInfo->width = dst_w;
  imageInfo->height = dst_h;
  imageInfo->imageAddr = dst;
  return std::shared_ptr<ImageInfo>(imageInfo,
                                    [dst](ImageInfo *imageInfo) {
                                      free(dst);
                                      delete imageInfo;
                                    });
#else
  if (rowRange.end > src_h || colRange.end > src_w || rowRange.start < 0 || colRange.start < 0) {  // crop区域要在原图范围内
    RCLCPP_ERROR(
        rclcpp::get_logger("hobot_cv crop"),
        "Invalid Range data, rowRange.start:%d rowRange.end:%d "
        "colRange.start: %d colRange.end: %d"
        "rowRange should be in [0, %d) and colRange should be in [0, %d)",
        rowRange.start,
        rowRange.end,
        colRange.start,
        colRange.end,
        src_h,
        src_w);
    return nullptr;
  }
  n2d_buffer_format_t format = N2D_NV12;
  float data_rate = 1.5;
  switch (format) {
    case N2D_NV12:
      data_rate = 1.5;
      break;
    case N2D_RGB888:
      data_rate = 3;
      break;
    default:
      return nullptr;
  }
  std::shared_ptr<ImageInfo> image_ptr = nullptr;
  std::shared_ptr<n2d_buffer_t> src_ptr = std::make_shared<n2d_buffer_t>();
  std::shared_ptr<n2d_buffer_t> dst_ptr = std::make_shared<n2d_buffer_t>();
  do {
    int64_t msStart = 0, msEnd = 0;
    {
      struct timespec ts;
      clock_gettime(CLOCK_MONOTONIC, &ts);
      msStart = (ts.tv_sec * 1000 + ts.tv_nsec / 1000000);
    }
    int error = n2d_open();
    if (N2D_IS_ERROR(error)) {
      printf("open context failed! error=%d.\n", error);
      //n2d_close();
      return image_ptr;
    }
    /* switch to default device and core */
    error = n2d_switch_device(N2D_DEVICE_0);
    if (N2D_IS_ERROR(error)) {
      printf("switch device context failed! error=%d.\n", error);
      //n2d_close();
      return image_ptr;
    }
    error = n2d_switch_core(N2D_CORE_0);
    if (N2D_IS_ERROR(error)) {
      printf("switch core context failed! error=%d.\n", error);
      //n2d_close();
      return image_ptr;
    }

    error = n2d_util_allocate_buffer(src_w, src_h, format, N2D_0, N2D_LINEAR, N2D_TSC_DISABLE, src_ptr.get());
    if (N2D_IS_ERROR(error))  {
      printf("n2d_util_allocate_buffer, src error=%d.\n", error);
      //n2d_close();
      return image_ptr;
    }

    error = n2d_util_allocate_buffer(dst_w, dst_h, format, N2D_0, N2D_LINEAR, N2D_TSC_DISABLE, dst_ptr.get());
    if (N2D_IS_ERROR(error)) {
      printf("n2d_util_allocate_buffer, dst error=%d.\n", error);
      n2d_free(src_ptr.get());
      return image_ptr;
    }
    
    memcpy(src_ptr->memory, src, src_w * src_h * data_rate);

        n2d_rectangle_t dst_src;
    dst_src.x = 0;
    dst_src.y = 0;
    dst_src.width  = dst_w;
    dst_src.height = dst_h;


    n2d_rectangle_t src_rect;
    src_rect.x = colRange.start;
    src_rect.y = rowRange.start;
    src_rect.width  = colRange.end - colRange.start;
    src_rect.height = rowRange.end - rowRange.start;

    error = n2d_blit(dst_ptr.get(), &dst_src, src_ptr.get(), &src_rect, N2D_BLEND_NONE);
    if (N2D_IS_ERROR(error)) {
      printf("blit error, error=%d.\n", error);
      break;
    }
    error = n2d_commit();
    if (N2D_IS_ERROR(error)) {
        printf("blit error, error=%d.\n", error);
        break;
    }
    {
      struct timespec ts;
      clock_gettime(CLOCK_MONOTONIC, &ts);
      msEnd = (ts.tv_sec * 1000 + ts.tv_nsec / 1000000);
    }
    RCLCPP_INFO(rclcpp::get_logger("hobot_cv"),
            "hobotcv_resize laps ms= %d", (msEnd - msStart));
    auto imageInfo = new ImageInfo;
    imageInfo->width = dst_w;
    imageInfo->height = dst_h;
    imageInfo->imageAddr = dst_ptr->memory;
    n2d_free(src_ptr.get());
    return std::shared_ptr<ImageInfo>(imageInfo,
                                      [dst_ptr](ImageInfo *imageInfo) {
                                        n2d_free(dst_ptr.get());
                                        delete imageInfo;
                                      });
  } while(0);
  n2d_free(src_ptr.get());
  n2d_free(dst_ptr.get());
  //n2d_close();
  return image_ptr;
#endif
}

std::shared_ptr<ImageInfo> hobotcv_rotate(const char *src,
                                          int src_h,
                                          int src_w,
                                          ROTATION_E rotate) {
  std::shared_ptr<ImageInfo> image_ptr = nullptr;
  return image_ptr;
}

std::shared_ptr<ImageInfo> hobotcv_imgproc(const char *src,
                                           int src_h,
                                           int src_w,
                                           int dst_h,
                                           int dst_w,
                                           ROTATION_E rotate,
                                           const cv::Range &rowRange,
                                           const cv::Range &colRange) {
  std::shared_ptr<ImageInfo> image_ptr = nullptr;
  return image_ptr;
}


}  // namespace hobot_cv
