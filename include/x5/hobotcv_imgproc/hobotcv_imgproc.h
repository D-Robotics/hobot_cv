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

#ifndef HOBOT_CV_INCLUDE_HOBOTCV_IMGPROC_HPP_
#define HOBOT_CV_INCLUDE_HOBOTCV_IMGPROC_HPP_

#include <memory>
#include <vector>

#include "opencv2/core/mat.hpp"
#include "opencv2/core/types.hpp"
#include "hbn_api.h"
#include "vse_cfg.h"

namespace hobot_cv {

typedef enum HOBOT_CV_ROTATION_E {
  ROTATION_0 = 0,
  ROTATION_90,
  ROTATION_180,
  ROTATION_270,
  ROTATION_MAX
} ROTATION_E;

typedef struct HOBOT_CV_IMAGE_INFO {
  int width;
  int height;
  void *imageAddr;
} ImageInfo;

typedef std::unique_ptr<char[]> HobotcvImagePtr;

// hobot_cv加速方式枚举，X5芯片的DNN库接口对比X3芯片有删除，目前hobot_cv里imgproc提供的接口均无法BPU加速，故删除了HOBOTCV_BPU枚举
enum HobotcvSpeedUpType { HOBOTCV_VPS = 1, HOBOTCV_CPU = 2 };

// 填充方式枚举
/*
HOBOTCV_CONSTANT为使用接口传入的value值进行填充
HOBOTCV_REPLICATE为使用原图中最边界的像素值进行填充(例如：aaaaaa|abcdefgh|hhhhhhh)
HOBOTCV_REFLECT为以原图边界为轴的镜像填充(例如：fedcba|abcdefgh|hgfedcb)
*/
enum class HobotcvPaddingType {
  HOBOTCV_CONSTANT = 0,
  HOBOTCV_REPLICATE,
  HOBOTCV_REFLECT
};

// padding区域
typedef struct Hobotcv_Padding_Area {
  uint32_t top;
  uint32_t bottom;
  uint32_t left;
  uint32_t right;
} PaddingArea;

/**
 * hobotcv加速图片resize处理
 * @param[in] src: 需要进行resize的原图，只支持nv12格式图片
 * @param[in] src_h: 原图高
 * @param[in] src_w: 原图宽
 * @param[out] dst：resize后输出的图像
 * @param[in] dst_h: resize后图片高
 * @param[in] dst_w: resize后图片宽
 * @param[in] type：加速方式，默认采用vps加速
 * @return 成功返回0，失败返回非0
 */
int hobotcv_resize(const cv::Mat &src,
                   int src_h,
                   int src_w,
                   cv::Mat &dst,
                   int dst_h,
                   int dst_w);

/**
 * hobotcv加速图片crop&resize处理
 * @param[in] src: 需要进行crop&resize的原图，只支持nv12格式图片
 * @param[in] src_h: 原图高
 * @param[in] src_w: 原图宽
 * @param[in] dst_h: resize后图片高
 * @param[in] dst_w: resize后图片宽
 * @param[in] rowRange：crop区域的纵向坐标范围，范围要在原图内
 * @param[in] colRange：crop区域的横向坐标范围，范围要在原图内
 * @param[in] type：加速方式，默认采用vps加速
 * @return 返回crop&resize后图片矩阵，失败返回空图片矩阵
 */
cv::Mat hobotcv_crop(const cv::Mat &src,
                     int src_h,
                     int src_w,
                     int dst_h,
                     int dst_w,
                     const cv::Range &rowRange,
                     const cv::Range &colRange,
                     HobotcvSpeedUpType type = HOBOTCV_VPS);

/**
 * hobotcv加速图片旋转处理
 * @param[in] src: 需要进行旋转的原图，只支持nv12格式图片
 * @param[out] dst：旋转后输出的图像
 * @param[in] rotate：旋转角度，支持90，180，270
 * @return 成功返回0，失败返回非0
 */
int hobotcv_rotate(const cv::Mat &src, cv::Mat &dst, ROTATION_E rotate);

/**
 * hobotcv加速图片crop&resize&rotate处理
 * @param[in] src: 需要进行crop&resize&rotate的原图，只支持nv12格式图片
 * @param[out] dst：crop&resize&rotate后输出的图像
 * @param[in] dst_h: crop&resize后图片高
 * @param[in] dst_w: crop&resize后图片宽
 * @param[in] rowRange：crop区域的纵向坐标范围，范围要在原图内
 * @param[in] colRange：crop区域的横向坐标范围，范围要在原图内
 * @return 成功返回0，失败返回非0
 */
int hobotcv_imgproc(const cv::Mat &src,
                    cv::Mat &dst,
                    int dst_h,
                    int dst_w,
                    ROTATION_E rotate,
                    const cv::Range &rowRange,
                    const cv::Range &colRange);


/**
 * hobotcv边界填充处理
 * @param[in] src: 需要填充边界的原图，只支持nv12格式图片
 * @param[in] src_h: 原图高
 * @param[in] src_w: 原图宽
 * @param[in] type：填充方式，支持指定值填充和复制原图边界两种方式
 * @param[in] area：上下左右填充区域
 * @param[in] value：填充的像素值，
 *            当填充方式为HOBOTCV_CONSTANT时，value值有效，取值范围0~255.默认值为0
 * @return 成功返回填充后的图片数据指针，失败返回nullptr
 */
HobotcvImagePtr hobotcv_BorderPadding(const char *src,
                                      const int &src_h,
                                      const int &src_w,
                                      const HobotcvPaddingType type,
                                      const PaddingArea &area,
                                      const uint8_t value = 0);

/**
 * hobotcv加速图片resize处理
 * @param[in] src: 需要进行resize的原图数据地址，只支持nv12格式图片
 * @param[in] src_h: 原图高
 * @param[in] src_w: 原图宽
 * @param[in] dst_h: resize后图片高
 * @param[in] dst_w: resize后图片宽
 * @param[in] type：加速方式，默认采用vps加速
 * @return 成功返回resize后的图片数据指针，失败返回nullptr
 */
std::shared_ptr<ImageInfo> hobotcv_resize(
    const char *src,
    int src_h,
    int src_w,
    int dst_h,
    int dst_w);

/**
 * hobotcv加速图片crop&resize处理
 * @param[in] src: 需要进行crop&resize的原图，只支持nv12格式图片
 * @param[in] src_h: 原图高
 * @param[in] src_w: 原图宽
 * @param[in] dst_h: resize后图片高
 * @param[in] dst_w: resize后图片宽
 * @param[in] rowRange：crop区域的纵向坐标范围，范围要在原图内
 * @param[in] colRange：crop区域的横向坐标范围，范围要在原图内
 * @param[in] type：加速方式，默认采用vps加速
 * @return 返回crop&resize后图片数据指针，失败返回nullptr
 */
std::shared_ptr<ImageInfo> hobotcv_crop(const char *src,
                                        int src_h,
                                        int src_w,
                                        int dst_h,
                                        int dst_w,
                                        const cv::Range &rowRange,
                                        const cv::Range &colRange,
                                        HobotcvSpeedUpType type = HOBOTCV_VPS);

/**
 * hobotcv加速图片旋转处理
 * @param[in] src: 需要进行旋转的原图，只支持nv12格式图片
 * @param[in] src_h: 原图高
 * @param[in] src_w: 原图宽
 * @param[in] rotate：旋转角度，支持90，180，270
 * @return 成功返回rotate后图片数据指针，失败返回nullptr
 */
std::shared_ptr<ImageInfo> hobotcv_rotate(const char *src,
                                          int src_h,
                                          int src_w,
                                          ROTATION_E rotate);

/**
 * hobotcv加速图片crop&resize&rotate处理
 * @param[in] src: 需要进行crop&resize&rotate的原图，只支持nv12格式图片
 * @param[in] src_h: 原图高
 * @param[in] src_w: 原图宽
 * @param[in] dst_h: crop&resize后图片高
 * @param[in] dst_w: crop&resize后图片宽
 * @param[in] rowRange：crop区域的纵向坐标范围，范围要在原图内
 * @param[in] colRange：crop区域的横向坐标范围，范围要在原图内
 * @return 成功返回处理后的图片数据指针，失败返回nullptr
 */
std::shared_ptr<ImageInfo> hobotcv_imgproc(const char *src,
                                           int src_h,
                                           int src_w,
                                           int dst_h,
                                           int dst_w,
                                           ROTATION_E rotate,
                                           const cv::Range &rowRange,
                                           const cv::Range &colRange);


}  // namespace hobot_cv

#endif  // HOBOT_CV_INCLUDE_HOBOTCV_IMGPROC_HPP_
