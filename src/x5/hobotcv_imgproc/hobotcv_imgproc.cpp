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
#include <arm_neon.h>

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

static void nv12_to_bgr24_neon(uint8_t* nv12, uint8_t* bgr24, int width, int height) {
  const uint8_t* yptr = nv12;
  const uint8_t* uvptr = nv12 + width * height;
  uint8x8_t _v128 = vdup_n_u8(128);
  int8x8_t _v127= vdup_n_s8(127);
  uint8x8_t _v16 = vdup_n_u8(16);
  uint8x8_t _v75 = vdup_n_u8(75);
  uint8x8_t _vu64 = vdup_n_u8(64);
  int8x8_t _v52 = vdup_n_s8(52);
  int8x8_t _v25 = vdup_n_s8(25);
  int8x8_t _v102 = vdup_n_s8(102);
  int16x8_t _v64 = vdupq_n_s16(64);

  for (int y = 0; y < height; y += 2)
  {
    const uint8_t* yptr0 = yptr;
    const uint8_t* yptr1 = yptr + width;
    unsigned char* rgb0 = bgr24;
    unsigned char* rgb1 = bgr24 + width * 3;
    int nn = width >> 3;

    for (; nn > 0; nn--)
    {
      int16x8_t _yy0 = vreinterpretq_s16_u16(vmull_u8(vqsub_u8(vld1_u8(yptr0), _v16), _v75));
      int16x8_t _yy1 = vreinterpretq_s16_u16(vmull_u8(vqsub_u8(vld1_u8(yptr1), _v16), _v75));
//      int16x8_t _yy0 = vreinterpretq_s16_u16(vmull_u8(vld1_u8(yptr0), _v75));
//      int16x8_t _yy1 = vreinterpretq_s16_u16(vmull_u8(vld1_u8(yptr1), _v75));
      int8x8_t _uuvv = vreinterpret_s8_u8(vsub_u8(vld1_u8(uvptr), _v128));
      int8x8x2_t _uuuuvvvv = vtrn_s8(_uuvv, _uuvv);
      int8x8_t _uu = _uuuuvvvv.val[0];
      int8x8_t _vv = _uuuuvvvv.val[1];

      int16x8_t _r0 = vmlal_s8(_yy0, _vv, _v102);
      int16x8_t _g0 = vmlsl_s8(_yy0, _vv, _v52);
      _g0 = vmlsl_s8(_g0, _uu, _v25);
      int16x8_t _b0 = vmlal_s8(_yy0, _uu, _v127);


      int16x8_t _r1 = vmlal_s8(_yy1, _vv, _v102);
      int16x8_t _g1 = vmlsl_s8(_yy1, _vv, _v52);
      _g1 = vmlsl_s8(_g1, _uu, _v25);
      int16x8_t _b1 = vmlal_s8(_yy1, _uu, _v127);

      uint8x8x3_t _rgb0;
      _rgb0.val[2] = vqshrun_n_s16(vaddq_s16(_r0, _v64), 6);
      _rgb0.val[1] = vqshrun_n_s16(vaddq_s16(_g0, _v64), 6);
      _rgb0.val[0] = vqshrun_n_s16(vaddq_s16(_b0, _v64), 6);

      uint8x8x3_t _rgb1;
      _rgb1.val[2] = vqshrun_n_s16(vaddq_s16(_r1, _v64), 6);
      _rgb1.val[1] = vqshrun_n_s16(vaddq_s16(_g1, _v64), 6);
      _rgb1.val[0] = vqshrun_n_s16(vaddq_s16(_b1, _v64), 6);

      vst3_u8(rgb0, _rgb0);
      vst3_u8(rgb1, _rgb1);

      yptr0 += 8;
      yptr1 += 8;
      uvptr += 8;
      rgb0 += 24;
      rgb1 += 24;
    }
    yptr += 2 * width;
    bgr24 += 2 * 3 * width;
  }
}

static void bgr24_to_nv12_neon(uint8_t* bgr24, uint8_t* nv12, int width, int height) {
  int frameSize = width * height;
  int yIndex = 0;
  int uvIndex = frameSize;
  const uint16x8_t u16_rounding = vdupq_n_u16(128);
  const int16x8_t s16_rounding = vdupq_n_s16(128);
  const int8x8_t s8_rounding = vdup_n_s8(128);
  const uint8x16_t offset = vdupq_n_u8(16);
  const uint16x8_t mask = vdupq_n_u16(255);

  for (int j = 0; j < height; j++) {
    for (int i = 0; i < width >> 4; i++) {
      // Load rgb
      uint8x16x3_t pixel_rgb;
      pixel_rgb = vld3q_u8(bgr24);
      bgr24 += 48;

      uint8x8x2_t uint8_r;
      uint8x8x2_t uint8_g;
      uint8x8x2_t uint8_b;
      uint8_r.val[0] = vget_low_u8(pixel_rgb.val[2]);
      uint8_r.val[1] = vget_high_u8(pixel_rgb.val[2]);
      uint8_g.val[0] = vget_low_u8(pixel_rgb.val[1]);
      uint8_g.val[1] = vget_high_u8(pixel_rgb.val[1]);
      uint8_b.val[0] = vget_low_u8(pixel_rgb.val[0]);
      uint8_b.val[1] = vget_high_u8(pixel_rgb.val[0]);

      uint16x8x2_t uint16_y;
      uint8x8_t scalar = vdup_n_u8(66);
      uint8x16_t y;

      uint16_y.val[0] = vmull_u8(uint8_r.val[0], scalar);
      uint16_y.val[1] = vmull_u8(uint8_r.val[1], scalar);
      scalar = vdup_n_u8(129);
      uint16_y.val[0] = vmlal_u8(uint16_y.val[0], uint8_g.val[0], scalar);
      uint16_y.val[1] = vmlal_u8(uint16_y.val[1], uint8_g.val[1], scalar);
      scalar = vdup_n_u8(25);
      uint16_y.val[0] = vmlal_u8(uint16_y.val[0], uint8_b.val[0], scalar);
      uint16_y.val[1] = vmlal_u8(uint16_y.val[1], uint8_b.val[1], scalar);

      uint16_y.val[0] = vaddq_u16(uint16_y.val[0], u16_rounding);
      uint16_y.val[1] = vaddq_u16(uint16_y.val[1], u16_rounding);

      y = vcombine_u8(vqshrn_n_u16(uint16_y.val[0], 8), vqshrn_n_u16(uint16_y.val[1], 8));
      y = vaddq_u8(y, offset);

      vst1q_u8(nv12 + yIndex, y);
      yIndex += 16;

      // Compute u and v in the even row
      if (j % 2 == 0) {
        int16x8_t u_scalar = vdupq_n_s16(-38);
        int16x8_t v_scalar = vdupq_n_s16(112);

        int16x8_t r = vreinterpretq_s16_u16(vandq_u16(vreinterpretq_u16_u8(pixel_rgb.val[2]), mask));
        int16x8_t g = vreinterpretq_s16_u16(vandq_u16(vreinterpretq_u16_u8(pixel_rgb.val[1]), mask));
        int16x8_t b = vreinterpretq_s16_u16(vandq_u16(vreinterpretq_u16_u8(pixel_rgb.val[0]), mask));

        int16x8_t u;
        int16x8_t v;
        uint8x8x2_t uv;

        u = vmulq_s16(r, u_scalar);
        v = vmulq_s16(r, v_scalar);

        u_scalar = vdupq_n_s16(-74);
        v_scalar = vdupq_n_s16(-94);
        u = vmlaq_s16(u, g, u_scalar);
        v = vmlaq_s16(v, g, v_scalar);

        u_scalar = vdupq_n_s16(112);
        v_scalar = vdupq_n_s16(-18);
        u = vmlaq_s16(u, b, u_scalar);
        v = vmlaq_s16(v, b, v_scalar);

        u = vaddq_s16(u, s16_rounding);
        v = vaddq_s16(v, s16_rounding);

        uv.val[0] = vreinterpret_u8_s8(vadd_s8(vqshrn_n_s16(u, 8), s8_rounding));
        uv.val[1] = vreinterpret_u8_s8(vadd_s8(vqshrn_n_s16(v, 8), s8_rounding));

        vst2_u8(nv12 + uvIndex, uv);

        uvIndex += 16;
      }
    }
  }
}

struct NV12ScaleParams {
  int src_w;
  int src_h;
  int dst_w;
  int dst_h;
  float x_ratio;
  float y_ratio;
  int32_t *x0_array;
  int32_t *x1_array;
  uint8_t *a_array;
};

void init_scale_params(
    NV12ScaleParams &params, int src_w, int src_h, int dst_w, int dst_h) {
  params.src_w = src_w;
  params.src_h = src_h;
  params.dst_w = dst_w;
  params.dst_h = dst_h;
  params.x_ratio = static_cast<float>(src_w) / dst_w;
  params.y_ratio = static_cast<float>(src_h) / dst_h;

  // 使用posix_memalign替代memalign
  posix_memalign((void **)&params.x0_array, 64, dst_w * sizeof(int32_t) * 2);
  params.x1_array = params.x0_array + dst_w;
  posix_memalign((void **)&params.a_array, 64, dst_w * sizeof(uint8_t));

#pragma omp parallel for
  for (int dx = 0; dx < dst_w; ++dx) {
    float sx = dx * params.x_ratio;
    params.x0_array[dx] = static_cast<int>(sx);
    params.x1_array[dx] = std::min(params.x0_array[dx] + 1, src_w - 1);
    params.a_array[dx] = static_cast<uint8_t>((sx - params.x0_array[dx]) * 255);
  }
}

// 释放缩放参数
void free_scale_params(NV12ScaleParams &params) {
  free(params.x0_array);
  free(params.a_array);
}

// 像素收集函数实现
inline void gather_pixels_16(const uint8_t *base,
                             int32x4_t x0,
                             int32x4_t x1,
                             int32x4_t x2,
                             int32x4_t x3,
                             uint8x16_t *result) {
  uint8x8_t low = vld1_u8(base + vgetq_lane_s32(x0, 0));
  low = vld1_lane_u8(base + vgetq_lane_s32(x0, 1), low, 1);
  low = vld1_lane_u8(base + vgetq_lane_s32(x0, 2), low, 2);
  low = vld1_lane_u8(base + vgetq_lane_s32(x0, 3), low, 3);
  low = vld1_lane_u8(base + vgetq_lane_s32(x1, 0), low, 4);
  low = vld1_lane_u8(base + vgetq_lane_s32(x1, 1), low, 5);
  low = vld1_lane_u8(base + vgetq_lane_s32(x1, 2), low, 6);
  low = vld1_lane_u8(base + vgetq_lane_s32(x1, 3), low, 7);

  uint8x8_t high = vld1_u8(base + vgetq_lane_s32(x2, 0));
  high = vld1_lane_u8(base + vgetq_lane_s32(x2, 1), high, 1);
  high = vld1_lane_u8(base + vgetq_lane_s32(x2, 2), high, 2);
  high = vld1_lane_u8(base + vgetq_lane_s32(x2, 3), high, 3);
  high = vld1_lane_u8(base + vgetq_lane_s32(x3, 0), high, 4);
  high = vld1_lane_u8(base + vgetq_lane_s32(x3, 1), high, 5);
  high = vld1_lane_u8(base + vgetq_lane_s32(x3, 2), high, 6);
  high = vld1_lane_u8(base + vgetq_lane_s32(x3, 3), high, 7);

  *result = vcombine_u8(low, high);
}

// UV解交织示例
inline void load_and_deinterleave(const uint8_t *src,
                                  int32x4_t x0,
                                  int32x4_t x1,
                                  uint8x8x2_t *uv) {
  uint8x8_t data0 = vld1_u8(src + vgetq_lane_s32(x0, 0));
  data0 = vld1_lane_u8(src + vgetq_lane_s32(x0, 1), data0, 1);
  data0 = vld1_lane_u8(src + vgetq_lane_s32(x0, 2), data0, 2);
  data0 = vld1_lane_u8(src + vgetq_lane_s32(x0, 3), data0, 3);
  data0 = vld1_lane_u8(src + vgetq_lane_s32(x1, 0), data0, 4);
  data0 = vld1_lane_u8(src + vgetq_lane_s32(x1, 1), data0, 5);
  data0 = vld1_lane_u8(src + vgetq_lane_s32(x1, 2), data0, 6);
  data0 = vld1_lane_u8(src + vgetq_lane_s32(x1, 3), data0, 7);

  uint8x8_t data1 = vld1_u8(src + vgetq_lane_s32(x0, 0) + 1);
  data1 = vld1_lane_u8(src + vgetq_lane_s32(x0, 1) + 1, data1, 1);
  data1 = vld1_lane_u8(src + vgetq_lane_s32(x0, 2) + 1, data1, 2);
  data1 = vld1_lane_u8(src + vgetq_lane_s32(x0, 3) + 1, data1, 3);
  data1 = vld1_lane_u8(src + vgetq_lane_s32(x1, 0) + 1, data1, 4);
  data1 = vld1_lane_u8(src + vgetq_lane_s32(x1, 1) + 1, data1, 5);
  data1 = vld1_lane_u8(src + vgetq_lane_s32(x1, 2) + 1, data1, 6);
  data1 = vld1_lane_u8(src + vgetq_lane_s32(x1, 3) + 1, data1, 7);

  (*uv).val[0] = data0;
  (*uv).val[1] = data1;
}

// 双线性插值函数实现（16像素版本）
inline uint8x16_t bilinear_interp_16(uint8x16_t p00,
                                     uint8x16_t p01,
                                     uint8x16_t p10,
                                     uint8x16_t p11,
                                     uint8x16_t a_coeff,
                                     uint8x8_t vec_a,
                                     uint8x8_t vec_b) {
  uint8x16_t inv_a_coeff = vsubq_u8(vdupq_n_u8(255), a_coeff);

  // 水平插值
  uint16x8_t h0_low = vmull_u8(vget_low_u8(p00), vget_low_u8(inv_a_coeff));
  h0_low = vmlal_u8(h0_low, vget_low_u8(p01), vget_low_u8(a_coeff));
  uint16x8_t h0_high = vmull_u8(vget_high_u8(p00), vget_high_u8(inv_a_coeff));
  h0_high = vmlal_u8(h0_high, vget_high_u8(p01), vget_high_u8(a_coeff));

  uint16x8_t h1_low = vmull_u8(vget_low_u8(p10), vget_low_u8(inv_a_coeff));
  h1_low = vmlal_u8(h1_low, vget_low_u8(p11), vget_low_u8(a_coeff));
  uint16x8_t h1_high = vmull_u8(vget_high_u8(p10), vget_high_u8(inv_a_coeff));
  h1_high = vmlal_u8(h1_high, vget_high_u8(p11), vget_high_u8(a_coeff));

  uint16x8_t v_low = vmull_u8(vshrn_n_u16(h0_low, 8), vec_a);
  v_low = vmlal_u8(v_low, vshrn_n_u16(h1_low, 8), vec_b);

  uint16x8_t v_high = vmull_u8(vshrn_n_u16(h0_high, 8), vec_a);
  v_high = vmlal_u8(v_high, vshrn_n_u16(h1_high, 8), vec_b);

  // 归一化
  uint8x8_t res_low = vshrn_n_u16(v_low, 8);
  uint8x8_t res_high = vshrn_n_u16(v_high, 8);
  return vcombine_u8(res_low, res_high);
}

// UV插值函数
inline uint8x8_t bilinear_interp_uv(uint8x8_t p00,
                                    uint8x8_t p01,
                                    uint8x8_t p10,
                                    uint8x8_t p11,
                                    uint8x8_t a_coeff,
                                    uint8x8_t vec_a,
                                    uint8x8_t vec_b) {
  uint8x8_t inv_a_coeff = vsub_u8(vdup_n_u8(255), a_coeff);

  // 水平插值
  uint16x8_t h0 = vmlal_u8(vmull_u8(p00, inv_a_coeff), p01, a_coeff);
  uint16x8_t h1 = vmlal_u8(vmull_u8(p10, inv_a_coeff), p11, a_coeff);

  // 垂直插值
  uint16x8_t v = vmull_u8(vshrn_n_u16(h0, 8), vec_a);
  v = vmlal_u8(v, vshrn_n_u16(h1, 8), vec_b);

  return vshrn_n_u16(v, 8);
}

// 修正后的Y平面缩放函数
void neon_scale_y(const uint8_t *src_y,
                  uint8_t *dst_y,
                  const NV12ScaleParams &params) {
  const int src_stride = params.src_w;
  const int dst_stride = params.dst_w;

#pragma omp parallel for
  for (int dy = 0; dy < params.dst_h; ++dy) {
    const float sy = dy * params.y_ratio;
    const int y0 = static_cast<int>(sy);
    const int y1 = std::min(y0 + 1, params.src_h - 1);
    const uint8_t b = static_cast<uint8_t>((sy - y0) * 255);
    const uint8_t a = 255 - b;
    uint8x8_t vec_b = vdup_n_u8(b);
    uint8x8_t vec_a = vsub_u8(vdup_n_u8(255), vec_b);

    const uint8_t *src_row0 = src_y + y0 * src_stride;
    const uint8_t *src_row1 = src_y + y1 * src_stride;
    uint8_t *dst_row = dst_y + dy * dst_stride;
    int dx = 0;
    for (; dx <= params.dst_w - 16; dx += 16) {
      int32x4_t x0_0 = vld1q_s32(params.x0_array + dx);
      int32x4_t x0_1 = vld1q_s32(params.x0_array + dx + 4);
      int32x4_t x0_2 = vld1q_s32(params.x0_array + dx + 8);
      int32x4_t x0_3 = vld1q_s32(params.x0_array + dx + 12);
      int32x4_t x1_0 = vld1q_s32(params.x1_array + dx);
      int32x4_t x1_1 = vld1q_s32(params.x1_array + dx + 4);
      int32x4_t x1_2 = vld1q_s32(params.x1_array + dx + 8);
      int32x4_t x1_3 = vld1q_s32(params.x1_array + dx + 12);

      uint8x16_t a_coeff = vld1q_u8(params.a_array + dx);

      uint8x16_t p00, p01, p10, p11;
      gather_pixels_16(src_row0, x0_0, x0_1, x0_2, x0_3, &p00);
      gather_pixels_16(src_row0, x1_0, x1_1, x1_2, x1_3, &p01);
      gather_pixels_16(src_row1, x0_0, x0_1, x0_2, x0_3, &p10);
      gather_pixels_16(src_row1, x1_0, x1_1, x1_2, x1_3, &p11);

      // ...类似处理其他像素...

      uint8x16_t result =
          bilinear_interp_16(p00, p01, p10, p11, a_coeff, vec_a, vec_b);
      vst1q_u8(dst_row + dx, result);
    }
    // 处理剩余像素（标量代码）
    for (; dx < params.dst_w; ++dx) {
      const int x0 = params.x0_array[dx];
      const int x1 = params.x1_array[dx];
      const uint8_t a1 = params.a_array[dx];
      const uint8_t inv_a1 = 255 - a1;

      const uint8_t p00 = src_row0[x0];
      const uint8_t p01 = src_row0[x1];
      const uint8_t p10 = src_row1[x0];
      const uint8_t p11 = src_row1[x1];

      // 水平插值
      const uint16_t h0 = p00 * inv_a1 + p01 * a1;
      const uint16_t h1 = p10 * inv_a1 + p11 * a1;

      // 垂直插值
      const uint16_t v = (h0 * a + h1 * b + 32512) / 65025;

      dst_row[dx] = static_cast<uint8_t>(v);
    }
  }
}

// Neon优化UV平面缩放（处理交错UV）
void neon_scale_uv(const uint8_t *src_uv,
                   uint8_t *dst_uv,
                   const NV12ScaleParams &params) {
  const int uv_src_w = params.src_w / 2;
  const int uv_src_h = params.src_h / 2;
  const int uv_dst_w = params.dst_w / 2;
  const int uv_dst_h = params.dst_h / 2;

  NV12ScaleParams uv_params;
  init_scale_params(uv_params, uv_src_w, uv_src_h, uv_dst_w, uv_dst_h);

#pragma omp parallel for
  for (int dy = 0; dy < uv_dst_h; ++dy) {
    const float sy = dy * uv_params.y_ratio;
    const int y0 = static_cast<int>(sy);
    const int y1 = std::min(y0 + 1, uv_src_h - 1);
    const uint8_t b = static_cast<uint8_t>((sy - y0) * 255);
    const uint8_t a = 255 - b;
    uint8x8_t vec_b = vdup_n_u8(b);
    uint8x8_t vec_a = vsub_u8(vdup_n_u8(255), vec_b);

    const uint8_t *src_row0 = src_uv + y0 * params.src_w;
    const uint8_t *src_row1 = src_uv + y1 * params.src_w;
    uint8_t *dst_row = dst_uv + dy * params.dst_w;

    int dx = 0;
    for (; dx <= uv_dst_w - 8; dx += 8) {
      // 加载UV坐标参数（每个UV对应两个Y像素）
      int32x4_t x0_0 = vshlq_n_s32(vld1q_s32(uv_params.x0_array + dx), 1);
      int32x4_t x0_1 = vshlq_n_s32(vld1q_s32(uv_params.x0_array + dx + 4), 1);
      int32x4_t x1_0 = vshlq_n_s32(vld1q_s32(uv_params.x1_array + dx), 1);
      int32x4_t x1_1 = vshlq_n_s32(vld1q_s32(uv_params.x1_array + dx + 4), 1);

      uint8x8_t a_coeff = vld1_u8(uv_params.a_array + dx);

      // 加载并解交织UV分量
      uint8x8x2_t uv00, uv01, uv10, uv11;
      load_and_deinterleave(src_row0, x0_0, x0_1, &uv00);
      load_and_deinterleave(src_row0, x1_0, x1_1, &uv01);
      load_and_deinterleave(src_row1, x0_0, x0_1, &uv10);
      load_and_deinterleave(src_row1, x1_0, x1_1, &uv11);

      // 分别处理U和V分量
      uint8x8x2_t result;
      result.val[0] = bilinear_interp_uv(uv00.val[0],
                                         uv01.val[0],
                                         uv10.val[0],
                                         uv11.val[0],
                                         a_coeff,
                                         vec_a,
                                         vec_b);
      result.val[1] = bilinear_interp_uv(uv00.val[1],
                                         uv01.val[1],
                                         uv10.val[1],
                                         uv11.val[1],
                                         a_coeff,
                                         vec_a,
                                         vec_b);

      vst2_u8(dst_row + dx * 2, result);
    }
    // 处理剩余像素（标量代码）
    for (; dx < uv_dst_w; ++dx) {
      const int x0 = (params.x0_array[dx]) * 2;
      const int x1 = (params.x1_array[dx]) * 2;
      const uint8_t a1 = params.a_array[dx];
      const uint8_t inv_a1 = 255 - a1;

      uint8_t p00 = src_row0[x0];
      uint8_t p01 = src_row0[x1];
      uint8_t p10 = src_row1[x0];
      uint8_t p11 = src_row1[x1];

      // 水平插值
      uint16_t h0 = p00 * inv_a1 + p01 * a1;
      uint16_t h1 = p10 * inv_a1 + p11 * a1;

      // 垂直插值
      uint16_t v = (h0 * a + h1 * b + 32512) / 65025;

      dst_row[dx * 2] = static_cast<uint8_t>(v);

      p00 = src_row0[x0 + 1];
      p01 = src_row0[x1 + 1];
      p10 = src_row1[x0 + 1];
      p11 = src_row1[x1 + 1];

      // 水平插值
      h0 = p00 * inv_a1 + p01 * a1;
      h1 = p10 * inv_a1 + p11 * a1;

      // 垂直插值
      v = (h0 * a + h1 * b + 32512) / 65025;

      dst_row[dx * 2 + 1] = static_cast<uint8_t>(v);
    }
  }
  free_scale_params(uv_params);
}


// NV12缩放主函数
void neon_nv12_resize(const uint8_t *src,
                      uint8_t *dst,
                      int src_w,
                      int src_h,
                      int dst_w,
                      int dst_h) {
  NV12ScaleParams params;
  init_scale_params(params, src_w, src_h, dst_w, dst_h);

  const int y_size = src_w * src_h;
  neon_scale_y(src, dst, params);                            // 处理Y平面
  neon_scale_uv(src + y_size, dst + dst_w * dst_h, params);  // 处理UV平面
  free_scale_params(params);
}

static int hobotcv_neon_resize_nv12(const cv::Mat &src,
                   int src_h,
                   int src_w,
                   cv::Mat &dst,
                   int dst_h,
                   int dst_w) {
  dst = cv::Mat(dst_h * 3 / 2, dst_w, CV_8UC1);
  neon_nv12_resize((const uint8_t *)src.data,
                   (uint8_t *)dst.data,
                   src_w,
                   src_h,
                   dst_w,
                   dst_h);
  return 0;
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

  if (((src_w > src_h) && (dst_w < dst_h)) || ((src_w > dst_w) && (src_h < dst_h)) ||
  ((src_w < src_h) && (dst_w > dst_h)) || ((src_w < dst_w) && (src_h > dst_h))) {
    RCLCPP_INFO(rclcpp::get_logger("hobot_cv"),
                "Resolution not suitable for VSE, using NEON resize: "
                "src(%d x %d) -> dst(%d x %d)",
                src_w,
                src_h,
                dst_w,
                dst_h);
    ret = hobotcv_neon_resize_nv12(src, src_h, src_w, dst, dst_h, dst_w);
    {
      struct timespec ts;
      clock_gettime(CLOCK_MONOTONIC, &ts);
      msEnd = (ts.tv_sec * 1000 + ts.tv_nsec / 1000000);
    }
    RCLCPP_INFO(rclcpp::get_logger("hobot_cv"),
                "hobotcv_resize neon laps ms= %ld",
                (msEnd - msStart));
    return ret;
  }

  RCLCPP_INFO(rclcpp::get_logger("hobot_cv"),
                "Using VSE resize: "
                "src(%d x %d) -> dst(%d x %d)",
                src_w,
                src_h,
                dst_w,
                dst_h);

  auto front_ptr = hobotcv_front_group::getInstance().getHobotcvFront(src_w, src_h, dst_w, dst_h, cv::Range(0, 0), cv::Range(0, 0));
  if (front_ptr == nullptr) {
    return -1;
  }
  
  dst = cv::Mat(dst_h * 3 / 2, dst_w, CV_8UC1);
  int dst_size = dst_w * dst_h * 1.5;
  ret = front_ptr->processFrame((const char*)src.data, src_w, src_h, (char*)dst.data, dst_size);
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

  auto front_ptr = hobotcv_front_group::getInstance().getHobotcvFront(src_w, src_h, dst_w, dst_h, rowRange, colRange);
  if (front_ptr == nullptr) {
    return dst_null;
  }
  
  cv::Mat dst(dst_h * 3 / 2, dst_w, CV_8UC1);
  int dst_size = dst_w * dst_h * 1.5;
  ret = front_ptr->processFrame((const char*)src.data, src_w, src_h, (char*)dst.data, dst_size);
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

int hobotcv_color(const cv::Mat &src, cv::Mat &dst, COLOR_E color) {
   int width = src.cols;
  int height = src.rows;
  switch (color) {
    case DCOLOR_YUV2BGR_NV12:
      height = src.rows / 1.5;
      dst.create(height, width, CV_8UC3);
      nv12_to_bgr24_neon(src.data, dst.data, width, height);
      break;
    case DCOLOR_BGR2YUV_NV12:
      dst.create(height * 1.5, width, CV_8UC1);
      bgr24_to_nv12_neon(src.data, dst.data, width, height);
      break;
    default:
      return -1;
  }
  return 0;
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

  auto front_ptr = hobotcv_front_group::getInstance().getHobotcvFront(src_w, src_h, dst_w, dst_h, cv::Range(0, 0), cv::Range(0, 0));
  if (front_ptr == nullptr) {
    return nullptr;
  }
  
  //dst = cv::Mat(dst_h * 3 / 2, dst_w, CV_8UC1);

  int dst_size = dst_w * dst_h * 1.5;
  char* dst = (char *)malloc(dst_size);
  ret = front_ptr->processFrame(src, src_w, src_h, dst, dst_size);
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

  auto front_ptr = hobotcv_front_group::getInstance().getHobotcvFront(src_w, src_h, dst_w, dst_h, rowRange, colRange);
  if (front_ptr == nullptr) {
    return nullptr;
  }

  int dst_size = dst_w * dst_h * 1.5;
  char* dst = (char *)malloc(dst_size);
  ret = front_ptr->processFrame(src, src_w, src_h, dst, dst_size);
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

std::shared_ptr<ImageInfo>  hobotcv_color(const char *src, int src_h, int src_w, COLOR_E color) {
  auto imageInfo = new ImageInfo;
  imageInfo->width = src_w;
  imageInfo->height = src_h;
  switch (color) {
    case DCOLOR_YUV2BGR_NV12:
      imageInfo->imageAddr = malloc(src_h * src_w * 3);
      nv12_to_bgr24_neon((uint8_t*)src, (uint8_t*)imageInfo->imageAddr, src_w, src_h);
      break;
    case DCOLOR_BGR2YUV_NV12:
      imageInfo->imageAddr = malloc(src_h * src_w * 1.5);
      bgr24_to_nv12_neon((uint8_t*)src, (uint8_t*)imageInfo->imageAddr, src_w, src_h);
      break;
    default:
      return nullptr;
  }
  return std::shared_ptr<ImageInfo>(imageInfo,
                                      [](ImageInfo *imageInfo) {
                                        free(imageInfo->imageAddr);
                                        delete imageInfo;
                                      });
}

}  // namespace hobot_cv
