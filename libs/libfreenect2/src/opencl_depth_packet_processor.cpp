/*
 * This file is part of the OpenKinect Project. http://www.openkinect.org
 *
 * Copyright (c) 2014 individual OpenKinect contributors. See the CONTRIB file
 * for details.
 *
 * This code is licensed to you under the terms of the Apache License, version
 * 2.0, or, at your option, the terms of the GNU General Public License,
 * version 2.0. See the APACHE20 and GPL2 files for the text of the licenses,
 * or the following URLs:
 * http://www.apache.org/licenses/LICENSE-2.0
 * http://www.gnu.org/licenses/gpl-2.0.txt
 *
 * If you redistribute this file in source form, modified or unmodified, you
 * may:
 *   1) Leave this header intact and distribute it under the same terms,
 *      accompanying it with the APACHE20 and GPL20 files, or
 *   2) Delete the Apache 2.0 clause and accompany it with the GPL2 file, or
 *   3) Delete the GPL v2 clause and accompany it with the APACHE20 file
 * In all cases you must keep the copyright notice intact and include a copy
 * of the CONTRIB file.
 *
 * Binary distributions must follow the binary distribution requirements of
 * either License.
 */

/** @file opencl_depth_packet_processor.cl Implementation of the OpenCL depth packet processor. */

#include <libfreenect2/depth_packet_processor.h>
#include <libfreenect2/resource.h>
#include <libfreenect2/protocol/response.h>
#include <libfreenect2/logging.h>

#include <sstream>

#define _USE_MATH_DEFINES
#include <math.h>

#define CL_USE_DEPRECATED_OPENCL_1_2_APIS
#define CL_USE_DEPRECATED_OPENCL_2_0_APIS

#ifdef LIBFREENECT2_OPENCL_ICD_LOADER_IS_OLD
#define CL_USE_DEPRECATED_OPENCL_1_1_APIS
#include <CL/cl.h>
#ifdef CL_VERSION_1_2
#undef CL_VERSION_1_2
#endif //CL_VERSION_1_2
#endif //LIBFREENECT2_OPENCL_ICD_LOADER_IS_OLD

#include <CL/cl.hpp>

#ifndef REG_OPENCL_FILE
#define REG_OPENCL_FILE ""
#endif

namespace libfreenect2
{

std::string loadCLSource(const std::string &filename)
{
  const unsigned char *data;
  size_t length = 0;

  if(!loadResource(filename, &data, &length))
  {
    LOG_ERROR << "failed to load cl source!";
    return "";
  }

  return std::string(reinterpret_cast<const char *>(data), length);
}

static const std::string clInlineSource = R"(

/*
 * This file is part of the OpenKinect Project. http://www.openkinect.org
 *
 * Copyright (c) 2014 individual OpenKinect contributors. See the CONTRIB file
 * for details.
 *
 * This code is licensed to you under the terms of the Apache License, version
 * 2.0, or, at your option, the terms of the GNU General Public License,
 * version 2.0. See the APACHE20 and GPL2 files for the text of the licenses,
 * or the following URLs:
 * http://www.apache.org/licenses/LICENSE-2.0
 * http://www.gnu.org/licenses/gpl-2.0.txt
 *
 * If you redistribute this file in source form, modified or unmodified, you
 * may:
 *   1) Leave this header intact and distribute it under the same terms,
 *      accompanying it with the APACHE20 and GPL20 files, or
 *   2) Delete the Apache 2.0 clause and accompany it with the GPL2 file, or
 *   3) Delete the GPL v2 clause and accompany it with the APACHE20 file
 * In all cases you must keep the copyright notice intact and include a copy
 * of the CONTRIB file.
 *
 * Binary distributions must follow the binary distribution requirements of
 * either License.
 */

/*******************************************************************************
 * Process pixel stage 1
 ******************************************************************************/

float decodePixelMeasurement(global const ushort *data, global const short *lut11to16, const uint sub, const uint x, const uint y)
{
  uint row_idx = (424 * sub + (y < 212 ? y + 212 : 423 - y)) * 352;
  uint idx = (((x >> 2) + ((x << 7) & BFI_BITMASK)) * 11) & (uint)0xffffffff;

  uint col_idx = idx >> 4;
  uint upper_bytes = idx & 15;
  uint lower_bytes = 16 - upper_bytes;

  uint data_idx0 = row_idx + col_idx;
  uint data_idx1 = row_idx + col_idx + 1;

  return (float)lut11to16[(x < 1 || 510 < x || col_idx > 352) ? 0 : ((data[data_idx0] >> upper_bytes) | (data[data_idx1] << lower_bytes)) & 2047];
}

float2 processMeasurementTriple(const float ab_multiplier_per_frq, const float p0, const float3 v, int *invalid)
{
  float3 p0vec = (float3)(p0 + PHASE_IN_RAD0, p0 + PHASE_IN_RAD1, p0 + PHASE_IN_RAD2);
  float3 p0cos = cos(p0vec);
  float3 p0sin = sin(-p0vec);

  *invalid = *invalid && any(isequal(v, (float3)(32767.0f)));

  return (float2)(dot(v, p0cos), dot(v, p0sin)) * ab_multiplier_per_frq;
}

void kernel processPixelStage1(global const short *lut11to16, global const float *z_table, global const float3 *p0_table, global const ushort *data,
                               global float3 *a_out, global float3 *b_out, global float3 *n_out, global float *ir_out)
{
  const uint i = get_global_id(0);

  const uint x = i % 512;
  const uint y = i / 512;

  const uint y_in = (423 - y);

  const float zmultiplier = z_table[i];
  int valid = (int)(0.0f < zmultiplier);
  int saturatedX = valid;
  int saturatedY = valid;
  int saturatedZ = valid;
  int3 invalid_pixel = (int3)((int)(!valid));
  const float3 p0 = p0_table[i];

  const float3 v0 = (float3)(decodePixelMeasurement(data, lut11to16, 0, x, y_in),
                             decodePixelMeasurement(data, lut11to16, 1, x, y_in),
                             decodePixelMeasurement(data, lut11to16, 2, x, y_in));
  const float2 ab0 = processMeasurementTriple(AB_MULTIPLIER_PER_FRQ0, p0.x, v0, &saturatedX);

  const float3 v1 = (float3)(decodePixelMeasurement(data, lut11to16, 3, x, y_in),
                             decodePixelMeasurement(data, lut11to16, 4, x, y_in),
                             decodePixelMeasurement(data, lut11to16, 5, x, y_in));
  const float2 ab1 = processMeasurementTriple(AB_MULTIPLIER_PER_FRQ1, p0.y, v1, &saturatedY);

  const float3 v2 = (float3)(decodePixelMeasurement(data, lut11to16, 6, x, y_in),
                             decodePixelMeasurement(data, lut11to16, 7, x, y_in),
                             decodePixelMeasurement(data, lut11to16, 8, x, y_in));
  const float2 ab2 = processMeasurementTriple(AB_MULTIPLIER_PER_FRQ2, p0.z, v2, &saturatedZ);

  float3 a = select((float3)(ab0.x, ab1.x, ab2.x), (float3)(0.0f), invalid_pixel);
  float3 b = select((float3)(ab0.y, ab1.y, ab2.y), (float3)(0.0f), invalid_pixel);
  float3 n = sqrt(a * a + b * b);

  int3 saturated = (int3)(saturatedX, saturatedY, saturatedZ);
  a = select(a, (float3)(0.0f), saturated);
  b = select(b, (float3)(0.0f), saturated);

  a_out[i] = a;
  b_out[i] = b;
  n_out[i] = n;
  ir_out[i] = min(dot(select(n, (float3)(65535.0f), saturated), (float3)(0.333333333f  * AB_MULTIPLIER * AB_OUTPUT_MULTIPLIER)), 65535.0f);
}

/*******************************************************************************
 * Filter pixel stage 1
 ******************************************************************************/
void kernel filterPixelStage1(global const float3 *a, global const float3 *b, global const float3 *n,
                              global float3 *a_out, global float3 *b_out, global uchar *max_edge_test)
{
  const uint i = get_global_id(0);

  const uint x = i % 512;
  const uint y = i / 512;

  const float3 self_a = a[i];
  const float3 self_b = b[i];

  const float gaussian[9] = {GAUSSIAN_KERNEL_0, GAUSSIAN_KERNEL_1, GAUSSIAN_KERNEL_2, GAUSSIAN_KERNEL_3, GAUSSIAN_KERNEL_4, GAUSSIAN_KERNEL_5, GAUSSIAN_KERNEL_6, GAUSSIAN_KERNEL_7, GAUSSIAN_KERNEL_8};

  if(x < 1 || y < 1 || x > 510 || y > 422)
  {
    a_out[i] = self_a;
    b_out[i] = self_b;
    max_edge_test[i] = 1;
  }
  else
  {
    float3 threshold = (float3)(JOINT_BILATERAL_THRESHOLD);
    float3 joint_bilateral_exp = (float3)(JOINT_BILATERAL_EXP);

    const float3 self_norm = n[i];
    const float3 self_normalized_a = self_a / self_norm;
    const float3 self_normalized_b = self_b / self_norm;

    float3 weight_acc = (float3)(0.0f);
    float3 weighted_a_acc = (float3)(0.0f);
    float3 weighted_b_acc = (float3)(0.0f);
    float3 dist_acc = (float3)(0.0f);

    const int3 c0 = isless(self_norm * self_norm, threshold);

    threshold = select(threshold, (float3)(0.0f), c0);
    joint_bilateral_exp = select(joint_bilateral_exp, (float3)(0.0f), c0);

    for(int yi = -1, j = 0; yi < 2; ++yi)
    {
      uint i_other = (y + yi) * 512 + x - 1;

      for(int xi = -1; xi < 2; ++xi, ++j, ++i_other)
      {
        const float3 other_a = a[i_other];
        const float3 other_b = b[i_other];
        const float3 other_norm = n[i_other];
        const float3 other_normalized_a = other_a / other_norm;
        const float3 other_normalized_b = other_b / other_norm;

        const int3 c1 = isless(other_norm * other_norm, threshold);

        const float3 dist = 0.5f * (1.0f - (self_normalized_a * other_normalized_a + self_normalized_b * other_normalized_b));
        const float3 weight = select(gaussian[j] * exp(-1.442695f * joint_bilateral_exp * dist), (float3)(0.0f), c1);

        weighted_a_acc += weight * other_a;
        weighted_b_acc += weight * other_b;
        weight_acc += weight;
        dist_acc += select(dist, (float3)(0.0f), c1);
      }
    }

    const int3 c2 = isless((float3)(0.0f), weight_acc.xyz);
    a_out[i] = select((float3)(0.0f), weighted_a_acc / weight_acc, c2);
    b_out[i] = select((float3)(0.0f), weighted_b_acc / weight_acc, c2);

    max_edge_test[i] = all(isless(dist_acc, (float3)(JOINT_BILATERAL_MAX_EDGE)));
  }
}

/*******************************************************************************
 * Process pixel stage 2
 ******************************************************************************/
void kernel processPixelStage2(global const float3 *a_in, global const float3 *b_in, global const float *x_table, global const float *z_table,
                               global float *depth, global float *ir_sums)
{
  const uint i = get_global_id(0);
  float3 a = a_in[i];
  float3 b = b_in[i];

  float3 phase = atan2(b, a);
  phase = select(phase, phase + 2.0f * M_PI_F, isless(phase, (float3)(0.0f)));
  phase = select(phase, (float3)(0.0f), isnan(phase));
  float3 ir = sqrt(a * a + b * b) * AB_MULTIPLIER;

  float ir_sum = ir.x + ir.y + ir.z;
  float ir_min = min(ir.x, min(ir.y, ir.z));
  float ir_max = max(ir.x, max(ir.y, ir.z));

  float phase_final = 0;

  if(ir_min >= INDIVIDUAL_AB_THRESHOLD && ir_sum >= AB_THRESHOLD)
  {
    float3 t = phase / (2.0f * M_PI_F) * (float3)(3.0f, 15.0f, 2.0f);

    float t0 = t.x;
    float t1 = t.y;
    float t2 = t.z;

    float t5 = (floor((t1 - t0) * 0.333333f + 0.5f) * 3.0f + t0);
    float t3 = (-t2 + t5);
    float t4 = t3 * 2.0f;

    bool c1 = t4 >= -t4; // true if t4 positive

    float f1 = c1 ? 2.0f : -2.0f;
    float f2 = c1 ? 0.5f : -0.5f;
    t3 *= f2;
    t3 = (t3 - floor(t3)) * f1;

    bool c2 = 0.5f < fabs(t3) && fabs(t3) < 1.5f;

    float t6 = c2 ? t5 + 15.0f : t5;
    float t7 = c2 ? t1 + 15.0f : t1;

    float t8 = (floor((-t2 + t6) * 0.5f + 0.5f) * 2.0f + t2) * 0.5f;

    t6 *= 0.333333f; // = / 3
    t7 *= 0.066667f; // = / 15

    float t9 = (t8 + t6 + t7); // transformed phase measurements (they are transformed and divided by the values the original values were multiplied with)
    float t10 = t9 * 0.333333f; // some avg

    t6 *= 2.0f * M_PI_F;
    t7 *= 2.0f * M_PI_F;
    t8 *= 2.0f * M_PI_F;

    // some cross product
    float t8_new = t7 * 0.826977f - t8 * 0.110264f;
    float t6_new = t8 * 0.551318f - t6 * 0.826977f;
    float t7_new = t6 * 0.110264f - t7 * 0.551318f;

    t8 = t8_new;
    t6 = t6_new;
    t7 = t7_new;

    float norm = t8 * t8 + t6 * t6 + t7 * t7;
    float mask = t9 >= 0.0f ? 1.0f : 0.0f;
    t10 *= mask;

    bool slope_positive = 0 < AB_CONFIDENCE_SLOPE;

    float ir_x = slope_positive ? ir_min : ir_max;

    ir_x = log(ir_x);
    ir_x = (ir_x * AB_CONFIDENCE_SLOPE * 0.301030f + AB_CONFIDENCE_OFFSET) * 3.321928f;
    ir_x = exp(ir_x);
    ir_x = clamp(ir_x, MIN_DEALIAS_CONFIDENCE, MAX_DEALIAS_CONFIDENCE);
    ir_x *= ir_x;

    float mask2 = ir_x >= norm ? 1.0f : 0.0f;

    float t11 = t10 * mask2;

    float mask3 = MAX_DEALIAS_CONFIDENCE * MAX_DEALIAS_CONFIDENCE >= norm ? 1.0f : 0.0f;
    t10 *= mask3;
    phase_final = true/*(modeMask & 2) != 0*/ ? t11 : t10;
  }

  float zmultiplier = z_table[i];
  float xmultiplier = x_table[i];

  phase_final = 0.0f < phase_final ? phase_final + PHASE_OFFSET : phase_final;

  float depth_linear = zmultiplier * phase_final;
  float max_depth = phase_final * UNAMBIGIOUS_DIST * 2.0;

  bool cond1 = /*(modeMask & 32) != 0*/ true && 0.0f < depth_linear && 0.0f < max_depth;

  xmultiplier = (xmultiplier * 90.0) / (max_depth * max_depth * 8192.0);

  float depth_fit = depth_linear / (-depth_linear * xmultiplier + 1);
  depth_fit = depth_fit < 0.0f ? 0.0f : depth_fit;

  float d = cond1 ? depth_fit : depth_linear; // r1.y -> later r2.z
  depth[i] = d;
  ir_sums[i] = ir_sum;
}

/*******************************************************************************
 * Filter pixel stage 2
 ******************************************************************************/
void kernel filterPixelStage2(global const float *depth, global const float *ir_sums, global const uchar *max_edge_test, global float *filtered)
{
  const uint i = get_global_id(0);

  const uint x = i % 512;
  const uint y = i / 512;

  const float raw_depth = depth[i];
  const float ir_sum = ir_sums[i];
  const uchar edge_test = max_edge_test[i];

  if(raw_depth >= MIN_DEPTH && raw_depth <= MAX_DEPTH)
  {
    if(x < 1 || y < 1 || x > 510 || y > 422)
    {
      filtered[i] = raw_depth;
    }
    else
    {
      float ir_sum_acc = ir_sum;
      float squared_ir_sum_acc = ir_sum * ir_sum;
      float min_depth = raw_depth;
      float max_depth = raw_depth;

      for(int yi = -1; yi < 2; ++yi)
      {
        uint i_other = (y + yi) * 512 + x - 1;

        for(int xi = -1; xi < 2; ++xi, ++i_other)
        {
          if(i_other == i)
          {
            continue;
          }

          const float raw_depth_other = depth[i_other];
          const float ir_sum_other = ir_sums[i_other];

          ir_sum_acc += ir_sum_other;
          squared_ir_sum_acc += ir_sum_other * ir_sum_other;

          if(0.0f < raw_depth_other)
          {
            min_depth = min(min_depth, raw_depth_other);
            max_depth = max(max_depth, raw_depth_other);
          }
        }
      }

      float tmp0 = sqrt(squared_ir_sum_acc * 9.0f - ir_sum_acc * ir_sum_acc) / 9.0f;
      float edge_avg = max(ir_sum_acc / 9.0f, EDGE_AB_AVG_MIN_VALUE);
      tmp0 /= edge_avg;

      float abs_min_diff = fabs(raw_depth - min_depth);
      float abs_max_diff = fabs(raw_depth - max_depth);

      float avg_diff = (abs_min_diff + abs_max_diff) * 0.5f;
      float max_abs_diff = max(abs_min_diff, abs_max_diff);

      bool cond0 =
          0.0f < raw_depth &&
          tmp0 >= EDGE_AB_STD_DEV_THRESHOLD &&
          EDGE_CLOSE_DELTA_THRESHOLD < abs_min_diff &&
          EDGE_FAR_DELTA_THRESHOLD < abs_max_diff &&
          EDGE_MAX_DELTA_THRESHOLD < max_abs_diff &&
          EDGE_AVG_DELTA_THRESHOLD < avg_diff;

      if(!cond0)
      {
        if(edge_test != 0)
        {
          float tmp1 = 1500.0f > raw_depth ? 30.0f : 0.02f * raw_depth;
          float edge_count = 0.0f;

          filtered[i] = edge_count > MAX_EDGE_COUNT ? 0.0f : raw_depth;
        }
        else
        {
          filtered[i] = 0.0f;
        }
      }
      else
      {
        filtered[i] = 0.0f;
      }
    }
  }
  else
  {
    filtered[i] = 0.0f;
  }
}
)";

class OpenCLDepthPacketProcessorImpl: public WithPerfLogging
{
public:
  cl_short lut11to16[2048];
  cl_float x_table[512 * 424];
  cl_float z_table[512 * 424];
  cl_float3 p0_table[512 * 424];
  libfreenect2::DepthPacketProcessor::Config config;
  DepthPacketProcessor::Parameters params;

  Frame *ir_frame, *depth_frame;

  cl::Context context;
  cl::Device device;

  cl::Program program;
  cl::CommandQueue queue;

  cl::Kernel kernel_processPixelStage1;
  cl::Kernel kernel_filterPixelStage1;
  cl::Kernel kernel_processPixelStage2;
  cl::Kernel kernel_filterPixelStage2;

  size_t image_size;

  // Read only buffers
  size_t buf_lut11to16_size;
  size_t buf_p0_table_size;
  size_t buf_x_table_size;
  size_t buf_z_table_size;
  size_t buf_packet_size;

  cl::Buffer buf_lut11to16;
  cl::Buffer buf_p0_table;
  cl::Buffer buf_x_table;
  cl::Buffer buf_z_table;
  cl::Buffer buf_packet;

  // Read-Write buffers
  size_t buf_a_size;
  size_t buf_b_size;
  size_t buf_n_size;
  size_t buf_ir_size;
  size_t buf_a_filtered_size;
  size_t buf_b_filtered_size;
  size_t buf_edge_test_size;
  size_t buf_depth_size;
  size_t buf_ir_sum_size;
  size_t buf_filtered_size;

  cl::Buffer buf_a;
  cl::Buffer buf_b;
  cl::Buffer buf_n;
  cl::Buffer buf_ir;
  cl::Buffer buf_a_filtered;
  cl::Buffer buf_b_filtered;
  cl::Buffer buf_edge_test;
  cl::Buffer buf_depth;
  cl::Buffer buf_ir_sum;
  cl::Buffer buf_filtered;

  bool deviceInitialized;
  bool programBuilt;
  bool programInitialized;
  std::string sourceCode;

  OpenCLDepthPacketProcessorImpl(const int deviceId = -1) 
    : deviceInitialized(false)
    , programBuilt(false)
    , programInitialized(false)
  {
    newIrFrame();
    newDepthFrame();

    image_size = 512 * 424;

    deviceInitialized = initDevice(deviceId);

    const int CL_ICDL_VERSION = 2;
    typedef cl_int (*icdloader_func)(int, size_t, void*, size_t*);
#ifdef _MSC_VER
#pragma warning(push)
#pragma warning(disable : 4996)
#else
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#endif
    icdloader_func clGetICDLoaderInfoOCLICD = (icdloader_func)clGetExtensionFunctionAddress("clGetICDLoaderInfoOCLICD");
#ifdef _MSC_VER
#pragma warning(pop)
#else
#pragma GCC diagnostic pop
#endif
    if (clGetICDLoaderInfoOCLICD != NULL)
    {
      char buf[16];
      if (clGetICDLoaderInfoOCLICD(CL_ICDL_VERSION, sizeof(buf), buf, NULL) == CL_SUCCESS)
      {
        if (strcmp(buf, "2.2.4") < 0)
          LOG_WARNING << "Your ocl-icd has deadlock bugs. Update to 2.2.4+ is recommended.";
      }
    }
  }

  void generateOptions(std::string &options) const
  {
    std::ostringstream oss;
    oss.precision(16);
    oss << std::scientific;
    oss << " -D BFI_BITMASK=" << "0x180";

    oss << " -D AB_MULTIPLIER=" << params.ab_multiplier << "f";
    oss << " -D AB_MULTIPLIER_PER_FRQ0=" << params.ab_multiplier_per_frq[0] << "f";
    oss << " -D AB_MULTIPLIER_PER_FRQ1=" << params.ab_multiplier_per_frq[1] << "f";
    oss << " -D AB_MULTIPLIER_PER_FRQ2=" << params.ab_multiplier_per_frq[2] << "f";
    oss << " -D AB_OUTPUT_MULTIPLIER=" << params.ab_output_multiplier << "f";

    oss << " -D PHASE_IN_RAD0=" << params.phase_in_rad[0] << "f";
    oss << " -D PHASE_IN_RAD1=" << params.phase_in_rad[1] << "f";
    oss << " -D PHASE_IN_RAD2=" << params.phase_in_rad[2] << "f";

    oss << " -D JOINT_BILATERAL_AB_THRESHOLD=" << params.joint_bilateral_ab_threshold << "f";
    oss << " -D JOINT_BILATERAL_MAX_EDGE=" << params.joint_bilateral_max_edge << "f";
    oss << " -D JOINT_BILATERAL_EXP=" << params.joint_bilateral_exp << "f";
    oss << " -D JOINT_BILATERAL_THRESHOLD=" << (params.joint_bilateral_ab_threshold * params.joint_bilateral_ab_threshold) / (params.ab_multiplier * params.ab_multiplier) << "f";
    oss << " -D GAUSSIAN_KERNEL_0=" << params.gaussian_kernel[0] << "f";
    oss << " -D GAUSSIAN_KERNEL_1=" << params.gaussian_kernel[1] << "f";
    oss << " -D GAUSSIAN_KERNEL_2=" << params.gaussian_kernel[2] << "f";
    oss << " -D GAUSSIAN_KERNEL_3=" << params.gaussian_kernel[3] << "f";
    oss << " -D GAUSSIAN_KERNEL_4=" << params.gaussian_kernel[4] << "f";
    oss << " -D GAUSSIAN_KERNEL_5=" << params.gaussian_kernel[5] << "f";
    oss << " -D GAUSSIAN_KERNEL_6=" << params.gaussian_kernel[6] << "f";
    oss << " -D GAUSSIAN_KERNEL_7=" << params.gaussian_kernel[7] << "f";
    oss << " -D GAUSSIAN_KERNEL_8=" << params.gaussian_kernel[8] << "f";

    oss << " -D PHASE_OFFSET=" << params.phase_offset << "f";
    oss << " -D UNAMBIGIOUS_DIST=" << params.unambigious_dist << "f";
    oss << " -D INDIVIDUAL_AB_THRESHOLD=" << params.individual_ab_threshold << "f";
    oss << " -D AB_THRESHOLD=" << params.ab_threshold << "f";
    oss << " -D AB_CONFIDENCE_SLOPE=" << params.ab_confidence_slope << "f";
    oss << " -D AB_CONFIDENCE_OFFSET=" << params.ab_confidence_offset << "f";
    oss << " -D MIN_DEALIAS_CONFIDENCE=" << params.min_dealias_confidence << "f";
    oss << " -D MAX_DEALIAS_CONFIDENCE=" << params.max_dealias_confidence << "f";

    oss << " -D EDGE_AB_AVG_MIN_VALUE=" << params.edge_ab_avg_min_value << "f";
    oss << " -D EDGE_AB_STD_DEV_THRESHOLD=" << params.edge_ab_std_dev_threshold << "f";
    oss << " -D EDGE_CLOSE_DELTA_THRESHOLD=" << params.edge_close_delta_threshold << "f";
    oss << " -D EDGE_FAR_DELTA_THRESHOLD=" << params.edge_far_delta_threshold << "f";
    oss << " -D EDGE_MAX_DELTA_THRESHOLD=" << params.edge_max_delta_threshold << "f";
    oss << " -D EDGE_AVG_DELTA_THRESHOLD=" << params.edge_avg_delta_threshold << "f";
    oss << " -D MAX_EDGE_COUNT=" << params.max_edge_count << "f";

    oss << " -D MIN_DEPTH=" << config.MinDepth * 1000.0f << "f";
    oss << " -D MAX_DEPTH=" << config.MaxDepth * 1000.0f << "f";
    options = oss.str();
  }

  void getDevices(const std::vector<cl::Platform> &platforms, std::vector<cl::Device> &devices)
  {
    devices.clear();
    for(size_t i = 0; i < platforms.size(); ++i)
    {
      const cl::Platform &platform = platforms[i];

      std::vector<cl::Device> devs;
      if(platform.getDevices(CL_DEVICE_TYPE_ALL, &devs) != CL_SUCCESS)
      {
        continue;
      }

      devices.insert(devices.end(), devs.begin(), devs.end());
    }
  }

  std::string deviceString(cl::Device &dev)
  {
    std::string devName, devVendor, devType;
    cl_device_type devTypeID;
    dev.getInfo(CL_DEVICE_NAME, &devName);
    dev.getInfo(CL_DEVICE_VENDOR, &devVendor);
    dev.getInfo(CL_DEVICE_TYPE, &devTypeID);

    switch(devTypeID)
    {
    case CL_DEVICE_TYPE_CPU:
      devType = "CPU";
      break;
    case CL_DEVICE_TYPE_GPU:
      devType = "GPU";
      break;
    case CL_DEVICE_TYPE_ACCELERATOR:
      devType = "ACCELERATOR";
      break;
    default:
      devType = "CUSTOM/UNKNOWN";
    }

    return devName + " (" + devType + ")[" + devVendor + ']';
  }

  void listDevice(std::vector<cl::Device> &devices)
  {
    LOG_INFO << " devices:";
    for(size_t i = 0; i < devices.size(); ++i)
    {
      LOG_INFO << "  " << i << ": " << deviceString(devices[i]);
    }
  }

  bool selectDevice(std::vector<cl::Device> &devices, const int deviceId)
  {
    if(deviceId != -1 && devices.size() > (size_t)deviceId)
    {
      device = devices[deviceId];
      return true;
    }

    bool selected = false;
    size_t selectedType = 0;

    for(size_t i = 0; i < devices.size(); ++i)
    {
      cl::Device &dev = devices[i];
      cl_device_type devTypeID = 0;
      dev.getInfo(CL_DEVICE_TYPE, &devTypeID);

      if(!selected || (selectedType != CL_DEVICE_TYPE_GPU && devTypeID == CL_DEVICE_TYPE_GPU))
      {
        selectedType = devTypeID;
        selected = true;
        device = dev;
      }
    }
    return selected;
  }

#define CHECK_CL_ERROR(err, str) do {if (err != CL_SUCCESS) {LOG_ERROR << str << " failed: " << err; return false; } } while(0)

  bool initDevice(const int deviceId)
  {
    if(!readProgram(sourceCode))
    {
      return false;
    }

    cl_int err = CL_SUCCESS;
    {
      std::vector<cl::Platform> platforms;
      err = cl::Platform::get(&platforms);
      CHECK_CL_ERROR(err, "cl::Platform::get");

      if(platforms.empty())
      {
        LOG_ERROR << "no opencl platforms found.";
        return false;
      }

      std::vector<cl::Device> devices;
      getDevices(platforms, devices);
      listDevice(devices);
      if(!selectDevice(devices, deviceId))
      {
        LOG_ERROR << "could not find any suitable device";
        return false;
      }
      LOG_INFO << "selected device: " << deviceString(device);

      context = cl::Context(device, NULL, NULL, NULL, &err);
      CHECK_CL_ERROR(err, "cl::Context");
    }

    return buildProgram(sourceCode);
  }

  bool initProgram()
  {
    if(!deviceInitialized)
    {
      return false;
    }

    if (!programBuilt)
      if (!buildProgram(sourceCode))
        return false;

    cl_int err = CL_SUCCESS;
    {
      queue = cl::CommandQueue(context, device, 0, &err);
      CHECK_CL_ERROR(err, "cl::CommandQueue");

      //Read only
      buf_lut11to16_size = 2048 * sizeof(cl_short);
      buf_p0_table_size = image_size * sizeof(cl_float3);
      buf_x_table_size = image_size * sizeof(cl_float);
      buf_z_table_size = image_size * sizeof(cl_float);
      buf_packet_size = ((image_size * 11) / 16) * 10 * sizeof(cl_ushort);

      buf_lut11to16 = cl::Buffer(context, CL_READ_ONLY_CACHE, buf_lut11to16_size, NULL, &err);
      CHECK_CL_ERROR(err, "cl::Buffer");
      buf_p0_table = cl::Buffer(context, CL_READ_ONLY_CACHE, buf_p0_table_size, NULL, &err);
      CHECK_CL_ERROR(err, "cl::Buffer");
      buf_x_table = cl::Buffer(context, CL_READ_ONLY_CACHE, buf_x_table_size, NULL, &err);
      CHECK_CL_ERROR(err, "cl::Buffer");
      buf_z_table = cl::Buffer(context, CL_READ_ONLY_CACHE, buf_z_table_size, NULL, &err);
      CHECK_CL_ERROR(err, "cl::Buffer");
      buf_packet = cl::Buffer(context, CL_READ_ONLY_CACHE, buf_packet_size, NULL, &err);
      CHECK_CL_ERROR(err, "cl::Buffer");

      //Read-Write
      buf_a_size = image_size * sizeof(cl_float3);
      buf_b_size = image_size * sizeof(cl_float3);
      buf_n_size = image_size * sizeof(cl_float3);
      buf_ir_size = image_size * sizeof(cl_float);
      buf_a_filtered_size = image_size * sizeof(cl_float3);
      buf_b_filtered_size = image_size * sizeof(cl_float3);
      buf_edge_test_size = image_size * sizeof(cl_uchar);
      buf_depth_size = image_size * sizeof(cl_float);
      buf_ir_sum_size = image_size * sizeof(cl_float);
      buf_filtered_size = image_size * sizeof(cl_float);

      buf_a = cl::Buffer(context, CL_READ_WRITE_CACHE, buf_a_size, NULL, &err);
      CHECK_CL_ERROR(err, "cl::Buffer");
      buf_b = cl::Buffer(context, CL_READ_WRITE_CACHE, buf_b_size, NULL, &err);
      CHECK_CL_ERROR(err, "cl::Buffer");
      buf_n = cl::Buffer(context, CL_READ_WRITE_CACHE, buf_n_size, NULL, &err);
      CHECK_CL_ERROR(err, "cl::Buffer");
      buf_ir = cl::Buffer(context, CL_READ_WRITE_CACHE, buf_ir_size, NULL, &err);
      CHECK_CL_ERROR(err, "cl::Buffer");
      buf_a_filtered = cl::Buffer(context, CL_READ_WRITE_CACHE, buf_a_filtered_size, NULL, &err);
      CHECK_CL_ERROR(err, "cl::Buffer");
      buf_b_filtered = cl::Buffer(context, CL_READ_WRITE_CACHE, buf_b_filtered_size, NULL, &err);
      CHECK_CL_ERROR(err, "cl::Buffer");
      buf_edge_test = cl::Buffer(context, CL_READ_WRITE_CACHE, buf_edge_test_size, NULL, &err);
      CHECK_CL_ERROR(err, "cl::Buffer");
      buf_depth = cl::Buffer(context, CL_READ_WRITE_CACHE, buf_depth_size, NULL, &err);
      CHECK_CL_ERROR(err, "cl::Buffer");
      buf_ir_sum = cl::Buffer(context, CL_READ_WRITE_CACHE, buf_ir_sum_size, NULL, &err);
      CHECK_CL_ERROR(err, "cl::Buffer");
      buf_filtered = cl::Buffer(context, CL_READ_WRITE_CACHE, buf_filtered_size, NULL, &err);
      CHECK_CL_ERROR(err, "cl::Buffer");

      kernel_processPixelStage1 = cl::Kernel(program, "processPixelStage1", &err);
      CHECK_CL_ERROR(err, "cl::Kernel");
      err = kernel_processPixelStage1.setArg(0, buf_lut11to16);
      CHECK_CL_ERROR(err, "setArg");
      err = kernel_processPixelStage1.setArg(1, buf_z_table);
      CHECK_CL_ERROR(err, "setArg");
      err = kernel_processPixelStage1.setArg(2, buf_p0_table);
      CHECK_CL_ERROR(err, "setArg");
      err = kernel_processPixelStage1.setArg(3, buf_packet);
      CHECK_CL_ERROR(err, "setArg");
      err = kernel_processPixelStage1.setArg(4, buf_a);
      CHECK_CL_ERROR(err, "setArg");
      err = kernel_processPixelStage1.setArg(5, buf_b);
      CHECK_CL_ERROR(err, "setArg");
      err = kernel_processPixelStage1.setArg(6, buf_n);
      CHECK_CL_ERROR(err, "setArg");
      err = kernel_processPixelStage1.setArg(7, buf_ir);
      CHECK_CL_ERROR(err, "setArg");

      kernel_filterPixelStage1 = cl::Kernel(program, "filterPixelStage1", &err);
      CHECK_CL_ERROR(err, "cl::Kernel");
      err = kernel_filterPixelStage1.setArg(0, buf_a);
      CHECK_CL_ERROR(err, "setArg");
      err = kernel_filterPixelStage1.setArg(1, buf_b);
      CHECK_CL_ERROR(err, "setArg");
      err = kernel_filterPixelStage1.setArg(2, buf_n);
      CHECK_CL_ERROR(err, "setArg");
      err = kernel_filterPixelStage1.setArg(3, buf_a_filtered);
      CHECK_CL_ERROR(err, "setArg");
      err = kernel_filterPixelStage1.setArg(4, buf_b_filtered);
      CHECK_CL_ERROR(err, "setArg");
      err = kernel_filterPixelStage1.setArg(5, buf_edge_test);
      CHECK_CL_ERROR(err, "setArg");

      kernel_processPixelStage2 = cl::Kernel(program, "processPixelStage2", &err);
      CHECK_CL_ERROR(err, "cl::Kernel");
      err = kernel_processPixelStage2.setArg(0, config.EnableBilateralFilter ? buf_a_filtered : buf_a);
      CHECK_CL_ERROR(err, "setArg");
      err = kernel_processPixelStage2.setArg(1, config.EnableBilateralFilter ? buf_b_filtered : buf_b);
      CHECK_CL_ERROR(err, "setArg");
      err = kernel_processPixelStage2.setArg(2, buf_x_table);
      CHECK_CL_ERROR(err, "setArg");
      err = kernel_processPixelStage2.setArg(3, buf_z_table);
      CHECK_CL_ERROR(err, "setArg");
      err = kernel_processPixelStage2.setArg(4, buf_depth);
      CHECK_CL_ERROR(err, "setArg");
      err = kernel_processPixelStage2.setArg(5, buf_ir_sum);
      CHECK_CL_ERROR(err, "setArg");

      kernel_filterPixelStage2 = cl::Kernel(program, "filterPixelStage2", &err);
      CHECK_CL_ERROR(err, "cl::Kernel");
      err = kernel_filterPixelStage2.setArg(0, buf_depth);
      CHECK_CL_ERROR(err, "setArg");
      err = kernel_filterPixelStage2.setArg(1, buf_ir_sum);
      CHECK_CL_ERROR(err, "setArg");
      err = kernel_filterPixelStage2.setArg(2, buf_edge_test);
      CHECK_CL_ERROR(err, "setArg");
      err = kernel_filterPixelStage2.setArg(3, buf_filtered);
      CHECK_CL_ERROR(err, "setArg");

      cl::Event event0, event1, event2, event3;
      err = queue.enqueueWriteBuffer(buf_lut11to16, CL_FALSE, 0, buf_lut11to16_size, lut11to16, NULL, &event0);
      CHECK_CL_ERROR(err, "enqueueWriteBuffer");
      err = queue.enqueueWriteBuffer(buf_p0_table, CL_FALSE, 0, buf_p0_table_size, p0_table, NULL, &event1);
      CHECK_CL_ERROR(err, "enqueueWriteBuffer");
      err = queue.enqueueWriteBuffer(buf_x_table, CL_FALSE, 0, buf_x_table_size, x_table, NULL, &event2);
      CHECK_CL_ERROR(err, "enqueueWriteBuffer");
      err = queue.enqueueWriteBuffer(buf_z_table, CL_FALSE, 0, buf_z_table_size, z_table, NULL, &event3);
      CHECK_CL_ERROR(err, "enqueueWriteBuffer");

      err = event0.wait();
      CHECK_CL_ERROR(err, "wait");
      err = event1.wait();
      CHECK_CL_ERROR(err, "wait");
      err = event2.wait();
      CHECK_CL_ERROR(err, "wait");
      err = event3.wait();
      CHECK_CL_ERROR(err, "wait");
    }

    programInitialized = true;
    return true;
  }

  bool run(const DepthPacket &packet)
  {
    cl_int err;
    {
      std::vector<cl::Event> eventWrite(1), eventPPS1(1), eventFPS1(1), eventPPS2(1), eventFPS2(1);
      cl::Event event0, event1;

      err = queue.enqueueWriteBuffer(buf_packet, CL_FALSE, 0, buf_packet_size, packet.buffer, NULL, &eventWrite[0]);
      CHECK_CL_ERROR(err, "enqueueWriteBuffer");

      err = queue.enqueueNDRangeKernel(kernel_processPixelStage1, cl::NullRange, cl::NDRange(image_size), cl::NullRange, &eventWrite, &eventPPS1[0]);
      CHECK_CL_ERROR(err, "enqueueNDRangeKernel");
      err = queue.enqueueReadBuffer(buf_ir, CL_FALSE, 0, buf_ir_size, ir_frame->data, &eventPPS1, &event0);
      CHECK_CL_ERROR(err, "enqueueReadBuffer");

      if(config.EnableBilateralFilter)
      {
        err = queue.enqueueNDRangeKernel(kernel_filterPixelStage1, cl::NullRange, cl::NDRange(image_size), cl::NullRange, &eventPPS1, &eventFPS1[0]);
        CHECK_CL_ERROR(err, "enqueueNDRangeKernel");
      }
      else
      {
        eventFPS1[0] = eventPPS1[0];
      }

      err = queue.enqueueNDRangeKernel(kernel_processPixelStage2, cl::NullRange, cl::NDRange(image_size), cl::NullRange, &eventFPS1, &eventPPS2[0]);
      CHECK_CL_ERROR(err, "enqueueNDRangeKernel");

      if(config.EnableEdgeAwareFilter)
      {
        err = queue.enqueueNDRangeKernel(kernel_filterPixelStage2, cl::NullRange, cl::NDRange(image_size), cl::NullRange, &eventPPS2, &eventFPS2[0]);
        CHECK_CL_ERROR(err, "enqueueWriteBuffer");
      }
      else
      {
        eventFPS2[0] = eventPPS2[0];
      }

      err = queue.enqueueReadBuffer(config.EnableEdgeAwareFilter ? buf_filtered : buf_depth, CL_FALSE, 0, buf_depth_size, depth_frame->data, &eventFPS2, &event1);
      CHECK_CL_ERROR(err, "enqueueReadBuffer");
      err = event0.wait();
      CHECK_CL_ERROR(err, "wait");
      err = event1.wait();
      CHECK_CL_ERROR(err, "wait");
    }
    return true;
  }

  bool readProgram(std::string &source) const
  {
    source = clInlineSource;
  
    //source = loadCLSource("opencl_depth_packet_processor.cl");
    return !source.empty();
  }

  bool buildProgram(const std::string& sources)
  {
    cl_int err;
    {
      LOG_INFO << "building OpenCL program...";

      std::string options;
      generateOptions(options);

      cl::Program::Sources source(1, std::make_pair(sources.c_str(), sources.length()));
      program = cl::Program(context, source, &err);
      CHECK_CL_ERROR(err, "cl::Program");

      err = program.build(options.c_str());
      if (err != CL_SUCCESS)
      {
        LOG_ERROR << "failed to build program: " << err;
        LOG_ERROR << "Build Status: " << program.getBuildInfo<CL_PROGRAM_BUILD_STATUS>(device);
        LOG_ERROR << "Build Options:\t" << program.getBuildInfo<CL_PROGRAM_BUILD_OPTIONS>(device);
        LOG_ERROR << "Build Log:\t " << program.getBuildInfo<CL_PROGRAM_BUILD_LOG>(device);
        programBuilt = false;
        return false;
      }
    }

    LOG_INFO << "OpenCL program built successfully";
    programBuilt = true;
    return true;
  }

  void newIrFrame()
  {
    ir_frame = new Frame(512, 424, 4);
  }

  void newDepthFrame()
  {
    depth_frame = new Frame(512, 424, 4);
  }

  void fill_trig_table(const libfreenect2::protocol::P0TablesResponse *p0table)
  {
    for(int r = 0; r < 424; ++r)
    {
      cl_float3 *it = &p0_table[r * 512];
      const uint16_t *it0 = &p0table->p0table0[r * 512];
      const uint16_t *it1 = &p0table->p0table1[r * 512];
      const uint16_t *it2 = &p0table->p0table2[r * 512];
      for(int c = 0; c < 512; ++c, ++it, ++it0, ++it1, ++it2)
      {
        it->s[0] = -((float) * it0) * 0.000031 * M_PI;
        it->s[1] = -((float) * it1) * 0.000031 * M_PI;
        it->s[2] = -((float) * it2) * 0.000031 * M_PI;
        it->s[3] = 0.0f;
      }
    }
  }
};

OpenCLDepthPacketProcessor::OpenCLDepthPacketProcessor(const int deviceId) :
  impl_(new OpenCLDepthPacketProcessorImpl(deviceId))
{
}

OpenCLDepthPacketProcessor::~OpenCLDepthPacketProcessor()
{
  delete impl_;
}

void OpenCLDepthPacketProcessor::setConfiguration(const libfreenect2::DepthPacketProcessor::Config &config)
{
  DepthPacketProcessor::setConfiguration(config);

  if ( impl_->config.MaxDepth != config.MaxDepth 
    || impl_->config.MinDepth != config.MinDepth)
  {
    // OpenCL program needs to be rebuilt, then reinitialized
    impl_->programBuilt = false;
    impl_->programInitialized = false;
  }
  else if (impl_->config.EnableBilateralFilter != config.EnableBilateralFilter
    || impl_->config.EnableEdgeAwareFilter != config.EnableEdgeAwareFilter)
  {
    // OpenCL program only needs to be reinitialized
    impl_->programInitialized = false;
  }

  impl_->config = config;
  if (!impl_->programBuilt)
    impl_->buildProgram(impl_->sourceCode);
}

void OpenCLDepthPacketProcessor::loadP0TablesFromCommandResponse(unsigned char *buffer, size_t buffer_length)
{
  libfreenect2::protocol::P0TablesResponse *p0table = (libfreenect2::protocol::P0TablesResponse *)buffer;

  if(buffer_length < sizeof(libfreenect2::protocol::P0TablesResponse))
  {
    LOG_ERROR << "P0Table response too short!";
    return;
  }

  impl_->fill_trig_table(p0table);
}

void OpenCLDepthPacketProcessor::loadXZTables(const float *xtable, const float *ztable)
{
  std::copy(xtable, xtable + TABLE_SIZE, impl_->x_table);
  std::copy(ztable, ztable + TABLE_SIZE, impl_->z_table);
}

void OpenCLDepthPacketProcessor::loadLookupTable(const short *lut)
{
  std::copy(lut, lut + LUT_SIZE, impl_->lut11to16);
}

void OpenCLDepthPacketProcessor::process(const DepthPacket &packet)
{
  bool has_listener = this->listener_ != 0;

  if(!impl_->programInitialized && !impl_->initProgram())
  {
    LOG_ERROR << "could not initialize OpenCLDepthPacketProcessor";
    return;
  }

  impl_->startTiming();

  impl_->ir_frame->timestamp = packet.timestamp;
  impl_->depth_frame->timestamp = packet.timestamp;
  impl_->ir_frame->sequence = packet.sequence;
  impl_->depth_frame->sequence = packet.sequence;

  bool r = impl_->run(packet);

  impl_->stopTiming(LOG_INFO);

  if(has_listener && r)
  {
    if(this->listener_->onNewFrame(Frame::Ir, impl_->ir_frame))
    {
      impl_->newIrFrame();
    }

    if(this->listener_->onNewFrame(Frame::Depth, impl_->depth_frame))
    {
      impl_->newDepthFrame();
    }
  }
}

} /* namespace libfreenect2 */

