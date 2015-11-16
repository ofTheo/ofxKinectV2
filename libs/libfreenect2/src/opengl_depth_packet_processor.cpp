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

/** @file opengl_depth_packet_processor.cpp Depth packet processor implementation using OpenGL. */


#include <libfreenect2/depth_packet_processor.h>
#include <libfreenect2/resource.h>
#include <libfreenect2/protocol/response.h>
#include <libfreenect2/logging.h>
#include "flextGL.h"
#include <GLFW/glfw3.h>

#include <fstream>
#include <string>
#include <map>
#include <cstdlib>

#include <stdint.h>

#define CHECKGL() do { \
for (GLenum glerror = glGetError(); glerror != GL_NO_ERROR; glerror = glGetError()) \
  LOG_ERROR << "line " << __LINE__ << ": GL error " << glerror; \
} while(0)

namespace libfreenect2
{

static const std::string debugfs = R"(

    uniform sampler2DRect Data;

    in vec2 TexCoord;

    out vec4 Color;

    void main(void)
    {
      ivec2 uv = ivec2(TexCoord.x, TexCoord.y);
      
      Color = texelFetch(Data, uv);
    }
)";


static const std::string defaultvs = R"(
    in vec2 InputPosition;
    in vec2 InputTexCoord;

    out vec2 TexCoord;

    void main(void)
    {
      gl_Position = vec4(InputPosition, 0.0, 1.0);
      TexCoord = InputTexCoord;
    }
)";


static const std::string filter1fs = R"(
    struct Parameters
    {
      float ab_multiplier;
      vec3 ab_multiplier_per_frq;
      float ab_output_multiplier;
      
      vec3 phase_in_rad;
      
      float joint_bilateral_ab_threshold;
      float joint_bilateral_max_edge;
      float joint_bilateral_exp;
      mat3 gaussian_kernel;
      
      float phase_offset;
      float unambigious_dist;
      float individual_ab_threshold;
      float ab_threshold;
      float ab_confidence_slope;
      float ab_confidence_offset;
      float min_dealias_confidence;
      float max_dealias_confidence;
      
      float edge_ab_avg_min_value;
      float edge_ab_std_dev_threshold;
      float edge_close_delta_threshold;
      float edge_far_delta_threshold;
      float edge_max_delta_threshold;
      float edge_avg_delta_threshold;
      float max_edge_count;
      
      float min_depth;
      float max_depth;
    };

    uniform sampler2DRect A;
    uniform sampler2DRect B;
    uniform sampler2DRect Norm;

    uniform Parameters Params;

    in vec2 TexCoord;

    /*layout(location = 0)*/ out vec4 Debug;
    /*layout(location = 1)*/ out vec3 FilterA;
    /*layout(location = 2)*/ out vec3 FilterB;
    /*layout(location = 3)*/ out uint MaxEdgeTest;

    void applyBilateralFilter(ivec2 uv)
    {
      vec3 threshold = vec3((Params.joint_bilateral_ab_threshold * Params.joint_bilateral_ab_threshold) / (Params.ab_multiplier * Params.ab_multiplier));
      vec3 joint_bilateral_exp = vec3(Params.joint_bilateral_exp);
      
      vec3 self_a = texelFetch(A, uv).xyz;
      vec3 self_b = texelFetch(B, uv).xyz;
      vec3 self_norm = texelFetch(Norm, uv).xyz;
      vec3 self_normalized_a = self_a / self_norm;
      vec3 self_normalized_b = self_b / self_norm;
      
      vec4 weight_acc = vec4(0.0);
      vec4 weighted_a_acc = vec4(0.0);
      vec4 weighted_b_acc = vec4(0.0);
      
      bvec3 c0 = lessThan(self_norm * self_norm, threshold);
      
      threshold = mix(threshold, vec3(0.0), c0);
      joint_bilateral_exp = mix(joint_bilateral_exp, vec3(0.0), c0);
      
      for(int y = 0; y < 3; ++y)
      {
        for(int x = 0; x < 3; ++x)
        {
          ivec2 ouv = uv + ivec2(x - 1, y - 1);
        
          vec3 other_a = texelFetch(A, ouv).xyz;
          vec3 other_b = texelFetch(B, ouv).xyz;
          vec3 other_norm = texelFetch(Norm, ouv).xyz;
          
          vec3 other_normalized_a = other_a / other_norm;
          vec3 other_normalized_b = other_b / other_norm;
                
          bvec3 c1 = lessThan(other_norm * other_norm, threshold);
          
          vec3 dist = 0.5f * (1.0f - (self_normalized_a * other_normalized_a + self_normalized_b * other_normalized_b));
          vec3 weight = mix(Params.gaussian_kernel[x][y] * exp(-1.442695 * joint_bilateral_exp * dist), vec3(0.0), c1);
          
          weighted_a_acc.xyz += weight * other_a;
          weighted_b_acc.xyz += weight * other_b;
          weight_acc.xyz += weight;
          
          // TODO: this sucks, but otherwise opengl reports error: temporary registers exceeded :(
          weighted_a_acc.w += mix(dist.x, 0, c1.x);
          weighted_b_acc.w += mix(dist.y, 0, c1.y);
          weight_acc.w += mix(dist.z, 0, c1.z);
        }
      }
      
      bvec3 c2 = lessThan(vec3(0.0), weight_acc.xyz);
      FilterA = mix(vec3(0.0), weighted_a_acc.xyz / weight_acc.xyz, c2);
      FilterB = mix(vec3(0.0), weighted_b_acc.xyz / weight_acc.xyz, c2);
      
      if(uv.x < 1 || uv.y < 1 || uv.x > 510 || uv.y > 422)
      {
        FilterA = self_a;
        FilterB = self_b;
      }
      
      vec3 dist_acc = vec3(weighted_a_acc.w, weighted_b_acc.w, weight_acc.w);
      MaxEdgeTest = uint(all(lessThan(dist_acc, vec3(Params.joint_bilateral_max_edge))));
      //Debug = vec4(vec3(MaxEdgeTest), 1);
    }

    void main(void)
    {
      ivec2 uv = ivec2(TexCoord.x, TexCoord.y);
        
      applyBilateralFilter(uv);
      
      vec3 norm = sqrt(FilterA * FilterA + FilterB * FilterB);
      float i = min(dot(norm, vec3(0.333333333  * Params.ab_multiplier * Params.ab_output_multiplier)), 65535.0);
      
      Debug = vec4(vec3(i, i, i) / 65535.0, 1);
    }

)";

static const std::string filter2fs = R"(

    struct Parameters
    {
      float ab_multiplier;
      vec3 ab_multiplier_per_frq;
      float ab_output_multiplier;
      
      vec3 phase_in_rad;
      
      float joint_bilateral_ab_threshold;
      float joint_bilateral_max_edge;
      float joint_bilateral_exp;
      mat3 gaussian_kernel;
      
      float phase_offset;
      float unambigious_dist;
      float individual_ab_threshold;
      float ab_threshold;
      float ab_confidence_slope;
      float ab_confidence_offset;
      float min_dealias_confidence;
      float max_dealias_confidence;
      
      float edge_ab_avg_min_value;
      float edge_ab_std_dev_threshold;
      float edge_close_delta_threshold;
      float edge_far_delta_threshold;
      float edge_max_delta_threshold;
      float edge_avg_delta_threshold;
      float max_edge_count;
      
      float min_depth;
      float max_depth;
    };

    uniform sampler2DRect DepthAndIrSum;
    uniform usampler2DRect MaxEdgeTest;

    uniform Parameters Params;

    in vec2 TexCoord;

    /*layout(location = 0)*/ out vec4 Debug;
    /*layout(location = 1)*/ out float FilterDepth;

    void applyEdgeAwareFilter(ivec2 uv)
    {
      vec2 v = texelFetch(DepthAndIrSum, uv).xy;
      
      if(v.x >= Params.min_depth && v.x <= Params.max_depth)
      {
        if(uv.x < 1 || uv.y < 1 || uv.x > 510 || uv.y > 422)
        {
          FilterDepth = v.x;
        }
        else
        {
          bool max_edge_test_ok = texelFetch(MaxEdgeTest, uv).x > 0u;
          
          float ir_sum_acc = v.y, squared_ir_sum_acc = v.y * v.y, min_depth = v.x, max_depth = v.x;

          for(int yi = -1; yi < 2; ++yi)
          {
            for(int xi = -1; xi < 2; ++xi)
            {
              if(yi == 0 && xi == 0) continue;

              vec2 other = texelFetch(DepthAndIrSum, uv + ivec2(xi, yi)).xy;

              ir_sum_acc += other.y;
              squared_ir_sum_acc += other.y * other.y;

              if(0.0f < other.x)
              {
                min_depth = min(min_depth, other.x);
                max_depth = max(max_depth, other.x);
              }
            }
          }

          float tmp0 = sqrt(squared_ir_sum_acc * 9.0f - ir_sum_acc * ir_sum_acc) / 9.0f;
          float edge_avg = max(ir_sum_acc / 9.0f, Params.edge_ab_avg_min_value);
          tmp0 /= edge_avg;

          float abs_min_diff = abs(v.x - min_depth);
          float abs_max_diff = abs(v.x - max_depth);

          float avg_diff = (abs_min_diff + abs_max_diff) * 0.5f;
          float max_abs_diff = max(abs_min_diff, abs_max_diff);

          bool cond0 =
              0.0f < v.x &&
              tmp0 >= Params.edge_ab_std_dev_threshold &&
              Params.edge_close_delta_threshold < abs_min_diff &&
              Params.edge_far_delta_threshold < abs_max_diff &&
              Params.edge_max_delta_threshold < max_abs_diff &&
              Params.edge_avg_delta_threshold < avg_diff;

          FilterDepth = cond0 ? 0.0f : v.x;

          if(!cond0)
          {
            if(max_edge_test_ok)
            {
              float tmp1 = 1500.0f > v.x ? 30.0f : 0.02f * v.x;
              float edge_count = 0.0f;

              FilterDepth = edge_count > Params.max_edge_count ? 0.0f : v.x;
            }
            else
            {
              FilterDepth = !max_edge_test_ok ? 0.0f : v.x;
              //FilterDepth = true ? FilterDepth : v.x;
            }
          }
        }
      }
      else
      {
        FilterDepth = 0.0f;
      }
    }

    void main(void)
    {
      ivec2 uv = ivec2(TexCoord.x, TexCoord.y);
      
      applyEdgeAwareFilter(uv);
      
      Debug = vec4(vec3(FilterDepth / Params.max_depth), 1);
    }

)";

static const std::string stage1fs = R"(

        struct Parameters
    {
      float ab_multiplier;
      vec3 ab_multiplier_per_frq;
      float ab_output_multiplier;
      
      vec3 phase_in_rad;
      
      float joint_bilateral_ab_threshold;
      float joint_bilateral_max_edge;
      float joint_bilateral_exp;
      mat3 gaussian_kernel;
      
      float phase_offset;
      float unambigious_dist;
      float individual_ab_threshold;
      float ab_threshold;
      float ab_confidence_slope;
      float ab_confidence_offset;
      float min_dealias_confidence;
      float max_dealias_confidence;
      
      float edge_ab_avg_min_value;
      float edge_ab_std_dev_threshold;
      float edge_close_delta_threshold;
      float edge_far_delta_threshold;
      float edge_max_delta_threshold;
      float edge_avg_delta_threshold;
      float max_edge_count;
      
      float min_depth;
      float max_depth;
    };

    uniform usampler2DRect P0Table0;
    uniform usampler2DRect P0Table1;
    uniform usampler2DRect P0Table2;
    uniform isampler2DRect Lut11to16;
    uniform usampler2DRect Data;
    uniform sampler2DRect ZTable;

    uniform Parameters Params;

    in vec2 TexCoord;

    /*layout(location = 0)*/ out vec4 Debug;
    /*                    */
    /*layout(location = 1)*/ out vec3 A;
    /*layout(location = 2)*/ out vec3 B;
    /*layout(location = 3)*/ out vec3 Norm;
    /*layout(location = 4)*/ out float Infrared;

    #define M_PI 3.1415926535897932384626433832795

    int data(ivec2 uv)
    {
      return int(texelFetch(Data, uv).x);
    }

    float decode_data(ivec2 uv, int sub)
    {
      int row_idx = 424 * sub + (uv.y < 212 ? uv.y + 212 : 423 - uv.y);

      int m = int(0xffffffff);
      int bitmask = (((1 << 2) - 1) << 7) & m;
      int idx = (((uv.x >> 2) + ((uv.x << 7) & bitmask)) * 11) & m;

      int col_idx = idx >> 4;
      int upper_bytes = idx & 15;
      int lower_bytes = 16 - upper_bytes;

      ivec2 data_idx0 = ivec2(col_idx, row_idx);
      ivec2 data_idx1 = ivec2(col_idx + 1, row_idx);

      int lut_idx = (uv.x < 1 || 510 < uv.x || col_idx > 352) ? 0 : ((data(data_idx0) >> upper_bytes) | (data(data_idx1) << lower_bytes)) & 2047;
      
      return float(texelFetch(Lut11to16, ivec2(int(lut_idx), 0)).x);
    }

    vec2 processMeasurementTriple(in ivec2 uv, in usampler2DRect p0table, in int offset, in float ab_multiplier_per_frq, inout bool saturated)
    {
      float p0 = -float(texelFetch(p0table, uv).x) * 0.000031 * M_PI;
      
      vec3 v = vec3(decode_data(uv, offset + 0), decode_data(uv, offset + 1), decode_data(uv, offset + 2));
      
      saturated = saturated && any(equal(v, vec3(32767.0)));
      
      float a = dot(v, cos( p0 + Params.phase_in_rad)) * ab_multiplier_per_frq;
      float b = dot(v, sin(-p0 - Params.phase_in_rad)) * ab_multiplier_per_frq;
      
      return vec2(a, b);
    }

    void main(void)
    {
      ivec2 uv = ivec2(TexCoord.x, TexCoord.y);
        
      bool valid_pixel = 0.0 < texelFetch(ZTable, uv).x;
      bvec3 saturated = bvec3(valid_pixel);
      
      vec2 ab0 = processMeasurementTriple(uv, P0Table0, 0, Params.ab_multiplier_per_frq.x, saturated.x);
      vec2 ab1 = processMeasurementTriple(uv, P0Table1, 3, Params.ab_multiplier_per_frq.y, saturated.y);
      vec2 ab2 = processMeasurementTriple(uv, P0Table2, 6, Params.ab_multiplier_per_frq.z, saturated.z);
      
    #ifdef MESA_BUGGY_BOOL_CMP
      valid_pixel = valid_pixel ? true : false;
    #endif
      bvec3 invalid_pixel = bvec3(!valid_pixel);
      
      A    = mix(vec3(ab0.x, ab1.x, ab2.x), vec3(0.0), invalid_pixel);
      B    = mix(vec3(ab0.y, ab1.y, ab2.y), vec3(0.0), invalid_pixel);
      Norm = sqrt(A * A + B * B);
      
      A = mix(A, vec3(0.0), saturated);
      B = mix(B, vec3(0.0), saturated);
      
      Infrared = min(dot(mix(Norm, vec3(65535.0), saturated), vec3(0.333333333  * Params.ab_multiplier * Params.ab_output_multiplier)), 65535.0);
      
      Debug = vec4(sqrt(vec3(Infrared / 65535.0)), 1.0);
    }


)";

static std::string stage2fs = R"(
    
        struct Parameters
    {
      float ab_multiplier;
      vec3 ab_multiplier_per_frq;
      float ab_output_multiplier;
      
      vec3 phase_in_rad;
      
      float joint_bilateral_ab_threshold;
      float joint_bilateral_max_edge;
      float joint_bilateral_exp;
      mat3 gaussian_kernel;
      
      float phase_offset;
      float unambigious_dist;
      float individual_ab_threshold;
      float ab_threshold;
      float ab_confidence_slope;
      float ab_confidence_offset;
      float min_dealias_confidence;
      float max_dealias_confidence;
      
      float edge_ab_avg_min_value;
      float edge_ab_std_dev_threshold;
      float edge_close_delta_threshold;
      float edge_far_delta_threshold;
      float edge_max_delta_threshold;
      float edge_avg_delta_threshold;
      float max_edge_count;
      
      float min_depth;
      float max_depth;
    };

    uniform sampler2DRect A;
    uniform sampler2DRect B;
    uniform sampler2DRect XTable;
    uniform sampler2DRect ZTable;

    uniform Parameters Params;

    in vec2 TexCoord;

    /*layout(location = 0)*/ out vec4 Debug;
    /*layout(location = 1)*/ out float Depth;
    /*layout(location = 2)*/ out vec2 DepthAndIrSum;

    #define M_PI 3.1415926535897932384626433832795

    void main(void)
    {
      ivec2 uv = ivec2(TexCoord.x, TexCoord.y);
          
      vec3 a = texelFetch(A, uv).xyz;
      vec3 b = texelFetch(B, uv).xyz;
      
      vec3 phase = atan(b, a);
      phase = mix(phase, phase + 2.0 * M_PI, lessThan(phase, vec3(0.0)));
      phase = mix(phase, vec3(0.0), isnan(phase));
      vec3 ir = sqrt(a * a + b * b) * Params.ab_multiplier;
      
      float ir_sum = ir.x + ir.y + ir.z;
      float ir_min = min(ir.x, min(ir.y, ir.z));
      float ir_max = max(ir.x, max(ir.y, ir.z));
      
      float phase_final = 0;
      
      if(ir_min >= Params.individual_ab_threshold && ir_sum >= Params.ab_threshold)
      {
        vec3 t = phase / (2.0 * M_PI) * vec3(3.0, 15.0, 2.0);
      
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

        bool c2 = 0.5f < abs(t3) && abs(t3) < 1.5f;

        float t6 = c2 ? t5 + 15.0f : t5;
        float t7 = c2 ? t1 + 15.0f : t1;

        float t8 = (floor((-t2 + t6) * 0.5f + 0.5f) * 2.0f + t2) * 0.5f;

        t6 *= 0.333333f; // = / 3
        t7 *= 0.066667f; // = / 15

        float t9 = (t8 + t6 + t7); // transformed phase measurements (they are transformed and divided by the values the original values were multiplied with)
        float t10 = t9 * 0.333333f; // some avg

        t6 *= 2.0f * M_PI;
        t7 *= 2.0f * M_PI;
        t8 *= 2.0f * M_PI;

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

        bool slope_positive = 0 < Params.ab_confidence_slope;

        float ir_x = slope_positive ? ir_min : ir_max;

        ir_x = log(ir_x);
        ir_x = (ir_x * Params.ab_confidence_slope * 0.301030f + Params.ab_confidence_offset) * 3.321928f;
        ir_x = exp(ir_x);
        ir_x = min(Params.max_dealias_confidence, max(Params.min_dealias_confidence, ir_x));
        ir_x *= ir_x;

        float mask2 = ir_x >= norm ? 1.0f : 0.0f;

        float t11 = t10 * mask2;

        float mask3 = Params.max_dealias_confidence * Params.max_dealias_confidence >= norm ? 1.0f : 0.0f;
        t10 *= mask3;
        phase_final = true/*(modeMask & 2) != 0*/ ? t11 : t10;
      }
      
      float zmultiplier = texelFetch(ZTable, uv).x;
      float xmultiplier = texelFetch(XTable, uv).x;

      phase_final = 0.0 < phase_final ? phase_final + Params.phase_offset : phase_final;

      float depth_linear = zmultiplier * phase_final;
      float max_depth = phase_final * Params.unambigious_dist * 2.0;

      bool cond1 = /*(modeMask & 32) != 0*/ true && 0.0 < depth_linear && 0.0 < max_depth;

      xmultiplier = (xmultiplier * 90.0) / (max_depth * max_depth * 8192.0);

      float depth_fit = depth_linear / (-depth_linear * xmultiplier + 1);
      depth_fit = depth_fit < 0.0 ? 0.0 : depth_fit;
      
      Depth = cond1 ? depth_fit : depth_linear; // r1.y -> later r2.z
      DepthAndIrSum = vec2(Depth, ir_sum);
      
      Debug = vec4(vec3(Depth / Params.max_depth), 1.0);
    }

)";


struct ChangeCurrentOpenGLContext
{
  GLFWwindow *last_ctx;

  ChangeCurrentOpenGLContext(GLFWwindow *new_context);
  ~ChangeCurrentOpenGLContext();
};

ChangeCurrentOpenGLContext::ChangeCurrentOpenGLContext(GLFWwindow *new_context)
{
  last_ctx = glfwGetCurrentContext();
  glfwMakeContextCurrent(new_context);
}

ChangeCurrentOpenGLContext::~ChangeCurrentOpenGLContext()
{
  //LOG_INFO << "restoring context!";
  if(last_ctx != 0)
  {
    glfwMakeContextCurrent(last_ctx);
  }
  else
  {
    glfwMakeContextCurrent(0);
  }
}

class WithOpenGLBindings
{
private:
  OpenGLBindings *bindings;
protected:
  WithOpenGLBindings() : bindings(0) {}
  virtual ~WithOpenGLBindings() {}
  
  virtual void onOpenGLBindingsChanged(OpenGLBindings *b) { }
public:
  void gl(OpenGLBindings *bindings)
  {
    this->bindings = bindings;
    onOpenGLBindingsChanged(this->bindings);
  }
  
  OpenGLBindings *gl()
  {
    return bindings;
  }
};

std::string loadShaderSource(const std::string& filename)
{
  const unsigned char* data;
  size_t length = 0;

  if(!loadResource(filename, &data, &length))
  {
    LOG_ERROR << "failed to load shader source!";
    return "";
  }

  return std::string(reinterpret_cast<const char*>(data), length);
}

struct ShaderProgram : public WithOpenGLBindings
{
  typedef std::map<std::string, int> FragDataMap;
  FragDataMap frag_data_map_;
  GLuint program, vertex_shader, fragment_shader;

  char error_buffer[2048];

  std::string defines;
  bool is_mesa_checked;

  ShaderProgram() :
    program(0),
    is_mesa_checked(false),
    vertex_shader(0),
    fragment_shader(0)
  {
  }

  void checkMesaBug()
  {
    if (is_mesa_checked)
      return;
    is_mesa_checked = true;
    std::string ren((const char*)glGetString(GL_RENDERER));
    std::string ver((const char*)glGetString(GL_VERSION));
    if (ren.find("Mesa DRI Intel") == 0)
    {
      size_t mesa_pos = ver.rfind("Mesa ");
      if (mesa_pos != std::string::npos)
      {
        double mesa_ver = atof(ver.substr(mesa_pos + 5).c_str());
        if (mesa_ver < 10.3)
        {
          defines += "#define MESA_BUGGY_BOOL_CMP\n";
          LOG_WARNING << "Working around buggy boolean instructions in your Mesa driver. Mesa DRI 10.3+ is recommended.";
        }
      }
    }
  }

  void setVertexShader(const std::string& src)
  {
    checkMesaBug();
    const GLchar *sources[] = {"#version 140\n", defines.c_str(), src.c_str()};
    vertex_shader = gl()->glCreateShader(GL_VERTEX_SHADER);
    gl()->glShaderSource(vertex_shader, 3, sources, NULL);
    CHECKGL();
  }

  void setFragmentShader(const std::string& src)
  {
    checkMesaBug();
    const GLchar *sources[] = {"#version 140\n", defines.c_str(), src.c_str()};
    fragment_shader = gl()->glCreateShader(GL_FRAGMENT_SHADER);
    gl()->glShaderSource(fragment_shader, 3, sources, NULL);
    CHECKGL();
  }

  void bindFragDataLocation(const std::string &name, int output)
  {
    frag_data_map_[name] = output;
  }

  void build()
  {
    GLint status;

    gl()->glCompileShader(vertex_shader);
    gl()->glGetShaderiv(vertex_shader, GL_COMPILE_STATUS, &status);

    if(status != GL_TRUE)
    {
      gl()->glGetShaderInfoLog(vertex_shader, sizeof(error_buffer), NULL, error_buffer);

      LOG_ERROR << "failed to compile vertex shader!" << std::endl << error_buffer;
    }

    gl()->glCompileShader(fragment_shader);

    gl()->glGetShaderiv(fragment_shader, GL_COMPILE_STATUS, &status);
    if(status != GL_TRUE)
    {
      gl()->glGetShaderInfoLog(fragment_shader, sizeof(error_buffer), NULL, error_buffer);

      LOG_ERROR << "failed to compile fragment shader!" << std::endl << error_buffer;
    }

    program = gl()->glCreateProgram();
    gl()->glAttachShader(program, vertex_shader);
    gl()->glAttachShader(program, fragment_shader);

    for(FragDataMap::iterator it = frag_data_map_.begin(); it != frag_data_map_.end(); ++it)
    {
      gl()->glBindFragDataLocation(program, it->second, it->first.c_str());
    }

    gl()->glLinkProgram(program);

    gl()->glGetProgramiv(program, GL_LINK_STATUS, &status);

    if(status != GL_TRUE)
    {
      gl()->glGetProgramInfoLog(program, sizeof(error_buffer), NULL, error_buffer);
      LOG_ERROR << "failed to link shader program!" << std::endl << error_buffer;
    }
    CHECKGL();
  }

  GLint getAttributeLocation(const std::string& name)
  {
    return gl()->glGetAttribLocation(program, name.c_str());
  }

  void setUniform(const std::string& name, GLint value)
  {
    GLint idx = gl()->glGetUniformLocation(program, name.c_str());
    if(idx == -1) return;

    gl()->glUniform1i(idx, value);
    CHECKGL();
  }

  void setUniform(const std::string& name, GLfloat value)
  {
    GLint idx = gl()->glGetUniformLocation(program, name.c_str());
    if(idx == -1) return;

    gl()->glUniform1f(idx, value);
    CHECKGL();
  }

  void setUniformVector3(const std::string& name, GLfloat value[3])
  {
    GLint idx = gl()->glGetUniformLocation(program, name.c_str());
    if(idx == -1) return;

    gl()->glUniform3fv(idx, 1, value);
    CHECKGL();
  }

  void setUniformMatrix3(const std::string& name, GLfloat value[9])
  {
    GLint idx = gl()->glGetUniformLocation(program, name.c_str());
    if(idx == -1) return;

    gl()->glUniformMatrix3fv(idx, 1, false, value);
    CHECKGL();
  }

  void use()
  {
    gl()->glUseProgram(program);
    CHECKGL();
  }
};

template<size_t TBytesPerPixel, GLenum TInternalFormat, GLenum TFormat, GLenum TType>
struct ImageFormat
{
  static const size_t BytesPerPixel = TBytesPerPixel;
  static const GLenum InternalFormat = TInternalFormat;
  static const GLenum Format = TFormat;
  static const GLenum Type = TType;
};

typedef ImageFormat<1, GL_R8UI, GL_RED_INTEGER, GL_UNSIGNED_BYTE> U8C1;
typedef ImageFormat<2, GL_R16I, GL_RED_INTEGER, GL_SHORT> S16C1;
typedef ImageFormat<2, GL_R16UI, GL_RED_INTEGER, GL_UNSIGNED_SHORT> U16C1;
typedef ImageFormat<4, GL_R32F, GL_RED, GL_FLOAT> F32C1;
typedef ImageFormat<8, GL_RG32F, GL_RG, GL_FLOAT> F32C2;
typedef ImageFormat<12, GL_RGB32F, GL_RGB, GL_FLOAT> F32C3;
typedef ImageFormat<16, GL_RGBA32F, GL_RGBA, GL_FLOAT> F32C4;

template<typename FormatT>
struct Texture : public WithOpenGLBindings
{
protected:
  size_t bytes_per_pixel, height, width;

public:
  GLuint texture;
  unsigned char *data;
  size_t size;

  Texture() : texture(0), data(0), size(0), bytes_per_pixel(FormatT::BytesPerPixel), height(0), width(0)
  {
  }

  void bindToUnit(GLenum unit)
  {
    gl()->glActiveTexture(unit);
    glBindTexture(GL_TEXTURE_RECTANGLE, texture);
    CHECKGL();
  }

  void allocate(size_t new_width, size_t new_height)
  {
    if (size)
      return;

    GLint max_size;
    glGetIntegerv(GL_MAX_RECTANGLE_TEXTURE_SIZE, &max_size);
    if (new_width > max_size || new_height > max_size)
    {
      LOG_ERROR << "GL_MAX_RECTANGLE_TEXTURE_SIZE is too small: " << max_size;
      exit(-1);
    }

    width = new_width;
    height = new_height;
    size = height * width * bytes_per_pixel;
    data = new unsigned char[size];

    glGenTextures(1, &texture);
    bindToUnit(GL_TEXTURE0);
    glTexParameteri(GL_TEXTURE_RECTANGLE, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_BORDER);
    glTexParameteri(GL_TEXTURE_RECTANGLE, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_BORDER);
    glTexParameteri(GL_TEXTURE_RECTANGLE, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_RECTANGLE, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexImage2D(GL_TEXTURE_RECTANGLE, 0, FormatT::InternalFormat, width, height, 0, FormatT::Format, FormatT::Type, 0);
    CHECKGL();
  }

  void upload()
  {
    bindToUnit(GL_TEXTURE0);
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    glTexSubImage2D(GL_TEXTURE_RECTANGLE, /*level*/0, /*xoffset*/0, /*yoffset*/0, width, height, FormatT::Format, FormatT::Type, data);
    CHECKGL();
  }

  void download()
  {
    downloadToBuffer(data);
  }

  void downloadToBuffer(unsigned char *data)
  {
    glReadPixels(0, 0, width, height, FormatT::Format, FormatT::Type, data);
    CHECKGL();
  }

  void flipY()
  {
    flipYBuffer(data);
  }

  void flipYBuffer(unsigned char *data)
  {
    typedef unsigned char type;

    int linestep = width * bytes_per_pixel / sizeof(type);

    type *first_line = reinterpret_cast<type *>(data), *last_line = reinterpret_cast<type *>(data) + (height - 1) * linestep;

    for(int y = 0; y < height / 2; ++y)
    {
      for(int x = 0; x < linestep; ++x, ++first_line, ++last_line)
      {
        std::swap(*first_line, *last_line);
      }
      last_line -= 2 * linestep;
    }
  }

  Frame *downloadToNewFrame()
  {
    Frame *f = new Frame(width, height, bytes_per_pixel);
    downloadToBuffer(f->data);
    flipYBuffer(f->data);

    return f;
  }
};

class OpenGLDepthPacketProcessorImpl : public WithOpenGLBindings, public WithPerfLogging
{
public:
  GLFWwindow *opengl_context_ptr;
  libfreenect2::DepthPacketProcessor::Config config;

  GLuint square_vbo, square_vao, stage1_framebuffer, filter1_framebuffer, stage2_framebuffer, filter2_framebuffer;
  Texture<S16C1> lut11to16;
  Texture<U16C1> p0table[3];
  Texture<F32C1> x_table, z_table;

  Texture<U16C1> input_data;

  Texture<F32C4> stage1_debug;
  Texture<F32C4> stage1_data[3];
  Texture<F32C1> stage1_infrared;

  Texture<F32C4> filter1_data[2];
  Texture<U8C1> filter1_max_edge_test;
  Texture<F32C4> filter1_debug;

  Texture<F32C4> stage2_debug;

  Texture<F32C1> stage2_depth;
  Texture<F32C2> stage2_depth_and_ir_sum;

  Texture<F32C4> filter2_debug;
  Texture<F32C1> filter2_depth;

  ShaderProgram stage1, filter1, stage2, filter2, debug;

  DepthPacketProcessor::Parameters params;
  bool params_need_update;

  bool do_debug;

  struct Vertex
  {
    float x, y;
    float u, v;
  };

  OpenGLDepthPacketProcessorImpl(GLFWwindow *new_opengl_context_ptr, bool debug) :
    opengl_context_ptr(new_opengl_context_ptr),
    square_vao(0),
    square_vbo(0),
    stage1_framebuffer(0),
    filter1_framebuffer(0),
    stage2_framebuffer(0),
    filter2_framebuffer(0),
    params_need_update(true),
    do_debug(debug)
  {
  }

  virtual ~OpenGLDepthPacketProcessorImpl()
  {
    if(gl() != 0)
    {
      delete gl();
      gl(0);
    }
    glfwDestroyWindow(opengl_context_ptr);
    opengl_context_ptr = 0;
  }
  
  virtual void onOpenGLBindingsChanged(OpenGLBindings *b) 
  {
    lut11to16.gl(b);
    p0table[0].gl(b);
    p0table[1].gl(b);
    p0table[2].gl(b);
    x_table.gl(b);
    z_table.gl(b);

    input_data.gl(b);

    stage1_debug.gl(b);
    stage1_data[0].gl(b);
    stage1_data[1].gl(b);
    stage1_data[2].gl(b);
    stage1_infrared.gl(b);

    filter1_data[0].gl(b);
    filter1_data[1].gl(b);
    filter1_max_edge_test.gl(b);
    filter1_debug.gl(b);

    stage2_debug.gl(b);

    stage2_depth.gl(b);
    stage2_depth_and_ir_sum.gl(b);

    filter2_debug.gl(b);
    filter2_depth.gl(b);
 
    stage1.gl(b);
    filter1.gl(b);
    stage2.gl(b);
    filter2.gl(b);
    debug.gl(b);
  }

  static void glfwErrorCallback(int error, const char* description)
  {
    LOG_ERROR << "GLFW error " << error << " " << description;
  }

  void checkFBO(GLenum target)
  {
    GLenum status = gl()->glCheckFramebufferStatus(target);
    if (status != GL_FRAMEBUFFER_COMPLETE)
    {
      LOG_ERROR << "incomplete FBO " << status;
      exit(-1);
    }
    CHECKGL();
  }

  void initialize()
  {
    ChangeCurrentOpenGLContext ctx(opengl_context_ptr);
    
    int major = glfwGetWindowAttrib(opengl_context_ptr, GLFW_CONTEXT_VERSION_MAJOR);
    int minor = glfwGetWindowAttrib(opengl_context_ptr, GLFW_CONTEXT_VERSION_MINOR);

    if (major * 10 + minor < 31) {
        LOG_ERROR << "OpenGL version 3.1 not supported.";
        LOG_ERROR << "Your version is " << major << "." << minor;
        LOG_ERROR << "Try updating your graphics driver.";
        exit(-1);
    }

    OpenGLBindings *b = new OpenGLBindings();
    flextInit(b);
    gl(b);

    input_data.allocate(352, 424 * 9);

    for(int i = 0; i < 3; ++i)
      stage1_data[i].allocate(512, 424);

    if(do_debug) stage1_debug.allocate(512, 424);
    stage1_infrared.allocate(512, 424);

    for(int i = 0; i < 2; ++i)
      filter1_data[i].allocate(512, 424);

    filter1_max_edge_test.allocate(512, 424);
    if(do_debug) filter1_debug.allocate(512, 424);

    if(do_debug) stage2_debug.allocate(512, 424);
    stage2_depth.allocate(512, 424);
    stage2_depth_and_ir_sum.allocate(512, 424);

    if(do_debug) filter2_debug.allocate(512, 424);
    filter2_depth.allocate(512, 424);

    stage1.setVertexShader(defaultvs);
    stage1.setFragmentShader(stage1fs);
    stage1.bindFragDataLocation("Debug", 0);
    stage1.bindFragDataLocation("A", 1);
    stage1.bindFragDataLocation("B", 2);
    stage1.bindFragDataLocation("Norm", 3);
    stage1.bindFragDataLocation("Infrared", 4);
    stage1.build();

    filter1.setVertexShader(defaultvs);
    filter1.setFragmentShader(filter1fs);
    filter1.bindFragDataLocation("Debug", 0);
    filter1.bindFragDataLocation("FilterA", 1);
    filter1.bindFragDataLocation("FilterB", 2);
    filter1.bindFragDataLocation("MaxEdgeTest", 3);
    filter1.build();

    stage2.setVertexShader(defaultvs);
    stage2.setFragmentShader(stage2fs);
    stage2.bindFragDataLocation("Debug", 0);
    stage2.bindFragDataLocation("Depth", 1);
    stage2.bindFragDataLocation("DepthAndIrSum", 2);
    stage2.build();

    filter2.setVertexShader(defaultvs);
    filter2.setFragmentShader(filter2fs);
    filter2.bindFragDataLocation("Debug", 0);
    filter2.bindFragDataLocation("FilterDepth", 1);
    filter2.build();

    if(do_debug)
    {
      debug.setVertexShader(defaultvs);
      debug.setFragmentShader(debugfs);
      debug.bindFragDataLocation("Debug", 0);
      debug.build();
    }

    GLenum debug_attachment = do_debug ? GL_COLOR_ATTACHMENT0 : GL_NONE;

    gl()->glGenFramebuffers(1, &stage1_framebuffer);
    gl()->glBindFramebuffer(GL_FRAMEBUFFER, stage1_framebuffer);

    const GLenum stage1_buffers[] = { debug_attachment, GL_COLOR_ATTACHMENT1, GL_COLOR_ATTACHMENT2, GL_COLOR_ATTACHMENT3, GL_COLOR_ATTACHMENT4 };
    gl()->glDrawBuffers(5, stage1_buffers);
    glReadBuffer(GL_COLOR_ATTACHMENT4);

    if(do_debug) gl()->glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_RECTANGLE, stage1_debug.texture, 0);
    gl()->glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT1, GL_TEXTURE_RECTANGLE, stage1_data[0].texture, 0);
    gl()->glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT2, GL_TEXTURE_RECTANGLE, stage1_data[1].texture, 0);
    gl()->glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT3, GL_TEXTURE_RECTANGLE, stage1_data[2].texture, 0);
    gl()->glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT4, GL_TEXTURE_RECTANGLE, stage1_infrared.texture, 0);
    checkFBO(GL_FRAMEBUFFER);

    gl()->glGenFramebuffers(1, &filter1_framebuffer);
    gl()->glBindFramebuffer(GL_FRAMEBUFFER, filter1_framebuffer);

    const GLenum filter1_buffers[] = { debug_attachment, GL_COLOR_ATTACHMENT1, GL_COLOR_ATTACHMENT2, GL_COLOR_ATTACHMENT3 };
    gl()->glDrawBuffers(4, filter1_buffers);
    glReadBuffer(GL_NONE);

    if(do_debug) gl()->glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_RECTANGLE, filter1_debug.texture, 0);
    gl()->glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT1, GL_TEXTURE_RECTANGLE, filter1_data[0].texture, 0);
    gl()->glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT2, GL_TEXTURE_RECTANGLE, filter1_data[1].texture, 0);
    gl()->glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT3, GL_TEXTURE_RECTANGLE, filter1_max_edge_test.texture, 0);
    checkFBO(GL_FRAMEBUFFER);

    gl()->glGenFramebuffers(1, &stage2_framebuffer);
    gl()->glBindFramebuffer(GL_FRAMEBUFFER, stage2_framebuffer);

    const GLenum stage2_buffers[] = { debug_attachment, GL_COLOR_ATTACHMENT1, GL_COLOR_ATTACHMENT2 };
    gl()->glDrawBuffers(3, stage2_buffers);
    glReadBuffer(GL_COLOR_ATTACHMENT1);

    if(do_debug) gl()->glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_RECTANGLE, stage2_debug.texture, 0);
    gl()->glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT1, GL_TEXTURE_RECTANGLE, stage2_depth.texture, 0);
    gl()->glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT2, GL_TEXTURE_RECTANGLE, stage2_depth_and_ir_sum.texture, 0);
    checkFBO(GL_FRAMEBUFFER);

    gl()->glGenFramebuffers(1, &filter2_framebuffer);
    gl()->glBindFramebuffer(GL_FRAMEBUFFER, filter2_framebuffer);

    const GLenum filter2_buffers[] = { debug_attachment, GL_COLOR_ATTACHMENT1 };
    gl()->glDrawBuffers(2, filter2_buffers);
    glReadBuffer(GL_COLOR_ATTACHMENT1);

    if(do_debug) gl()->glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_RECTANGLE, filter2_debug.texture, 0);
    gl()->glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT1, GL_TEXTURE_RECTANGLE, filter2_depth.texture, 0);
    checkFBO(GL_FRAMEBUFFER);

    Vertex bl = {-1.0f, -1.0f, 0.0f, 0.0f }, br = { 1.0f, -1.0f, 512.0f, 0.0f }, tl = {-1.0f, 1.0f, 0.0f, 424.0f }, tr = { 1.0f, 1.0f, 512.0f, 424.0f };
    Vertex vertices[] = {
        bl, tl, tr, tr, br, bl
    };
    gl()->glGenBuffers(1, &square_vbo);
    gl()->glGenVertexArrays(1, &square_vao);

    gl()->glBindVertexArray(square_vao);
    gl()->glBindBuffer(GL_ARRAY_BUFFER, square_vbo);
    gl()->glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);

    GLint position_attr = stage1.getAttributeLocation("InputPosition");
    gl()->glVertexAttribPointer(position_attr, 2, GL_FLOAT, GL_FALSE, sizeof(Vertex), (GLvoid*)0);
    gl()->glEnableVertexAttribArray(position_attr);

    GLint texcoord_attr = stage1.getAttributeLocation("InputTexCoord");
    gl()->glVertexAttribPointer(texcoord_attr, 2, GL_FLOAT, GL_FALSE, sizeof(Vertex), (GLvoid*)(2 * sizeof(float)));
    gl()->glEnableVertexAttribArray(texcoord_attr);
    CHECKGL();
  }

  void deinitialize()
  {
  }

  void updateShaderParametersForProgram(ShaderProgram &program)
  {
    if(!params_need_update) return;

    program.setUniform("Params.ab_multiplier", params.ab_multiplier);
    program.setUniformVector3("Params.ab_multiplier_per_frq", params.ab_multiplier_per_frq);
    program.setUniform("Params.ab_output_multiplier", params.ab_output_multiplier);

    program.setUniformVector3("Params.phase_in_rad", params.phase_in_rad);

    program.setUniform("Params.joint_bilateral_ab_threshold", params.joint_bilateral_ab_threshold);
    program.setUniform("Params.joint_bilateral_max_edge", params.joint_bilateral_max_edge);
    program.setUniform("Params.joint_bilateral_exp", params.joint_bilateral_exp);
    program.setUniformMatrix3("Params.gaussian_kernel", params.gaussian_kernel);

    program.setUniform("Params.phase_offset", params.phase_offset);
    program.setUniform("Params.unambigious_dist", params.unambigious_dist);
    program.setUniform("Params.individual_ab_threshold", params.individual_ab_threshold);
    program.setUniform("Params.ab_threshold", params.ab_threshold);
    program.setUniform("Params.ab_confidence_slope", params.ab_confidence_slope);
    program.setUniform("Params.ab_confidence_offset", params.ab_confidence_offset);
    program.setUniform("Params.min_dealias_confidence", params.min_dealias_confidence);
    program.setUniform("Params.max_dealias_confidence", params.max_dealias_confidence);

    program.setUniform("Params.edge_ab_avg_min_value", params.edge_ab_avg_min_value);
    program.setUniform("Params.edge_ab_std_dev_threshold", params.edge_ab_std_dev_threshold);
    program.setUniform("Params.edge_close_delta_threshold", params.edge_close_delta_threshold);
    program.setUniform("Params.edge_far_delta_threshold", params.edge_far_delta_threshold);
    program.setUniform("Params.edge_max_delta_threshold", params.edge_max_delta_threshold);
    program.setUniform("Params.edge_avg_delta_threshold", params.edge_avg_delta_threshold);
    program.setUniform("Params.max_edge_count", params.max_edge_count);

    program.setUniform("Params.min_depth", params.min_depth);
    program.setUniform("Params.max_depth", params.max_depth);
  }

  void run(Frame **ir, Frame **depth)
  {
    // data processing 1
    glViewport(0, 0, 512, 424);
    stage1.use();
    updateShaderParametersForProgram(stage1);

    p0table[0].bindToUnit(GL_TEXTURE0);
    stage1.setUniform("P0Table0", 0);
    p0table[1].bindToUnit(GL_TEXTURE1);
    stage1.setUniform("P0Table1", 1);
    p0table[2].bindToUnit(GL_TEXTURE2);
    stage1.setUniform("P0Table2", 2);
    lut11to16.bindToUnit(GL_TEXTURE3);
    stage1.setUniform("Lut11to16", 3);
    input_data.bindToUnit(GL_TEXTURE4);
    stage1.setUniform("Data", 4);
    z_table.bindToUnit(GL_TEXTURE5);
    stage1.setUniform("ZTable", 5);

    gl()->glBindFramebuffer(GL_DRAW_FRAMEBUFFER, stage1_framebuffer);
    glClear(GL_COLOR_BUFFER_BIT);

    gl()->glBindVertexArray(square_vao);
    glDrawArrays(GL_TRIANGLES, 0, 6);
    CHECKGL();

    if(ir != 0)
    {
      gl()->glBindFramebuffer(GL_READ_FRAMEBUFFER, stage1_framebuffer);
      glReadBuffer(GL_COLOR_ATTACHMENT4);
      *ir = stage1_infrared.downloadToNewFrame();
    }

    if(config.EnableBilateralFilter)
    {
      // bilateral filter
      gl()->glBindFramebuffer(GL_DRAW_FRAMEBUFFER, filter1_framebuffer);
      glClear(GL_COLOR_BUFFER_BIT);

      filter1.use();
      updateShaderParametersForProgram(filter1);

      stage1_data[0].bindToUnit(GL_TEXTURE0);
      filter1.setUniform("A", 0);
      stage1_data[1].bindToUnit(GL_TEXTURE1);
      filter1.setUniform("B", 1);
      stage1_data[2].bindToUnit(GL_TEXTURE2);
      filter1.setUniform("Norm", 2);

      gl()->glBindVertexArray(square_vao);
      glDrawArrays(GL_TRIANGLES, 0, 6);
    }
    // data processing 2
    gl()->glBindFramebuffer(GL_DRAW_FRAMEBUFFER, stage2_framebuffer);
    glClear(GL_COLOR_BUFFER_BIT);

    stage2.use();
    updateShaderParametersForProgram(stage2);
    CHECKGL();

    if(config.EnableBilateralFilter)
    {
      filter1_data[0].bindToUnit(GL_TEXTURE0);
      filter1_data[1].bindToUnit(GL_TEXTURE1);
    }
    else
    {
      stage1_data[0].bindToUnit(GL_TEXTURE0);
      stage1_data[1].bindToUnit(GL_TEXTURE1);
    }
    stage2.setUniform("A", 0);
    stage2.setUniform("B", 1);
    x_table.bindToUnit(GL_TEXTURE2);
    stage2.setUniform("XTable", 2);
    z_table.bindToUnit(GL_TEXTURE3);
    stage2.setUniform("ZTable", 3);

    gl()->glBindVertexArray(square_vao);
    glDrawArrays(GL_TRIANGLES, 0, 6);
    CHECKGL();

    if(config.EnableEdgeAwareFilter)
    {
      // edge aware filter
      gl()->glBindFramebuffer(GL_DRAW_FRAMEBUFFER, filter2_framebuffer);
      glClear(GL_COLOR_BUFFER_BIT);

      filter2.use();
      updateShaderParametersForProgram(filter2);

      stage2_depth_and_ir_sum.bindToUnit(GL_TEXTURE0);
      filter2.setUniform("DepthAndIrSum", 0);
      filter1_max_edge_test.bindToUnit(GL_TEXTURE1);
      filter2.setUniform("MaxEdgeTest", 1);

      gl()->glBindVertexArray(square_vao);
      glDrawArrays(GL_TRIANGLES, 0, 6);
      if(depth != 0)
      {
        gl()->glBindFramebuffer(GL_READ_FRAMEBUFFER, filter2_framebuffer);
        glReadBuffer(GL_COLOR_ATTACHMENT1);
        *depth = filter2_depth.downloadToNewFrame();
      }
    }
    else
    {
      if(depth != 0)
      {
        gl()->glBindFramebuffer(GL_READ_FRAMEBUFFER, stage2_framebuffer);
        glReadBuffer(GL_COLOR_ATTACHMENT1);
        *depth = stage2_depth.downloadToNewFrame();
      }
    }
    CHECKGL();

    if(do_debug)
    {
      // debug drawing
      gl()->glBindFramebuffer(GL_DRAW_FRAMEBUFFER, 0);
      glClear(GL_COLOR_BUFFER_BIT);

      gl()->glBindVertexArray(square_vao);

      debug.use();
      stage2_debug.bindToUnit(GL_TEXTURE0);
      debug.setUniform("Debug", 0);

      glDrawArrays(GL_TRIANGLES, 0, 6);

      glViewport(512, 0, 512, 424);
      filter2_debug.bindToUnit(GL_TEXTURE0);
      debug.setUniform("Debug", 0);

      glDrawArrays(GL_TRIANGLES, 0, 6);

      glViewport(0, 424, 512, 424);
      stage1_debug.bindToUnit(GL_TEXTURE0);
      debug.setUniform("Debug", 0);

      glDrawArrays(GL_TRIANGLES, 0, 6);
    }
    CHECKGL();

    params_need_update = false;
  }
};

OpenGLDepthPacketProcessor::OpenGLDepthPacketProcessor(void *parent_opengl_context_ptr, bool debug)
{
  GLFWwindow* parent_window = (GLFWwindow *)parent_opengl_context_ptr;

  GLFWerrorfun prev_func = glfwSetErrorCallback(&OpenGLDepthPacketProcessorImpl::glfwErrorCallback);
  if (prev_func)
    glfwSetErrorCallback(prev_func);

  // init glfw - if already initialized nothing happens
  if (glfwInit() == GL_FALSE)
  {
      LOG_ERROR << "Failed to initialize GLFW.";
      exit(-1);
  }
  
  // setup context
  glfwDefaultWindowHints();
  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
#ifdef __APPLE__
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
  glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
  glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
#else
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 1);
  glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_ANY_PROFILE);
#endif
  glfwWindowHint(GLFW_VISIBLE, debug ? GL_TRUE : GL_FALSE);

  GLFWwindow* window = glfwCreateWindow(1024, 848, "OpenGLDepthPacketProcessor", 0, parent_window);

  if (window == NULL)
  {
      LOG_ERROR << "Failed to create opengl window.";
      exit(-1);
  }

  impl_ = new OpenGLDepthPacketProcessorImpl(window, debug);
  impl_->initialize();
}

OpenGLDepthPacketProcessor::~OpenGLDepthPacketProcessor()
{
  delete impl_;
}


void OpenGLDepthPacketProcessor::setConfiguration(const libfreenect2::DepthPacketProcessor::Config &config)
{
  DepthPacketProcessor::setConfiguration(config);
  impl_->config = config;

  impl_->params.min_depth = impl_->config.MinDepth * 1000.0f;
  impl_->params.max_depth = impl_->config.MaxDepth * 1000.0f;

  impl_->params_need_update = true;
}

void OpenGLDepthPacketProcessor::loadP0TablesFromCommandResponse(unsigned char* buffer, size_t buffer_length)
{
  ChangeCurrentOpenGLContext ctx(impl_->opengl_context_ptr);

  size_t n = 512 * 424;
  libfreenect2::protocol::P0TablesResponse* p0table = (libfreenect2::protocol::P0TablesResponse*)buffer;

  impl_->p0table[0].allocate(512, 424);
  std::copy(reinterpret_cast<unsigned char*>(p0table->p0table0), reinterpret_cast<unsigned char*>(p0table->p0table0 + n), impl_->p0table[0].data);
  impl_->p0table[0].flipY();
  impl_->p0table[0].upload();

  impl_->p0table[1].allocate(512, 424);
  std::copy(reinterpret_cast<unsigned char*>(p0table->p0table1), reinterpret_cast<unsigned char*>(p0table->p0table1 + n), impl_->p0table[1].data);
  impl_->p0table[1].flipY();
  impl_->p0table[1].upload();

  impl_->p0table[2].allocate(512, 424);
  std::copy(reinterpret_cast<unsigned char*>(p0table->p0table2), reinterpret_cast<unsigned char*>(p0table->p0table2 + n), impl_->p0table[2].data);
  impl_->p0table[2].flipY();
  impl_->p0table[2].upload();

}

void OpenGLDepthPacketProcessor::loadXZTables(const float *xtable, const float *ztable)
{
  ChangeCurrentOpenGLContext ctx(impl_->opengl_context_ptr);

  impl_->x_table.allocate(512, 424);
  std::copy(xtable, xtable + TABLE_SIZE, (float *)impl_->x_table.data);
  impl_->x_table.upload();

  impl_->z_table.allocate(512, 424);
  std::copy(ztable, ztable + TABLE_SIZE, (float *)impl_->z_table.data);
  impl_->z_table.upload();
}

void OpenGLDepthPacketProcessor::loadLookupTable(const short *lut)
{
  ChangeCurrentOpenGLContext ctx(impl_->opengl_context_ptr);

  impl_->lut11to16.allocate(2048, 1);
  std::copy(lut, lut + LUT_SIZE, (short *)impl_->lut11to16.data);
  impl_->lut11to16.upload();
}

void OpenGLDepthPacketProcessor::process(const DepthPacket &packet)
{
  bool has_listener = this->listener_ != 0;
  Frame *ir = 0, *depth = 0;

  impl_->startTiming();

  glfwMakeContextCurrent(impl_->opengl_context_ptr);

  std::copy(packet.buffer, packet.buffer + packet.buffer_length/10*9, impl_->input_data.data);
  impl_->input_data.upload();
  impl_->run(has_listener ? &ir : 0, has_listener ? &depth : 0);

  if(impl_->do_debug) glfwSwapBuffers(impl_->opengl_context_ptr);

  impl_->stopTiming(LOG_INFO);

  if(has_listener)
  {
    ir->timestamp = packet.timestamp;
    depth->timestamp = packet.timestamp;
    ir->sequence = packet.sequence;
    depth->sequence = packet.sequence;

    if(!this->listener_->onNewFrame(Frame::Ir, ir))
    {
      delete ir;
    }

    if(!this->listener_->onNewFrame(Frame::Depth, depth))
    {
      delete depth;
    }
  }
}

} /* namespace libfreenect2 */
