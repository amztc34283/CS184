#include "texture.h"
#include "CGL/color.h"

#include <cmath>
#include <algorithm>

namespace CGL {

Color Texture::sample(const SampleParams &sp) {
  // TODO: Task 6: Fill this in.

  // NOTE: When assigning or passing Texture or MipLevel type, use pointers or reference!
  // Remember that C/C++ is pass by reference, directly passing Texture or MipLevel
  // will cause horrible performance.

  // e.g. MipLevel& l = mipmap[0]; // This is pass by reference. Note that the type here is MipLevel&
  // e.g. MipLevel* l = &mipmap[0]; // This is pass by pointer. Note that the type here is MipLevel*
  // DO NOT do things like MipLevel l = mipmap[0]; This will cause a copy. You should only copy basic types
  // like int, float, double, Vector2D, Vector3D, ... An exception is std::vector, you can return a std::vector
  // from a function and it will not be duplicated.


    float level = get_level(sp);
    switch (sp.psm) {
        case P_NEAREST:
        {
            if (sp.lsm == L_LINEAR) {
                float cropped_level = std::max(0.0f, std::min((float) mipmap.size() - 1, level));
                int prev_level = std::floor(cropped_level);
                int next_level = std::ceil(cropped_level);
                if (next_level == prev_level)
                    next_level += 1;
                Color prev_color = sample_nearest(sp.p_uv, prev_level);
                Color next_color = sample_nearest(sp.p_uv, next_level);
                return (cropped_level - prev_level) * next_color + (next_level - cropped_level) * prev_color;
            } else {
                int nearest_level = (int) std::round(
                        std::max(0.0f,
                                 std::min((float) mipmap.size() - 1, level)));
                return sample_nearest(sp.p_uv, nearest_level);
            }
        }
        case P_LINEAR:
        {
            if (sp.lsm == L_LINEAR) {
                float cropped_level = std::max(0.0f, std::min((float) mipmap.size() - 1, level));
                int prev_level = std::floor(cropped_level);
                int next_level = std::ceil(cropped_level);
                if (next_level == prev_level)
                    next_level += 1;
                Color prev_color = sample_bilinear(sp.p_uv, prev_level);
                Color next_color = sample_bilinear(sp.p_uv, next_level);
                return (cropped_level - prev_level) * next_color + (next_level - cropped_level) * prev_color;
            } else {
                int nearest_level = (int) std::round(
                        std::max(0.0f,
                                 std::min((float) mipmap.size() - 1, level)));
                return sample_bilinear(sp.p_uv, nearest_level);
            }

        }
    }
}

float Texture::get_level(const SampleParams &sp) {
  // TODO: Task 6: Fill this in.
  // NOTE: When assigning or passing Texture or MipLevel type, use pointers or reference!
  // Remember that C/C++ is pass by reference, directly passing Texture or MipLevel
  // will cause horrible performance.
    switch (sp.lsm) {
        case L_ZERO:
            return 0;
        case L_NEAREST:
            return std::round(std::log2(std::max(
                    std::sqrt(std::pow(sp.p_dx_uv[0] - sp.p_uv[0], 2) * width
                              + std::pow(sp.p_dx_uv[1] - sp.p_uv[1], 2) * height),
                    std::sqrt(std::pow(sp.p_dy_uv[0] - sp.p_uv[0], 2) * width
                              + std::pow(sp.p_dy_uv[1] - sp.p_uv[1], 2) * height))));
        case L_LINEAR:
            return std::log2(std::max(
                    std::sqrt(std::pow(sp.p_dx_uv[0] - sp.p_uv[0], 2) * width
                              + std::pow(sp.p_dx_uv[1] - sp.p_uv[1], 2) * height),
                    std::sqrt(std::pow(sp.p_dy_uv[0] - sp.p_uv[0], 2) * width
                              + std::pow(sp.p_dy_uv[1] - sp.p_uv[1], 2) * height)));
    }
}

Color MipLevel::get_texel(int tx, int ty) {
  return Color(&texels[tx*3 + ty*width*3]);
}

Color Texture::sample_nearest(Vector2D uv, int level) {
  // TODO: Task 5: Fill this in.
    double u = uv[0];
    double v = uv[1];
    // round to nearest int
    // convert from uv coordinate to txty coordinate using the width and height
    int nearest_tx = std::round(u * (width - 1));
    int nearest_ty = std::round(v * (height - 1));
    MipLevel &mipmap_level = mipmap[level];
    if (nearest_tx >= 0 && nearest_tx < width
        && nearest_ty >= 0 && nearest_ty < height
        && nearest_tx < mipmap_level.width
        && nearest_ty < mipmap_level.height)
        return mipmap_level.get_texel(nearest_tx, nearest_ty);
}

Color Texture::sample_bilinear(Vector2D uv, int level) {
  // TODO: Task 5: Fill this in.
    double u = uv[0];
    double v = uv[1];
    // floor to left-upper corner of the square
    // offset 0.5 to get the correct four squares <- not used anymore
    // convert from uv coordinate to txty coordinate using the width and height
    float tx = u * (width - 1);
    float ty = v * (height - 1);
    int start_x = std::floor(tx);
    int start_y = std::floor(ty);
    int end_x = start_x + 1;
    int end_y = start_y + 1;
    MipLevel &mipmap_level = mipmap[level];
    // linear interpolate
    float s = end_x - tx;
    float t = end_y - ty;
    float s_inv = 1 - s;
    float t_inv = 1 - t;
    if (start_x >= 0 && start_x < width && start_y >= 0 && start_y < height
        && end_x >= 0 && end_x < width && end_y >= 0 && end_y < height
        && start_x < mipmap_level.width && start_y < mipmap_level.height
        && end_x < mipmap_level.width && end_y < mipmap_level.height)
        // Area interpolation
        return s * t * mipmap_level.get_texel(start_x, start_y)
               + s * t_inv * mipmap_level.get_texel(start_x, end_y)
               + s_inv * t * mipmap_level.get_texel(end_x, start_y)
               + s_inv * t_inv * mipmap_level.get_texel(end_x, end_y);
}



/****************************************************************************/

// Helpers

inline void uint8_to_float(float dst[3], unsigned char *src) {
  uint8_t *src_uint8 = (uint8_t *)src;
  dst[0] = src_uint8[0] / 255.f;
  dst[1] = src_uint8[1] / 255.f;
  dst[2] = src_uint8[2] / 255.f;
}

inline void float_to_uint8(unsigned char *dst, float src[3]) {
  uint8_t *dst_uint8 = (uint8_t *)dst;
  dst_uint8[0] = (uint8_t)(255.f * max(0.0f, min(1.0f, src[0])));
  dst_uint8[1] = (uint8_t)(255.f * max(0.0f, min(1.0f, src[1])));
  dst_uint8[2] = (uint8_t)(255.f * max(0.0f, min(1.0f, src[2])));
}

void Texture::generate_mips(int startLevel) {

  // make sure there's a valid texture
  if (startLevel >= mipmap.size()) {
    std::cerr << "Invalid start level";
  }

  // allocate sublevels
  int baseWidth = mipmap[startLevel].width;
  int baseHeight = mipmap[startLevel].height;
  int numSubLevels = (int)(log2f((float)max(baseWidth, baseHeight)));

  numSubLevels = min(numSubLevels, kMaxMipLevels - startLevel - 1);
  mipmap.resize(startLevel + numSubLevels + 1);

  int width = baseWidth;
  int height = baseHeight;
  for (int i = 1; i <= numSubLevels; i++) {

    MipLevel &level = mipmap[startLevel + i];

    // handle odd size texture by rounding down
    width = max(1, width / 2);
    //assert (width > 0);
    height = max(1, height / 2);
    //assert (height > 0);

    level.width = width;
    level.height = height;
    level.texels = vector<unsigned char>(3 * width * height);
  }

  // create mips
  int subLevels = numSubLevels - (startLevel + 1);
  for (int mipLevel = startLevel + 1; mipLevel < startLevel + subLevels + 1;
       mipLevel++) {

    MipLevel &prevLevel = mipmap[mipLevel - 1];
    MipLevel &currLevel = mipmap[mipLevel];

    int prevLevelPitch = prevLevel.width * 3; // 32 bit RGB
    int currLevelPitch = currLevel.width * 3; // 32 bit RGB

    unsigned char *prevLevelMem;
    unsigned char *currLevelMem;

    currLevelMem = (unsigned char *)&currLevel.texels[0];
    prevLevelMem = (unsigned char *)&prevLevel.texels[0];

    float wDecimal, wNorm, wWeight[3];
    int wSupport;
    float hDecimal, hNorm, hWeight[3];
    int hSupport;

    float result[3];
    float input[3];

    // conditional differentiates no rounding case from round down case
    if (prevLevel.width & 1) {
      wSupport = 3;
      wDecimal = 1.0f / (float)currLevel.width;
    } else {
      wSupport = 2;
      wDecimal = 0.0f;
    }

    // conditional differentiates no rounding case from round down case
    if (prevLevel.height & 1) {
      hSupport = 3;
      hDecimal = 1.0f / (float)currLevel.height;
    } else {
      hSupport = 2;
      hDecimal = 0.0f;
    }

    wNorm = 1.0f / (2.0f + wDecimal);
    hNorm = 1.0f / (2.0f + hDecimal);

    // case 1: reduction only in horizontal size (vertical size is 1)
    if (currLevel.height == prevLevel.height) {
      //assert (currLevel.height == 1);

      for (int i = 0; i < currLevel.width; i++) {
        wWeight[0] = wNorm * (1.0f - wDecimal * i);
        wWeight[1] = wNorm * 1.0f;
        wWeight[2] = wNorm * wDecimal * (i + 1);

        result[0] = result[1] = result[2] = 0.0f;

        for (int ii = 0; ii < wSupport; ii++) {
          uint8_to_float(input, prevLevelMem + 3 * (2 * i + ii));
          result[0] += wWeight[ii] * input[0];
          result[1] += wWeight[ii] * input[1];
          result[2] += wWeight[ii] * input[2];
        }

        // convert back to format of the texture
        float_to_uint8(currLevelMem + (3 * i), result);
      }

      // case 2: reduction only in vertical size (horizontal size is 1)
    } else if (currLevel.width == prevLevel.width) {
      //assert (currLevel.width == 1);

      for (int j = 0; j < currLevel.height; j++) {
        hWeight[0] = hNorm * (1.0f - hDecimal * j);
        hWeight[1] = hNorm;
        hWeight[2] = hNorm * hDecimal * (j + 1);

        result[0] = result[1] = result[2] = 0.0f;
        for (int jj = 0; jj < hSupport; jj++) {
          uint8_to_float(input, prevLevelMem + prevLevelPitch * (2 * j + jj));
          result[0] += hWeight[jj] * input[0];
          result[1] += hWeight[jj] * input[1];
          result[2] += hWeight[jj] * input[2];
        }

        // convert back to format of the texture
        float_to_uint8(currLevelMem + (currLevelPitch * j), result);
      }

      // case 3: reduction in both horizontal and vertical size
    } else {

      for (int j = 0; j < currLevel.height; j++) {
        hWeight[0] = hNorm * (1.0f - hDecimal * j);
        hWeight[1] = hNorm;
        hWeight[2] = hNorm * hDecimal * (j + 1);

        for (int i = 0; i < currLevel.width; i++) {
          wWeight[0] = wNorm * (1.0f - wDecimal * i);
          wWeight[1] = wNorm * 1.0f;
          wWeight[2] = wNorm * wDecimal * (i + 1);

          result[0] = result[1] = result[2] = 0.0f;

          // convolve source image with a trapezoidal filter.
          // in the case of no rounding this is just a box filter of width 2.
          // in the general case, the support region is 3x3.
          for (int jj = 0; jj < hSupport; jj++)
            for (int ii = 0; ii < wSupport; ii++) {
              float weight = hWeight[jj] * wWeight[ii];
              uint8_to_float(input, prevLevelMem +
                                        prevLevelPitch * (2 * j + jj) +
                                        3 * (2 * i + ii));
              result[0] += weight * input[0];
              result[1] += weight * input[1];
              result[2] += weight * input[2];
            }

          // convert back to format of the texture
          float_to_uint8(currLevelMem + currLevelPitch * j + 3 * i, result);
        }
      }
    }
  }
}

}
