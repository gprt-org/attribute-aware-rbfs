#pragma once
#include <iostream>
#include <stdint.h>
#include <stdexcept>
#include <vector>
// stb already defined by hostCode.cpp
// #define STB_IMAGE_IMPLEMENTATION
// #include <stb/stb_image.h>
#include "sharedCode.h"

struct ColorMap {
  ColorMap() = default;
  ColorMap(const uint8_t asPNG[], size_t numBytes)
  {
    int w, h, n;
    uint8_t *img_data = stbi_load_from_memory(asPNG, numBytes, &w, &h, &n, 4);
    if (n != 3 && n != 4)
      throw std::runtime_error("ColorMap::fromPNG: only supporting "
                               "PNG files with either 3 or 4 channels");
    values_.reserve(w);
    for (std::size_t i = 0; i < w; ++i) {
      float4 v;
      v.x = img_data[i * 4 + 0] / 255.f;
      v.y = img_data[i * 4 + 1] / 255.f;
      v.z = img_data[i * 4 + 2] / 255.f;
      v.w
        = (n == 3)
        ? 1.f
        : img_data[i * 4 + 3] / 255.f;
      // v.w = 1.f;
      values_.push_back(v);
    }

    stbi_image_free(img_data);
  }

  ColorMap(const std::vector<std::pair<float,float4>> &controlPoints, int N=1024)
  {
    values_.resize(N);
    for (size_t i=0; i<values_.size(); ++i) {
      float f=i/float(values_.size()-1);
      for (size_t j=0; j<controlPoints.size()-1; ++j) {
        if (controlPoints[j].first <= f && controlPoints[j+1].first >= f) {
          float f0 = controlPoints[j].first;
          float4 c0 = controlPoints[j].second;
          float f1 = controlPoints[j+1].first;
          float4 c1 = controlPoints[j+1].second;

          float frac=(f-f0)/(f1-f0);
          values_[i] = c0*(1.f-frac)+c1*frac;
          //std::cout << j << ',' << values_[i] << '\n';
          break;
        }
      }
    }
  }

  float4 at(float f01)
  {
    float f0 = f01*(values_.size()-1);
    float f1 = f01*(values_.size()-1)+1;

    int tc0 = (int)linalg::clamp(f0, 0.f, values_.size()-1.f);
    int tc1 = (int)linalg::clamp(f1, 0.f, values_.size()-1.f);

    float frac = f0-tc0;

    return values_[tc0]*frac + values_[tc1]*(1.f-frac);
  }

  std::vector<float4> values_;
};

