// MIT License

// Copyright (c) 2022 Nathan V. Morrical

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include "gprt.h"

#define MAX_DEPTH 10000
#define EPSILON 2.2204460492503130808472633361816E-16
#define FLOAT_EPSILON 1.19209290e-7F
#define DOUBLE_EPSILON 2.2204460492503131e-16

/* variables available to all programs */

struct ParticleData {
  alignas(4) uint32_t numParticles;
  alignas(4) float rbfRadius;
  alignas(4) float clampMaxCumulativeValue;
  alignas(16) gprt::Buffer particles;
  alignas(16) gprt::Buffer aabbs;
  alignas(16) gprt::Buffer globalAABB;
};

struct RayGenData {
  alignas(16) gprt::Texture guiTexture;
  alignas(16) gprt::Buffer frameBuffer;
  alignas(16) gprt::Buffer accumBuffer;
  alignas(4) uint32_t frameID;

  alignas(16) gprt::Accel world;
  alignas(16) gprt::Buffer globalAABB;
  alignas(4) float rbfRadius;
  alignas(4) float clampMaxCumulativeValue;
  alignas(16) gprt::Buffer particles;

  // colormap for visualization
  alignas(16) gprt::Texture colormap;
  alignas(16) gprt::Sampler colormapSampler;

  // In the case that we end up rasterizing our particles
  // into a grid.
  alignas(16) gprt::Buffer volume;
  alignas(16) gprt::Buffer volumeCount;
  alignas(16) uint3 volumeDimensions;

  // For controling the relative density of delta tracking
  alignas(4) float unit;

  struct {
    alignas(4) float azimuth;
    alignas(4) float elevation;
    alignas(4) float ambient;
  } light;

  struct {
    alignas(16) float3 pos;
    alignas(16) float3 dir_00;
    alignas(16) float3 dir_du;
    alignas(16) float3 dir_dv;
  } camera;
};

/* variables for the miss program */
struct MissProgData {
  alignas(16) float3 color0;
  alignas(16) float3 color1;
};
