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
};

struct RayGenData {
  alignas(16) gprt::Texture guiTexture;
  alignas(16) gprt::Buffer frameBuffer;
  alignas(16) gprt::Buffer accumBuffer;
  alignas(4) uint32_t frameID;

  alignas(16) gprt::Accel world;
  alignas(16) float3 globalAABBMin;
  alignas(16) float3 globalAABBMax;
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

#ifdef GPRT_DEVICE

/* shared functions */
float3 getLightDirection(float azimuth, float elevation) {
  return float3(
      sin(elevation * 3.14f - 0.5f * 3.14f) * cos(azimuth * 2.f * 3.14f),
      sin(elevation * 3.14f - 0.5f * 3.14f) * sin(azimuth * 2.f * 3.14f),
      cos(elevation * 3.14f - 0.5f * 3.14f));
}

// P is the particle center
// X is the intersection point
// r is the radius of the particle
float evaluate_rbf(float3 X, float3 P, float r) {
  return exp(-pow(distance(X, P), 2.f) / pow(r, 2.f));
}

float4 over(float4 a, float4 b) {
  float4 result;
  result.a = a.a + b.a * (1.f - a.a);
  result.rgb = (a.rgb * a.a + b.rgb * b.a * (1.f - a.a)) / result.a;
  return result;
}

bool aabbIntersection(in RayDesc rayDesc, float3 aabbMin, float3 aabbMax,
                      out float tenter, out float texit) {
  // typical ray AABB intersection test
  float3 dirfrac; // direction is unit direction vector of ray
  dirfrac.x = 1.0f / rayDesc.Direction.x;
  dirfrac.y = 1.0f / rayDesc.Direction.y;
  dirfrac.z = 1.0f / rayDesc.Direction.z;
  // lb is the corner of AABB with minimal coordinates - left bottom, rt is
  // maximal corner origin is origin of ray
  float3 rt = aabbMax;
  float t2 = (rt.x - rayDesc.Origin.x) * dirfrac.x;
  float t4 = (rt.y - rayDesc.Origin.y) * dirfrac.y;
  float t6 = (rt.z - rayDesc.Origin.z) * dirfrac.z;
  float3 lb = aabbMin;
  float t1 = (lb.x - rayDesc.Origin.x) * dirfrac.x;
  float t3 = (lb.y - rayDesc.Origin.y) * dirfrac.y;
  float t5 = (lb.z - rayDesc.Origin.z) * dirfrac.z;
  tenter = max(max(min(t1, t2), min(t3, t4)), min(t5, t6));
  texit = min(min(max(t1, t2), max(t3, t4)),
              max(t5, t6)); // clip hit to near position
  // truncate interval to ray description tmin/max
  tenter = max(tenter, rayDesc.TMin);
  texit = min(texit, rayDesc.TMax);
  // if texit < 0, ray (line) is intersecting AABB, but the whole AABB is behind
  // us
  bool hit = true;
  if (texit < 0) {
    hit = false;
  }
  // if tenter > texit, ray doesn't intersect AABB
  if (tenter >= texit) {
    hit = false;
  }
  return hit;
}

#endif
