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

#include "sharedCode.h"

#include "rng.h"

GPRT_COMPUTE_PROGRAM(GenParticles, (ParticleData, record), (1, 1, 1)) {
  int primID = DispatchThreadID.x;
  float t = float(primID) / float(record.numParticles);
  float4 particle = float4(t * sin(t * 256.f * 3.14f),  t * cos(t * 256.f * 3.14f), t * sin(t * 256.f * 3.14f) * cos(t * 256.f * 3.14f), 1.f);
  gprt::store<float4>(record.particles, primID, particle);
}

GPRT_COMPUTE_PROGRAM(GenRBFBounds, (ParticleData, record), (1, 1, 1)) {
  int primID = DispatchThreadID.x;
  float4 particle = gprt::load<float4>(record.particles, primID);
  float radius = record.rbfRadius; // prior work just set this to some global constant.
  float3 aabbMin = particle.xyz - float3(radius, radius, radius);
  float3 aabbMax = particle.xyz + float3(radius, radius, radius);
  gprt::store(record.aabbs, 2 * primID, aabbMin);
  gprt::store(record.aabbs, 2 * primID + 1, aabbMax);
}

GPRT_COMPUTE_PROGRAM(AccumulateRBFBounds, (RayGenData, record), (1,1,1)) {
  int primID = DispatchThreadID.x;
  float4 particle = gprt::load<float4>(record.particles, primID);
  float radius = record.rbfRadius; // prior work just set this to some global constant.
  float3 aabbMin = particle.xyz - float3(radius, radius, radius);
  float3 aabbMax = particle.xyz + float3(radius, radius, radius);

  float3 rt = record.globalAABBMax;
  float3 lb = record.globalAABBMin;
  int3 dims = record.volumeDimensions;

  Texture1D colormap = gprt::getTexture1DHandle(record.colormap);
  SamplerState sampler = gprt::getSamplerHandle(record.colormapSampler);
  float clampMaxCumulativeValue = record.clampMaxCumulativeValue;

  // transform particle into voxel space
  aabbMin = (aabbMin - lb) / (rt - lb);
  aabbMax = (aabbMax - lb) / (rt - lb);

  aabbMin = aabbMin * dims;
  aabbMax = aabbMax * dims;

  aabbMin = min(aabbMin, dims - 1);
  aabbMax = min(aabbMax, dims - 1);

  // testing
  // float4 xf = float4(1.0f, 0.f, 0.f, 1.0f);
  // gprt::store<float4>(record.volume, primID, xf);
  // gprt::store<float>(record.volumeCount, primID, 1.f);



  // if (primID == 0) {
  //   printf("storing to ");
  // }
  
  // rasterize particle to all voxels it touches
  for (uint z = aabbMin.z; z <= aabbMax.z; ++z) {
    for (uint y = aabbMin.y; y <= aabbMax.y; ++y) {
      for (uint x = aabbMin.x; x <= aabbMax.x; ++x) {
        uint32_t addr = x + y * dims.x + z * dims.x * dims.y;

        float3 pt = float3(x, y, z) + .5; // voxel space
        pt = pt / float3(dims); // normalized
        pt = (pt * (rt - lb)) + lb; // world space

        if (distance(pt, particle.xyz) >= radius) continue;

        float density = evaluate_rbf(pt, particle.xyz, radius);

        if (clampMaxCumulativeValue > 0.f) density /= clampMaxCumulativeValue;
        float4 xf = colormap.SampleGrad(sampler, density, 0.f, 0.f);

        
        // if (primID == 0) {
        //   printf("%lu ", addr);
        // }
        // gprt::store<float4>(record.volume, addr, xf);

        gprt::atomicAdd32f(record.volume, addr * 4 + 0, xf.r);
        gprt::atomicAdd32f(record.volume, addr * 4 + 1, xf.g);
        gprt::atomicAdd32f(record.volume, addr * 4 + 2, xf.b);
        gprt::atomicAdd32f(record.volume, addr * 4 + 3, xf.w);
        gprt::atomicAdd32f(record.volumeCount, addr, 1.f);
      }
    }
  }
  // if (primID == 0) {
  //   printf("\n ");
  // }
}

GPRT_COMPUTE_PROGRAM(AverageRBFBounds, (RayGenData, record), (1,1,1)) {
  int voxelID = DispatchThreadID.x;
  float4 voxel = gprt::load<float4>(record.volume, voxelID);
  float count = gprt::load<float>(record.volumeCount, voxelID);
  if (count > 0.f) {
    float4 color = float4((voxel.xyz / count), voxel.w);
    gprt::store<float4>(record.volume, voxelID, color);
  } else {
    Texture1D colormap = gprt::getTexture1DHandle(record.colormap);
    SamplerState sampler = gprt::getSamplerHandle(record.colormapSampler);
    float4 xf = colormap.SampleGrad(sampler, 0.f, 0.f, 0.f);
    gprt::store<float4>(record.volume, voxelID, xf);
  }
}

struct NullPayload{
  int tmp;
};

GPRT_MISS_PROGRAM(miss, (MissProgData, record), (NullPayload, payload)) {}
