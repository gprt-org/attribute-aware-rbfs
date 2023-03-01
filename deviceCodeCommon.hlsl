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
  uint32_t clusterID = DispatchThreadID.x; 
  uint32_t particlesPerLeaf = record.particlesPerLeaf;
  uint32_t numParticles = record.numParticles;
  float radius = record.rbfRadius;

  float3 clusterAABBMin, clusterAABBMax;
  for (int i = 0; i < particlesPerLeaf; ++i) {
    int primID = DispatchThreadID.x * particlesPerLeaf + i;
    if (primID >= numParticles) break;

    float4 particle = gprt::load<float4>(record.particles, primID);
    float3 aabbMin = particle.xyz - float3(radius, radius, radius);
    float3 aabbMax = particle.xyz + float3(radius, radius, radius);
    if (i == 0) {
      clusterAABBMin = aabbMin;
      clusterAABBMax = aabbMax;
    }
    else {
      clusterAABBMin = min(clusterAABBMin, aabbMin);
      clusterAABBMax = max(clusterAABBMax, aabbMax);
    }
  }
  gprt::store(record.aabbs, 2 * clusterID, clusterAABBMin);
  gprt::store(record.aabbs, 2 * clusterID + 1, clusterAABBMax);
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
        // float u = density;
        // if (clampMaxCumulativeValue > 0.f) u /= clampMaxCumulativeValue;
        float4 xf = colormap.SampleGrad(sampler, particle.w, 0.f, 0.f);

        
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
  float clampMaxCumulativeValue = record.clampMaxCumulativeValue;
  if (count > 0.f) {
    if (clampMaxCumulativeValue > 0.f) voxel.w /= clampMaxCumulativeValue;
    float4 color = float4((voxel.xyz / count), voxel.w);
    gprt::store<float4>(record.volume, voxelID, color);
  } else {
    Texture1D colormap = gprt::getTexture1DHandle(record.colormap);
    SamplerState sampler = gprt::getSamplerHandle(record.colormapSampler);
    float4 xf = colormap.SampleGrad(sampler, 0.f, 0.f, 0.f);
    gprt::store<float4>(record.volume, voxelID, xf);
  }
}

GPRT_COMPUTE_PROGRAM(MinMaxRBFBounds, (RayGenData, record), (1,1,1)) {
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

  // rasterize particle to all voxels it touches
  for (uint z = aabbMin.z; z <= aabbMax.z; ++z) {
    for (uint y = aabbMin.y; y <= aabbMax.y; ++y) {
      for (uint x = aabbMin.x; x <= aabbMax.x; ++x) {
        uint32_t addr = x + y * dims.x + z * dims.x * dims.y;

        // Find the nearest and farthest corners using 
        
        // voxel space       
        float3 lbPt = float3(x, y, z); 
        float3 rtPt = float3(x, y, z) + 1.f;

        // normalized
        lbPt = lbPt / float3(dims);
        rtPt = rtPt / float3(dims);

        // world space
        lbPt = (lbPt * (rt - lb)) + lb;
        rtPt = (rtPt * (rt - lb)) + lb;

        // evaluate the RBF at these two extremes
        float minDensity, maxDensity;
        float mnd = minDist(particle.xyz, lbPt, rtPt);
        float mxd = maxDist(particle.xyz, lbPt, rtPt);

        if (mnd * mnd > radius * radius) minDensity = 0.f;
        else minDensity = evaluate_rbf(mnd, radius);

        if (mxd * mxd > radius * radius) maxDensity = 0.f;
        else maxDensity = evaluate_rbf(mxd, radius);

        // Keep track of the atomic sum of these two
        gprt::atomicAdd32f(record.minMaxVolume, addr * 2 + 0, minDensity);
        gprt::atomicAdd32f(record.minMaxVolume, addr * 2 + 1, maxDensity);
      }
    }
  }
}

GPRT_COMPUTE_PROGRAM(ComputeMajorantGrid, (RayGenData, record), (1,1,1)) {
  int voxelID = DispatchThreadID.x;
  float2 minmax = gprt::load<float2>(record.minMaxVolume, voxelID);
  minmax = min(minmax, float2(record.clampMaxCumulativeValue, record.clampMaxCumulativeValue));
  
  Texture1D colormap = gprt::getTexture1DHandle(record.colormap);
  uint32_t colormapWidth;
  colormap.GetDimensions(colormapWidth);

  // transform data min max to colormap space
  uint32_t start = uint32_t(clamp(minmax.x, 0.f, 1.f) * (colormapWidth - 1));
  uint32_t stop  = uint32_t(clamp(minmax.y, 0.f, 1.f) * (colormapWidth - 1));

  float majorant = 0.f;
  for (uint32_t i = start; i <= stop; ++i) {
    majorant = max(majorant, colormap[i].w);
  }
  gprt::store<float>(record.majorants, voxelID, majorant);
}

struct NullPayload{
  int tmp;
};

GPRT_MISS_PROGRAM(miss, (MissProgData, record), (NullPayload, payload)) {}
