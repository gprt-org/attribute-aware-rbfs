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

  float3 clusterAABBMin = float3(1.#INF, 1.#INF, 1.#INF);
  float3 clusterAABBMax = float3(-1.#INF, -1.#INF, -1.#INF);
  SamplerState sampler = gprt::getSamplerHandle(record.colormapSampler);
  Texture1D radiusmap = gprt::getTexture1DHandle(record.radiusmap);

  const static float NaN = 0.0f / 0.0f;

  // need this check since particles per frame can vary
  if (clusterID * particlesPerLeaf > numParticles) {
    // negative volume box, should be culled away per spec
    clusterAABBMin = float3(NaN, NaN, NaN);
    clusterAABBMax = float3(NaN, NaN, NaN);
    gprt::store(record.aabbs, 2 * clusterID, clusterAABBMin);
    gprt::store(record.aabbs, 2 * clusterID + 1, clusterAABBMax);
    return;
  }
  
  for (int i = 0; i < particlesPerLeaf; ++i) {
    int primID = DispatchThreadID.x * particlesPerLeaf + i;
    if (primID >= numParticles) break;

    float4 particle = gprt::load<float4>(record.particles, primID);
    float radiusPercent = radiusmap.SampleGrad(sampler, particle.w, 0.f, 0.f).r;

    // if (clusterID == 0) printf("radiusPercent %f\n", radiusPercent);
    if (radiusPercent == 0.f) continue;

    float3 aabbMin = particle.xyz - float3(radius, radius, radius) * radiusPercent;
    float3 aabbMax = particle.xyz + float3(radius, radius, radius) * radiusPercent;
    if (i == 0) {
      clusterAABBMin = aabbMin;
      clusterAABBMax = aabbMax;
    }
    else {
      clusterAABBMin = min(clusterAABBMin, aabbMin);
      clusterAABBMax = max(clusterAABBMax, aabbMax);
    }
  }
  if (any(clusterAABBMin > clusterAABBMax)) {
    // negative volume box, should be culled away per spec
    clusterAABBMin = float3(NaN, NaN, NaN);
    clusterAABBMax = float3(NaN, NaN, NaN);
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

  float3 rt = record.globalAABBMax + record.rbfRadius;
  float3 lb = record.globalAABBMin - record.rbfRadius;
  int3 dims = record.volumeDimensions;

  Texture1D colormap = gprt::getTexture1DHandle(record.colormap);
  SamplerState sampler = gprt::getSamplerHandle(record.colormapSampler);
  float clampMaxCumulativeValue = record.clampMaxCumulativeValue;
  bool visualizeAttributes = record.visualizeAttributes;

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

        float rbf = evaluate_rbf(pt, particle.xyz, radius, record.sigma);
        // float u = density;
        // if (clampMaxCumulativeValue > 0.f) u /= clampMaxCumulativeValue;
        float u = (visualizeAttributes) ? particle.w : rbf;
        float4 cxf = colormap.SampleGrad(sampler, particle.w, 0.f, 0.f);
        float3 color = cxf.rgb;
        float density = (visualizeAttributes) ? cxf.w * rbf : cxf.w;

        
        // if (primID == 0) {
        //   printf("%lu ", addr);
        // }
        // gprt::store<float4>(record.volume, addr, xf);

        if (!record.disableColorCorrection) 
        {
          color = pow(color, 2.2f);
        }

        gprt::atomicAdd32f(record.volume, addr * 4 + 0, color.r);
        gprt::atomicAdd32f(record.volume, addr * 4 + 1, color.g);
        gprt::atomicAdd32f(record.volume, addr * 4 + 2, color.b);
        gprt::atomicAdd32f(record.volume, addr * 4 + 3, density);
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
    if (!record.disableColorCorrection) {
      color.rgb = pow(color, 1.f / 2.2f);
    }
    
    gprt::store<float4>(record.volume, voxelID, color);
  } else {
    // Texture1D colormap = gprt::getTexture1DHandle(record.colormap);
    // SamplerState sampler = gprt::getSamplerHandle(record.colormapSampler);
    float4 xf = float4(0.f, 0.f, 0.f, 0.f);//colormap.SampleGrad(sampler, 0.f, 0.f, 0.f);
    gprt::store<float4>(record.volume, voxelID, xf);
  }
}

GPRT_COMPUTE_PROGRAM(ClearMinMaxGrid, (RayGenData, record), (1,1,1)) {
  int voxelID = DispatchThreadID.x;
  // clearing rbf density range to 0,0, since this will ultimately be a sum.
  // clearing attribute density to inf, 0 since there we're taking min/max
  gprt::store<float4>(record.minMaxVolume, voxelID, float4(0.f, 0.f, 1e20f, 0.f));

  // if (voxelID == 0) printf("TEST\n");
}

GPRT_COMPUTE_PROGRAM(MinMaxRBFBounds, (RayGenData, record), (1,1,1)) {
  float radius = record.rbfRadius; // prior work just set this to some global constant.
  float3 aabbMin;
  float3 aabbMax;
  int particlesPerLeaf = record.particlesPerLeaf;
  
  float attributeMin, attributeMax;

  // need this check since particles per frame can vary
  if (DispatchThreadID.x * particlesPerLeaf > record.numParticles) return;
   
  // compute bounding box
  for (uint32_t i = 0; i < particlesPerLeaf; ++i) {
    int primID = DispatchThreadID.x * particlesPerLeaf + i;
    if (i >= record.numParticles) break;
    
    float4 particle = gprt::load<float4>(record.particles, primID);

    if (i == 0) {
      aabbMin = particle.xyz - float3(radius, radius, radius);
      aabbMax = particle.xyz + float3(radius, radius, radius);
      attributeMin = attributeMax = particle.w;
    } else {
      aabbMin = min(aabbMin, particle.xyz - float3(radius, radius, radius));
      aabbMax = max(aabbMax, particle.xyz + float3(radius, radius, radius));
      attributeMin = min(attributeMin, particle.w);
      attributeMax = max(attributeMax, particle.w);
    }
  }

  float3 rt = record.globalAABBMax + record.rbfRadius;
  float3 lb = record.globalAABBMin - record.rbfRadius;
  int3 dims = record.ddaDimensions;

  float clampMaxCumulativeValue = record.clampMaxCumulativeValue;

  // transform particle into voxel space
  float3 gridAabbMin = worldPosToGrid(aabbMin, lb, rt, dims);;
  float3 gridAabbMax = worldPosToGrid(aabbMax, lb, rt, dims);;

  // just in case points touch sides, clamp
  gridAabbMin = clamp(gridAabbMin, 0, dims - 1);
  gridAabbMax = clamp(gridAabbMax, 0, dims - 1);

  // rasterize particle to all voxels it touches
  for (uint z = gridAabbMin.z; z <= gridAabbMax.z; ++z) {
    for (uint y = gridAabbMin.y; y <= gridAabbMax.y; ++y) {
      for (uint x = gridAabbMin.x; x <= gridAabbMax.x; ++x) {
        uint32_t addr = x + y * dims.x + z * dims.x * dims.y;

        // Find the nearest and farthest corners using 
        
        // voxel space       
        float3 lbPt = float3(x, y, z); 
        float3 rtPt = float3(x, y, z) + 1.f;

        lbPt = gridPosToWorld(lbPt, lb, rt, dims);
        rtPt = gridPosToWorld(rtPt, lb, rt, dims);

        // evaluate the RBF at these two extremes
        float minDensity, maxDensity;

        // note, distances here are squared
        // float mnd = minDist(aabbMin, aabbMax, lbPt, rtPt);
        // float mxd = maxDist(aabbMin, aabbMax, lbPt, rtPt);

        // if (DispatchThreadID.x == 0) {
        //   printf("min %f max %f\n", mnd, mxd);
        // }

        // a little tricky here, minimum distance -> maximum density
        // if (mnd > radius * radius) maxDensity = 0.f;
        // else maxDensity = evaluate_rbf(mnd, radius);

        // if (mxd > radius * radius) minDensity = 0.f;
        // else minDensity = evaluate_rbf(mxd, radius);
        // gprt::atomicAdd32f(record.minMaxVolume, addr * 4 + 0, minDensity);
        // gprt::atomicAdd32f(record.minMaxVolume, addr * 4 + 1, maxDensity);

        // Keep track of the atomic sum of these two
        gprt::atomicAdd32f(record.minMaxVolume, addr * 4 + 0, 0); // ...
        gprt::atomicAdd32f(record.minMaxVolume, addr * 4 + 1, particlesPerLeaf);

        // Keep track of the atomic min/max of these two
        gprt::atomicMin32f(record.minMaxVolume, addr * 4 + 2, attributeMin);
        gprt::atomicMax32f(record.minMaxVolume, addr * 4 + 3, attributeMax);
      }
    }
  }
}

GPRT_COMPUTE_PROGRAM(ComputeMajorantGrid, (RayGenData, record), (1,1,1)) {
  uint voxelID = DispatchThreadID.x;
  float4 minmax = gprt::load<float4>(record.minMaxVolume, voxelID);

  if (record.clampMaxCumulativeValue > 0.f)
    // note, using xy here and leaving zw alone
    minmax.xy = minmax.xy / record.clampMaxCumulativeValue;
  
  Texture1D colormap = gprt::getTexture1DHandle(record.colormap);
  uint32_t colormapWidth;
  colormap.GetDimensions(colormapWidth);
  
  // if (voxelID < 20) printf("min %f max %f\n", minmax.x, minmax.y);

  // if (voxelID < 20) printf("start %u stop %u\n", start, stop);

  // transform data min max to colormap space
  
  
  // note, using "x" and "y", which store density range
  // note, using "z" and "w", which store attribute range
  uint32_t start, stop;
  if (!record.visualizeAttributes) {
    start = uint32_t( floor( clamp(minmax.x, 0.f, 1.f) * colormapWidth)); 
    stop  = uint32_t( ceil(clamp(minmax.y, 0.f, 1.f) * colormapWidth)); 
  } else {
    start = uint32_t( floor( clamp(minmax.z, 0.f, 1.f) * colormapWidth)); 
    stop  = uint32_t( ceil(clamp(minmax.w, 0.f, 1.f) * colormapWidth)); 
  }

  float majorant = 0.f;
  for (uint32_t i = start; i < stop; ++i) {
    majorant = max(majorant, pow(colormap[i].w, 3));
  }
  
  gprt::store<float>(record.majorants, voxelID, majorant);
}

struct [raypayload] NullPayload{
  int tmp;
};

GPRT_MISS_PROGRAM(miss, (MissProgData, record), (NullPayload, payload)) {}
