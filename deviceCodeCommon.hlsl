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
  Texture1D densitymap = gprt::getTexture1DHandle(record.densitymap);
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
        float4 cxf = colormap.SampleGrad(sampler, particle.w, 0.f, 0.f);
        float3 color = cxf.rgb;
        float density = rbf; //densitymap.SampleGrad(sampler, rbf, 0.f, 0.f).r; //rbf; // (visualizeAttributes) ? cxf.w * rbf : cxf.w;
        // float4 dxf = densitymap.SampleGrad(sampler, rbf, 0.f, 0.f);
        if (visualizeAttributes) density *= cxf.w;

        
        // if (primID == 0) {
        //   printf("%lu ", addr);
        // }
        // gprt::store<float4>(record.volume, addr, xf);

        if (!record.disableColorCorrection) 
        {
          color = pow(color, 2.2f);
        }

        color = color * rbf;

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

  Texture1D colormap = gprt::getTexture1DHandle(record.colormap);
  Texture1D densitymap = gprt::getTexture1DHandle(record.densitymap);
  SamplerState sampler = gprt::getSamplerHandle(record.colormapSampler);

  if (count > 0.f) {
    float4 color;
    if (record.visualizeAttributes) {
      color = float4(voxel.xyz / voxel.w, voxel.w);
    }
    else {
      color = float4(voxel.xyz / count, voxel.w);
    }
     
    if (clampMaxCumulativeValue > 0.f) voxel.w /= clampMaxCumulativeValue;
    if (!record.disableColorCorrection) {
      color.rgb = pow(color, 1.f / 2.2f).rgb;
    }
    if (!record.visualizeAttributes) {
      color.rgb = colormap.SampleGrad(sampler, voxel.w, 0.f, 0.f).rgb;
    }
    color.w = densitymap.SampleGrad(sampler, voxel.w, 0.f, 0.f).r;
    
    gprt::store<float4>(record.volume, voxelID, color);
  } else {
    // Texture1D colormap = gprt::getTexture1DHandle(record.colormap);
    // SamplerState sampler = gprt::getSamplerHandle(record.colormapSampler);
    float4 xf = float4(1.f, 1.f, 1.f, 0.f);//colormap.SampleGrad(sampler, 0.f, 0.f, 0.f);
    gprt::store<float4>(record.volume, voxelID, xf);
  }
}

float3 rgb2ycocg(in float3 rgb)
{
    float co = rgb.r - rgb.b;
    float t = rgb.b + co / 2.0;
    float cg = rgb.g - t;
    float y = t + cg / 2.0;
    return float3(y, co, cg);
}


float3 ycocg2rgb(in float3 ycocg)
{
    float t = ycocg.r - ycocg.b / 2.0;
    float g = ycocg.b + t;
    float b = t - ycocg.g / 2.0;
    float r = ycocg.g + b;
    return float3(r, g, b);
}

float3 clipToAABB(in float3 cOld, in float3 cNew, in float3 centre, in float3 halfSize)
{
    if (all(abs(cOld - centre) <= halfSize)) {
        return cOld;
    }
    
    float3 dir = (cNew - cOld);
    float3 near = centre - sign(dir) * halfSize;
    float3 tAll = (near - cOld) / dir;
    float t = 1e20;
    for (int i = 0; i < 3; i++) {
        if (tAll[i] >= 0.0 && tAll[i] < t) {
            t = tAll[i];
        }
    }
    
    if (t >= 1e20) {
		return cOld;
    }
    return cOld + dir * t;
}

GPRT_COMPUTE_PROGRAM(CompositeGui, (RayGenData, record), (1,1,1)) {
  int2 pixelID = DispatchThreadID.xy;
  float2 fragCoord = pixelID + float2(.5f, .5f);
  float2 uv = (fragCoord) / float2(record.fbSize);
  const int fbOfs = pixelID.x + record.fbSize.x * pixelID.y;
  SamplerState sampler = gprt::getDefaultSampler();

  Texture2D imageTexture = gprt::getTexture2DHandle(record.imageTexture);

  // get the neighborhood min / max from this frame's render
  float4 center = imageTexture.SampleGrad(sampler, uv, float2(0.f, 0.f), float2(0.f, 0.f));
  
  float3 minColor = rgb2ycocg(center.rgb);
  float3 maxColor = rgb2ycocg(center.rgb);
  for (int iy = -1; iy <= 1; ++iy)
  {
      for (int ix = -1; ix <= 1; ++ix)
      {          
        if (ix == 0 && iy == 0) continue;

        float2 offsetUV = ((fragCoord + float2(ix, iy)) / record.fbSize.xy);
        float3 color = imageTexture.SampleGrad(sampler, offsetUV, float2(0.f, 0.f), float2(0.f, 0.f)).rgb;
        color = rgb2ycocg(color);
        minColor = min(minColor, color);
        maxColor = max(maxColor, color);
      }
  }

  float4 old = gprt::load<float4>(record.taaPrevBuffer, fbOfs);
  
  // get last frame's pixel and clamp it to the neighborhood of this frame
  old.rgb = ycocg2rgb(max(rgb2ycocg(old.rgb), minColor));
  old.rgb = ycocg2rgb(min(rgb2ycocg(old.rgb), maxColor));

  float lerpAmount = .4f;
  float4 pixelColor = lerp(old, center, lerpAmount);        
  gprt::store(record.taaBuffer, fbOfs, pixelColor);

  // Composite on top of everything else our user interface
  Texture2D guiTexture = gprt::getTexture2DHandle(record.guiTexture);
  float4 guiColor = guiTexture.SampleGrad(sampler, uv, float2(0.f, 0.f), float2(0.f, 0.f));
  pixelColor = over(guiColor, float4(pixelColor.r, pixelColor.g, pixelColor.b, pixelColor.a));
  gprt::store(record.frameBuffer, fbOfs, gprt::make_bgra(pixelColor));
}

struct [raypayload] NullPayload{
  int tmp;
};

GPRT_MISS_PROGRAM(miss, (MissProgData, record), (NullPayload, payload)) {}
