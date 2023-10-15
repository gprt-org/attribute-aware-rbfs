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

[[vk::push_constant]] PushConstants pc;

GPRT_COMPUTE_PROGRAM(GenRBFBounds, (ParticleData, record), (1024, 1, 1)) {
  uint32_t clusterID = DispatchThreadID.x; 
  if (clusterID >= record.numAABBs) return;

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

  float4 pixelColor;
  if (!record.disableTAA) {  
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

    float lerpAmount = .2f;
    pixelColor = lerp(old, center, lerpAmount);        
    gprt::store(record.taaBuffer, fbOfs, pixelColor);
  } else {
    pixelColor = center;
  }

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
