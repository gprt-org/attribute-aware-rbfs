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

[[vk::push_constant]] BoundsConstants pc;

GPRT_COMPUTE_PROGRAM(GenRBFBounds, (UnusedRecord, record), (1024, 1, 1)) {
  uint32_t clusterID = DispatchThreadID.x; 
  if (clusterID >= pc.numAABBs) return;

  uint32_t particlesPerLeaf = pc.particlesPerLeaf;
  uint32_t numParticles = pc.numParticles;
  float radius = pc.rbfRadius;

  float3 clusterAABBMin = float3(1.#INF, 1.#INF, 1.#INF);
  float3 clusterAABBMax = float3(-1.#INF, -1.#INF, -1.#INF);
  SamplerState sampler = gprt::getDefaultSampler();
  Texture1D radiusmap = gprt::getTexture1DHandle(pc.radiusmap);

  const static float NaN = 0.0f / 0.0f;

  // need this check since particles per frame can vary
  if (clusterID * particlesPerLeaf > numParticles) {
    // negative volume box, should be culled away per spec
    clusterAABBMin = float3(NaN, NaN, NaN);
    clusterAABBMax = float3(NaN, NaN, NaN);
    gprt::store(pc.aabbs, 2 * clusterID, clusterAABBMin);
    gprt::store(pc.aabbs, 2 * clusterID + 1, clusterAABBMax);
    return;
  }
  
  for (int i = 0; i < particlesPerLeaf; ++i) {
    int primID = DispatchThreadID.x * particlesPerLeaf + i;
    if (primID >= numParticles) break;

    float4 particle = gprt::load<float4>(pc.particles, primID);
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
  gprt::store(pc.aabbs, 2 * clusterID, clusterAABBMin);
  gprt::store(pc.aabbs, 2 * clusterID + 1, clusterAABBMax);
}
