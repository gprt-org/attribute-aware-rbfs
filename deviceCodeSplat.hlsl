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

[[vk::push_constant]] PushConstants pc;

// https://www.sci.utah.edu/~wald/Publications/2019/rtgems/ParticleSplatting.pdf

struct ParticleSample {
  float t;
  uint32_t id;
};

// original paper proposes 31, but should be 15 because registers are 32 bits and
// there are a max of 32 ray payload registers
#define PARTICLE_BUFFER_SIZE 15

struct [raypayload]  SplatPayload {
  uint32_t tail; // end index of the array
  uint32_t pad;
  ParticleSample particles[PARTICLE_BUFFER_SIZE];   // array

  // sorts all particles from front to back with a basic bubble sort
  void sort() {
    uint N = tail;
    for (uint j = 1; j < N; ++j) {
      for (uint i = 0; i < N - 1; ++i) {
        if (particles[i].t > particles[i + 1].t) {
          ParticleSample tmp = particles[i];
          particles[i] = particles[i + 1];
          particles[i+1] = tmp;
        }
      }
    }
  }
};

GPRT_RAYGEN_PROGRAM(ParticleSplatRayGen, (RayGenData, record)) {
  uint2 pixelID = DispatchRaysIndex().xy;
  uint2 centerID = DispatchRaysDimensions().xy / 2;
  uint2 fbSize = DispatchRaysDimensions().xy;
  int accumID = pc.accumID; // todo, change per frame
  
  float2 screen = (float2(pixelID) + float2(.5f, .5f)) / float2(fbSize);

  float3 rt = record.globalAABBMax + pc.rbfRadius;
  float3 lb = record.globalAABBMin - pc.rbfRadius;

  RayDesc rayDesc;
  rayDesc.Origin = pc.camera.pos;
  rayDesc.Direction =
      normalize(pc.camera.dir_00 + screen.x * pc.camera.dir_du + screen.y * pc.camera.dir_dv);
  rayDesc.TMin = 0.0;
  rayDesc.TMax = 10000.0;

  float tenter, texit;
  bool hit = aabbIntersection(rayDesc, lb, rt, tenter, texit);

  // for now, assuming one global radius.
  float particlesPerSlab = PARTICLE_BUFFER_SIZE / 8; // just taking this for now

  RaytracingAccelerationStructure world = gprt::getAccelHandle(pc.world);
  Texture1D colormap = gprt::getTexture1DHandle(pc.colormap);
  Texture1D densitymap = gprt::getTexture1DHandle(pc.densitymap);
  SamplerState colormapSampler = gprt::getDefaultSampler();

  // float4 result_color = float4(1.f, 1.f, 1.f, 0.f);
  float4 result_color = float4(0.f, 0.f, 0.f, 0.f);
  if (tenter < texit) {
    const float slab_spacing = particlesPerSlab * pc.unit;
    float tslab = 0.f;
    
    while (tslab < texit) {
      if (result_color.a > 0.99f) break;

      SplatPayload payload;
      payload.tail = 0;
      rayDesc.TMin = max(tenter, tslab);
      rayDesc.TMax = min(texit, tslab + slab_spacing);

      if (rayDesc.TMax > tenter)
      {
        TraceRay(world,        // the tree
                RAY_FLAG_NONE, // ray flags
                0xff,          // instance inclusion mask
                1,             // ray type
                2,             // number of ray types
                0,             // miss type
                rayDesc,       // the ray to trace
                payload        // the payload IO
        );

        // if (all(pixelID == centerID)) printf("%d\n", payload.tail);
        payload.sort();

        // Integrate depth-sorted list of particles
        for (int i = 0; i < payload.tail; ++i) {
          float4 P = gprt::load<float4>(pc.particles, payload.particles[i].id);
          float3 X = rayDesc.Origin + rayDesc.Direction * payload.particles[i].t;
          float drbf = evaluate_rbf(X, P.xyz, pc.rbfRadius, 3.f);

          float4 color;
          if (pc.visualizeAttributes) {
            color = colormap.SampleGrad(colormapSampler, P.w, 0.f, 0.f);
            drbf = densitymap.SampleGrad(colormapSampler, drbf, 0.f, 0.f).r;
            color.w *= drbf;
          }
          else {
            color.rgb = colormap.SampleGrad(colormapSampler, drbf, 0.f, 0.f).rgb;
            color.w = densitymap.SampleGrad(colormapSampler, drbf, 0.f, 0.f).r;
          }

          color.w *= exp(-pc.unit * particlesPerSlab);

          result_color = over(result_color, color);
           
          // float alpha_1msa = color.a * (1.0 - result_color.a);
          // result_color.rgb += alpha_1msa * color.rgb;
          // result_color.a += alpha_1msa;
        }
      }
      tslab += slab_spacing;
    }
  }

  const int fbOfs = pixelID.x + fbSize.x * pixelID.y;
  // int pattern = (pixelID.x / 32) ^ (pixelID.y / 32);
  float4 backgroundColor = float4(0.f, 0.f, 0.f, 1.f);//(pattern & 1) ? float4(.1f, .1f, .1f, 1.f) : float4(.2f, .2f, .2f, 1.f);
  // float4 backgroundColor = float4(0.f, 0.f, 0.f, 1.f);//(pattern & 1) ? float4(.1f, .1f, .1f, 1.f) : float4(.2f, .2f, .2f, 1.f);

  result_color = over(result_color, backgroundColor);

  gprt::store(record.imageBuffer, fbOfs, result_color);
}

GPRT_INTERSECTION_PROGRAM(ParticleSplatIntersection, (UnusedRecord, record)) {
  uint clusterID = PrimitiveIndex();
  uint32_t particlesPerLeaf = pc.particlesPerLeaf;
  uint32_t numParticles = pc.numParticles;
  
  for (uint32_t i = 0; i < particlesPerLeaf; ++i) {
    uint32_t primID = clusterID * particlesPerLeaf + i;
    if (primID >= numParticles) break;

    float3 center = gprt::load<float4>(pc.particles, primID).xyz;
    float radius = pc.rbfRadius;
    float3 origin = WorldRayOrigin();
    float3 direction = WorldRayDirection();
    float t = distance(center, origin);
    float3 sample_pos = origin + t * direction;
    float3 offset = center - sample_pos;

    float tmin = RayTMin();
    float tmax = RayTCurrent();

    uint2 pixelID = DispatchRaysIndex().xy;
    uint2 centerID = DispatchRaysDimensions().xy / 2;

    if ((dot(offset, offset) < radius * radius) && (tmin <= t) && (t < tmax)) 
    {
      ParticleSample attr;
      attr.t = t;
      attr.id = primID;
      ReportHit(t, /*hitKind*/ 0, attr);
    }
  }
}

GPRT_ANY_HIT_PROGRAM(ParticleSplatAnyHit, (UnusedRecord, record), (SplatPayload, payload), (ParticleSample, hit_particle)) {
  if (payload.tail < PARTICLE_BUFFER_SIZE) {
    payload.particles[payload.tail++] = hit_particle;
    gprt::ignoreHit();
  }
}