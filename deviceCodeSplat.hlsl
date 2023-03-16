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
  int frameId = record.frameID; // todo, change per frame
  LCGRand rng = get_rng(frameId, DispatchRaysIndex().xy, DispatchRaysDimensions().xy);
  
  float2 screen = (float2(pixelID) + float2(.5f, .5f)) / float2(fbSize);

  float3 rt = record.globalAABBMax + record.rbfRadius;
  float3 lb = record.globalAABBMin - record.rbfRadius;

  RayDesc rayDesc;
  rayDesc.Origin = record.camera.pos;
  rayDesc.Direction =
      normalize(record.camera.dir_00 + screen.x * record.camera.dir_du + screen.y * record.camera.dir_dv);
  rayDesc.TMin = 0.0;
  rayDesc.TMax = 10000.0;

  float tenter, texit;
  bool hit = aabbIntersection(rayDesc, lb, rt, tenter, texit);

  // for now, assuming one global radius.
  float radius = record.rbfRadius;
  float particlesPerSlab = PARTICLE_BUFFER_SIZE / 8; // just taking this for now

  RaytracingAccelerationStructure world = gprt::getAccelHandle(record.world);
  Texture1D densitymap = gprt::getTexture1DHandle(record.densitymap);
  Texture1D colormap = gprt::getTexture1DHandle(record.colormap);
  SamplerState colormapSampler = gprt::getSamplerHandle(record.colormapSampler);
  float clampMaxCumulativeValue = record.clampMaxCumulativeValue;
  int visualizeAttributes = record.visualizeAttributes;

  float4 result_color = float4(0.f, 0.f, 0.f, 0.f);
  if (tenter < texit) {
    const float slab_spacing = particlesPerSlab * radius;
    float tslab = 0.f;
    
    while (tslab < texit) {
      if (result_color.a > 0.97f) break;

      SplatPayload payload;
      payload.tail = 0;
      rayDesc.TMin = max(tenter, tslab);
      rayDesc.TMax = min(texit, tslab + slab_spacing);

      if (rayDesc.TMax > tenter)
      {
        TraceRay(world,         // the tree
                RAY_FLAG_NONE,   // ray flags
                0xff,                    // instance inclusion mask
                1,                       // ray type
                gprt::getNumRayTypes(),  // number of ray types
                0,                       // miss type
                rayDesc,                 // the ray to trace
                payload                  // the payload IO
        );

        // if (all(pixelID == centerID)) printf("%d\n", payload.tail);
        payload.sort();

        // Integrate depth-sorted list of particles
        for (int i = 0; i < payload.tail; ++i) {
          float4 P = gprt::load<float4>(record.particles, payload.particles[i].id);
          float3 X = rayDesc.Origin + rayDesc.Direction * payload.particles[i].t;
          float drbf = evaluate_rbf(X, P.xyz, radius);
          if (clampMaxCumulativeValue) drbf = min(drbf, clampMaxCumulativeValue);

          // Idea: parameterize density by both a denisty map and a colormap.
          // The denstiy map is parameterized on the RBF density.
          // The colormap density is parameterized by an attribute.
          float4 color_sample = colormap.SampleGrad(colormapSampler, P.w, 0.f, 0.f);
          float4 density_sample = densitymap.SampleGrad(colormapSampler, drbf, 0.f, 0.f);
          float alpha_1msa = ((visualizeAttributes) ? color_sample.w * density_sample.w : density_sample.w) * (1.0 - result_color.a);
          result_color.rgb += alpha_1msa * ((visualizeAttributes) ? color_sample.rgb : density_sample.rgb);
          result_color.a += alpha_1msa;
        }
      }
      tslab += slab_spacing;
    }
  }

  // int pattern = (pixelID.x / 32) ^ (pixelID.y / 32);
  float4 backgroundColor = float4(1.f, 1.f, 1.f, 1.f);//(pattern & 1) ? float4(.1f, .1f, .1f, 1.f) : float4(.2f, .2f, .2f, 1.f);

  result_color = over(result_color, backgroundColor);

  // if (any(pixelID == centerID))
  //   result_color.rgb = float3(1.f, 1.f, 1.f) - result_color.rgb;

    // Composite on top of everything else our user interface
  Texture2D guiTexture = gprt::getTexture2DHandle(record.guiTexture);
  SamplerState guiSampler = gprt::getDefaultSampler();
  float4 guiColor = guiTexture.SampleGrad(guiSampler, screen, float2(0.f, 0.f), float2(0.f, 0.f));
  result_color = over(guiColor, float4(result_color.r, result_color.g, result_color.b, 1.f));

  const int fbOfs = pixelID.x + fbSize.x * pixelID.y;
  gprt::store(record.frameBuffer, fbOfs, gprt::make_bgra(result_color));
}

GPRT_INTERSECTION_PROGRAM(ParticleSplatIntersection, (ParticleData, record)) {
  uint clusterID = PrimitiveIndex();
  uint32_t particlesPerLeaf = record.particlesPerLeaf;
  uint32_t numParticles = record.numParticles;
  
  for (uint32_t i = 0; i < particlesPerLeaf; ++i) {
    uint32_t primID = clusterID * particlesPerLeaf + i;
    if (primID >= numParticles) break;

    float3 center = gprt::load<float4>(record.particles, primID).xyz;
    float radius = record.rbfRadius;
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

GPRT_ANY_HIT_PROGRAM(ParticleSplatAnyHit, (ParticleData, record), (SplatPayload, payload), (ParticleSample, hit_particle)) {
  if (payload.tail < PARTICLE_BUFFER_SIZE) {
    payload.particles[payload.tail++] = hit_particle;
    gprt::ignoreHit();
  }
}