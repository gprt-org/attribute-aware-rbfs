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

// struct Payload {
//     uint32_t test;
//     float3 color;// : read(caller) : write(caller, anyhit);
// };

// https://www.sci.utah.edu/~wald/Publications/2019/rtgems/ParticleSplatting.pdf

struct ParticleSample {
  float t;
  uint32_t id;
};

// original paper proposes 31, but should be 15 because registers are 32 bits and
// there are a max of 32 ray payload registers
#define PARTICLE_BUFFER_SIZE 15

struct SplatPayload {
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

// P is the particle center
// X is the intersection point
// r is the radius of the particle
float evaluate_rbf(float3 X, float3 P, float r) {
  return exp( -pow(distance(X, P), 2.f) / pow(r, 2.f) );
}

float4 over(float4 a, float4 b) {
  float4 result;
  result.a = a.a + b.a * (1.f - a.a);
  result.rgb = (a.rgb * a.a + b.rgb * b.a * (1.f - a.a)) / result.a;
  return result;
}

// This ray generation program will kick off the ray tracing process,
// generating rays and tracing them into the world.
GPRT_RAYGEN_PROGRAM(ParticleSplatRayGen, (RayGenData, record)) {

  uint dim;
  // RWByteAddressBuffer buf = buffers[0]; //gprt::getBufferHandle(record.globalAABB);
  // buf.GetDimensions(dim);
  // buffers[record.globalAABB.y].GetDimensions(dim);
  // if (all(DispatchRaysIndex().xy == int2(0,0))) {
  //   printf("buffer dim is %d\n", dim);
  // }

  // // payload.color = float3(0.f, 0.f, 0.f);

  uint2 pixelID = DispatchRaysIndex().xy;
  uint2 centerID = DispatchRaysDimensions().xy / 2;
  uint2 fbSize = DispatchRaysDimensions().xy;
  float2 screen = (float2(pixelID) + float2(.5f, .5f)) / float2(fbSize);

  RayDesc rayDesc;
  rayDesc.Origin = record.camera.pos;
  rayDesc.Direction =
      normalize(record.camera.dir_00 + screen.x * record.camera.dir_du + screen.y * record.camera.dir_dv);
  rayDesc.TMin = 0.0;
  rayDesc.TMax = 10000.0;

  // typical ray AABB intersection test
  float3 dirfrac; // direction is unit direction vector of ray
  dirfrac.x = 1.0f / rayDesc.Direction.x;
  dirfrac.y = 1.0f / rayDesc.Direction.y;
  dirfrac.z = 1.0f / rayDesc.Direction.z;
  // lb is the corner of AABB with minimal coordinates - left bottom, rt is maximal corner
  // origin is origin of ray
  float3 rt = gprt::load<float3>(record.globalAABB, 1);
  float t2 = (rt.x - rayDesc.Origin.x) * dirfrac.x;
  float t4 = (rt.y - rayDesc.Origin.y) * dirfrac.y;
  float t6 = (rt.z - rayDesc.Origin.z) * dirfrac.z;
  float3 lb = gprt::load<float3>(record.globalAABB, 0);
  float t1 = (lb.x - rayDesc.Origin.x) * dirfrac.x;
  float t3 = (lb.y - rayDesc.Origin.y) * dirfrac.y;
  float t5 = (lb.z - rayDesc.Origin.z) * dirfrac.z;
  float tenter = max(max(min(t1, t2), min(t3, t4)), min(t5, t6));
  float texit = min(min(max(t1, t2), max(t3, t4)), max(t5, t6)); // clip hit to near position
  // thit0 = max(thit0, rayDesc.TMin);
  // thit1 = min(thit1, rayDesc.TMax);
  // if tmax < 0, ray (line) is intersecting AABB, but the whole AABB is behind us
  bool hit = true;
  if (texit < 0) { hit = false; }
  // if tmin > tmax, ray doesn't intersect AABB
  if (tenter >= texit) { hit = false; }

  // for now, assuming one global radius.
  float radius = record.rbfRadius;
  float particlesPerSlab = PARTICLE_BUFFER_SIZE / 8; // just taking this for now

  RaytracingAccelerationStructure world = gprt::getAccelHandle(record.world);
  Texture1D colormap = gprt::getTexture1DHandle(record.colormap);
  SamplerState colormapSampler = gprt::getSamplerHandle(record.colormapSampler);

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
                0,                       // ray type
                1,                       // number of ray types
                0,                       // miss type
                rayDesc,                 // the ray to trace
                payload                  // the payload IO
        );

        float val = float(payload.tail) / float(PARTICLE_BUFFER_SIZE - 1);

        // if (payload.tail == 1) {
        //   result_color = float4(1.f, 1.f, 1.f, 1.f);
        // }

        // if (payload.tail == 2) {
        //   result_color = float4(1.f, 0.f, 0.f, 1.f);
        // }

        // if (payload.tail == 3) {
        //   result_color = float4(0.f, 1.f, 0.f, 1.f);
        // }

        // if (payload.tail == 4) {
        //   result_color = float4(0.f, 0.f, 1.f, 1.f);
        // }

        // if (payload.tail >= 5) {
        //   result_color = float4(1.f, 0.f, 1.f, 1.f);
        // }

        // if (payload.tail > 0) {
        
        // }

        // if (all(pixelID == centerID)) printf("%d\n", payload.tail);
        payload.sort();

        // Integrate depth-sorted list of particles
        for (int i = 0; i < payload.tail; ++i) {
          float3 P = gprt::load<float4>(record.particles, payload.particles[i].id).xyz;
          float3 X = rayDesc.Origin + rayDesc.Direction * payload.particles[i].t;
          float drbf = evaluate_rbf(X, P, radius);
          float4 color_sample = colormap.SampleGrad(colormapSampler, drbf, 0.f, 0.f);
          float alpha_1msa = color_sample.w * (1.0 - result_color.a);
          result_color.rgb += alpha_1msa * color_sample.rgb;
          result_color.a += alpha_1msa;
        }

      //  break;
      }
      tslab += slab_spacing;
    }
  }

  int pattern = (pixelID.x / 32) ^ (pixelID.y / 32);
  float4 backgroundColor = (pattern & 1) ? float4(.1f, .1f, .1f, 1.f) : float4(.2f, .2f, .2f, 1.f);

  result_color = over(result_color, backgroundColor);

  if (any(pixelID == centerID))
    result_color.rgb = float3(1.f, 1.f, 1.f) - result_color.rgb;

    // Composite on top of everything else our user interface
  Texture2D guiTexture = gprt::getTexture2DHandle(record.guiTexture);
  SamplerState guiSampler = gprt::getDefaultSampler();
  float4 guiColor = guiTexture.SampleGrad(guiSampler, screen, float2(0.f, 0.f), float2(0.f, 0.f));
  result_color = over(guiColor, float4(result_color.r, result_color.g, result_color.b, 1.f));

  const int fbOfs = pixelID.x + fbSize.x * pixelID.y;
  gprt::store(record.frameBuffer, fbOfs, gprt::make_bgra(result_color));
}

GPRT_INTERSECTION_PROGRAM(ParticleSplatIntersection, (ParticleData, record)) {
  uint primID = PrimitiveIndex();
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

GPRT_ANY_HIT_PROGRAM(ParticleSplatAnyHit, (ParticleData, record), (SplatPayload, payload), (ParticleSample, hit_particle)) {
  if (payload.tail < PARTICLE_BUFFER_SIZE) {
    payload.particles[payload.tail++] = hit_particle;
    gprt::ignoreHit();
  }
}

GPRT_COMPUTE_PROGRAM(GenParticles, (ParticleData, record), (1, 1, 1)) {
  int primID = DispatchThreadID.x;
  float t = float(primID) / float(record.numParticles);
  float4 particle = float4(t * sin(t * 256.f * 3.14f),  t * cos(t * 256.f * 3.14f), 0.f, 1.f);
  gprt::store<float4>(record.particles, primID, particle);
}

// inline float atomicMin(in RWByteAddressBuffer buffer, int addr, float val)
// {
//   float tmp = buffer.Load<float>(addr);
//   uint ret = asuint(tmp);
//   while(val < asfloat(ret)) {
//     uint old = ret;
//     buffer.InterlockedCompareExchange(addr, old, asuint(val), ret);
//     if (ret == old) break;
//   }
//   return asfloat(ret);
// }

// inline float atomicMax(in RWByteAddressBuffer buffer, int addr, float val)
// {
//   float tmp = buffer.Load<float>(addr);
//   uint ret = asuint(tmp);
//   while(val > asfloat(ret)) {
//     uint old = ret;
//     buffer.InterlockedCompareExchange(addr, old, asuint(val), ret);
//     if (ret == old) break;
//   }
//   return asfloat(ret);
// }

GPRT_COMPUTE_PROGRAM(GenRBFBounds, (ParticleData, record), (1, 1, 1)) {
  int primID = DispatchThreadID.x;
  float4 particle = gprt::load<float4>(record.particles, primID);
  float radius = record.rbfRadius; // prior work just set this to some global constant.
  float3 aabbMin = particle.xyz - float3(radius, radius, radius);
  float3 aabbMax = particle.xyz + float3(radius, radius, radius);
  gprt::store(record.aabbs, 2 * primID, aabbMin);
  gprt::store(record.aabbs, 2 * primID + 1, aabbMax);

  gprt::atomicMin32f(record.globalAABB, 0, aabbMin.x);
  gprt::atomicMin32f(record.globalAABB, 1, aabbMin.y);
  gprt::atomicMin32f(record.globalAABB, 2, aabbMin.z);

  gprt::atomicMax32f(record.globalAABB, 3, aabbMax.x);
  gprt::atomicMax32f(record.globalAABB, 4, aabbMax.y);
  gprt::atomicMax32f(record.globalAABB, 5, aabbMax.z);
}

GPRT_MISS_PROGRAM(miss, (MissProgData, record), (ParticleSample, payload)) {
  // uint2 pixelID = DispatchRaysIndex().xy;
  // int pattern = (pixelID.x / 32) ^ (pixelID.y / 32);
  // payload.color = (pattern & 1) ? record.color1 : record.color0;
}
