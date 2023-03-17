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

struct Attribute {
  float distance;
  float bug; // on NVIDIA, everything locks up if I don't have at least two attributes... arg...
};

struct [raypayload] Payload {
  float radius;
  float farthest;
  uint32_t count;
  uint32_t K;
};



GPRT_RAYGEN_PROGRAM(OptimizeTreeRayGen, (RayGenData, record)) {
  uint particleID = DispatchRaysIndex().x;

  if (particleID >= record.numParticles) return;

  float4 particle = gprt::load<float4>(record.particles, particleID);
  float radius = gprt::load<float>(record.particleRadii, particleID);

  RaytracingAccelerationStructure world = gprt::getAccelHandle(record.world);

  // Sample heterogeneous media
  RayDesc pointDesc;
  pointDesc.Origin = particle.xyz;
  pointDesc.Direction = float3(1.f, 1.f, 1.f); // something non-zero
  pointDesc.TMin = 0.0;
  pointDesc.TMax = 0.0;

  Payload payload; 
  payload.radius = radius;       
  payload.farthest = radius;
  payload.count = 0;
  payload.K = record.K;
  TraceRay(world,         // the tree
          RAY_FLAG_NONE,   // ray flags
          0xff,                    // instance inclusion mask
          2,                       // ray type
          gprt::getNumRayTypes(),  // number of ray types
          0,                       // miss type
          pointDesc,               // the ray to trace
          payload                  // the payload IO
  );

  if (payload.count > record.K) {    
    // printf("Particle %d Old Radius %f New Radius %f\n", particleID, radius, payload.farthest);
    // mark that we need another iteration
    gprt::store<int>(record.atomicDone, 0, 1);
    radius = radius * .9;// = payload.farthest; //min(radius, payload.farthest * .49f);
  }



  // update radius
  gprt::store<float>(record.newParticleRadii, particleID, radius);
}

GPRT_COMPUTE_PROGRAM(CopyParticleRadii, (RayGenData, record), (1, 1, 1)) {
  int primID = DispatchThreadID.x;
  if (primID >= record.numParticles) return;

  float radius = gprt::load<float>(record.newParticleRadii, primID);
  gprt::store<float>(record.particleRadii, primID, radius);
}

GPRT_INTERSECTION_PROGRAM(OptimizeTreeIntersection, (ParticleData, record)) {
  uint clusterID = PrimitiveIndex();
  uint32_t particlesPerLeaf = record.particlesPerLeaf;
  uint32_t numParticles = record.numParticles;
    
  for (uint32_t i = 0; i < particlesPerLeaf; ++i) {
    uint32_t primID = clusterID * particlesPerLeaf + i;
    if (primID >= numParticles) break;

    if (primID == DispatchRaysIndex().x) continue; // skip the current particle
    
    float4 particle = gprt::load<float4>(record.particles, primID);
    float radius = gprt::load<float>(record.particleRadii, primID);
    
    float3 origin = WorldRayOrigin();
    float dist = distance(particle.xyz, origin);
    if (dist < radius) { // if that particle intersects us
      Attribute attr;
      attr.distance = dist;
      ReportHit(0.0f, 0, attr);
    }
  }
}

GPRT_ANY_HIT_PROGRAM(OptimizeTreeAnyHit, (ParticleData, record), (Payload, payload), (Attribute, hit_particle)) {
  // if that particle is beyond ours
  if (hit_particle.distance >= payload.farthest)  {
    gprt::ignoreHit(); // forces traversal to continue
    return;
  }

  payload.count++;
  if (payload.count > payload.K) {
    gprt::acceptHitAndEndSearch(); // Too many in radius, give up so that we can shrink radii and try again.
  } else {
    gprt::ignoreHit(); // forces traversal to continue
  }
}