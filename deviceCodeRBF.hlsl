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

struct RBFAttribute {
  float density;
  float attribute;
};

struct [raypayload] RBFPayload {
  uint32_t count;
  float density;
  float4 color;
};

class ParticleTracker {
  int i;
  float random;
  LCGRand rng; // for tracking

  float unit;
  float clampMaxCumulativeValue;
  // RayDesc rayDesc;

  float4 albedo;
  float t;

  bool dbg;
  bool visualizeAttributes;
  bool disableColorCorrection;

  gprt::Texture cmap;
  gprt::Texture dmap;
  gprt::Sampler cmapSampler; 
  gprt::Accel tree;

  float3 lb;
  float3 rt;

  float LHS;

  bool shadowRay;
  bool doMarching;


  void track(RayDesc ray) {    
    if (doMarching) {
      stochasticMarching(ray);
    }
    else {
      stochasticTracking(ray);
    }
  }
  
  void stochasticMarching(RayDesc ray) {    
    RaytracingAccelerationStructure accel = gprt::getAccelHandle(tree);
    SamplerState sampler = gprt::getSamplerHandle(cmapSampler);
    Texture1D colormap = gprt::getTexture1DHandle(cmap);
    Texture1D densitymap = gprt::getTexture1DHandle(dmap);

    float RHS = -log(1.f - random * .99);

    // float 
    for (; i < MAX_DEPTH; ++i) {
      t = t + unit; 

      // A boundary has been hit
      if (t >= ray.TMax) {
        return;
      }

      // Update current position
      float3 x = ray.Origin + t * ray.Direction;

      // Sample heterogeneous media
      RayDesc pointDesc;
      pointDesc.Origin = x;
      pointDesc.Direction = float3(1.f, 1.f, 1.f); // something non-zero
      pointDesc.TMin = 0.0;
      pointDesc.TMax = 0.0;
      RBFPayload payload; 
      payload.count = 0;
      payload.density = 0.f;       
      payload.color = float4(0.f, 0.f, 0.f, 0.f);
      TraceRay(accel,                  // the tree
              RAY_FLAG_NONE,           // ray flags
              0xff,                    // instance inclusion mask
              0,                       // ray type
              pc.rayTypeCount,         // number of ray types
              0,                       // miss type
              pointDesc,               // the ray to trace
              payload                  // the payload IO
      );

      if (payload.count > 0 && payload.density > 0.f) {
        payload.color /= payload.density;
        if (!disableColorCorrection) payload.color.rgb = pow(payload.color.rgb, 1.f / 2.2f);
      }
      else {
        continue;
      }
      
      if (clampMaxCumulativeValue > 0.f) payload.density = min(payload.density, clampMaxCumulativeValue) / clampMaxCumulativeValue;

      float density = densitymap.SampleGrad(sampler, payload.density, 0.f, 0.f).r;
      
      if (!visualizeAttributes) {
        float4 densityxf = colormap.SampleGrad(sampler, payload.density, 0.f, 0.f);
        payload.color.rgb = densityxf.rgb;
      }

      LHS += density;

      if (LHS > RHS && !shadowRay) {
        albedo = float4(payload.color.rgb, 1.f);
        return;
      }

      if (shadowRay) {
        albedo.w += 1.f - exp(-density);
        if (albedo.w >= 1.f) {
          albedo.w = 1.f;
          return;
        }
      }
    }
  };

  void stochasticTracking(RayDesc ray) {
    float majorant = 1.f;

    RaytracingAccelerationStructure accel = gprt::getAccelHandle(tree);
    SamplerState sampler = gprt::getSamplerHandle(cmapSampler);
    Texture1D colormap = gprt::getTexture1DHandle(cmap);
    Texture1D densitymap = gprt::getTexture1DHandle(dmap);

    // float 
    t = ray.TMin;
    for (; i < MAX_DEPTH; ++i) {
      // Sample a distance
      t = t - (log(1.0f - lcg_randomf(rng)) / majorant) * unit;

      // A boundary has been hit
      if (t >= ray.TMax) {
        return;
      }

      // Update current position
      float3 x = ray.Origin + t * ray.Direction;

      // Sample heterogeneous media
      RayDesc pointDesc;
      pointDesc.Origin = x;
      pointDesc.Direction = float3(1.f, 1.f, 1.f); // something non-zero
      pointDesc.TMin = 0.0;
      pointDesc.TMax = 0.0;
      RBFPayload payload; 
      payload.count = 0;
      payload.density = 0.f;       
      payload.color = float4(0.f, 0.f, 0.f, 0.f);
      TraceRay(accel,                  // the tree
              RAY_FLAG_NONE,           // ray flags
              0xff,                    // instance inclusion mask
              0,                       // ray type
              pc.rayTypeCount,         // number of ray types
              0,                       // miss type
              pointDesc,               // the ray to trace
              payload                  // the payload IO
      );


      if (payload.count > 0) {
        payload.color /= payload.density;
        if (!disableColorCorrection) payload.color.rgb = pow(payload.color.rgb, 1.f / 2.2f);
      }
      else continue;
      
      if (clampMaxCumulativeValue > 0.f) payload.density = min(payload.density, clampMaxCumulativeValue) / clampMaxCumulativeValue;

      float density = densitymap.SampleGrad(sampler, payload.density, 0.f, 0.f).r;
      
      if (!visualizeAttributes) {
        float4 densityxf = colormap.SampleGrad(sampler, payload.density, 0.f, 0.f);
        payload.color.rgb = densityxf.rgb;
        // density = pow(densityxf.w, 3);
      }

      if (lcg_randomf(rng) < density / (majorant)) {
        albedo = float4(payload.color.rgb, 1.f);
        return;
      }
    }
  };
};

GPRT_RAYGEN_PROGRAM(ParticleRBFRayGen, (RayGenData, record)) {
  uint2 pixelID = DispatchRaysIndex().xy;
  uint2 centerID = DispatchRaysDimensions().xy / 2;
  uint2 fbSize = DispatchRaysDimensions().xy;
  const int fbOfs = pixelID.x + fbSize.x * pixelID.y;
  int accumID = record.accumID;
  int frameID = record.frameID;

  LCGRand rng = get_rng(frameID, DispatchRaysIndex().xy, DispatchRaysDimensions().xy);

  float2 screen = (float2(pixelID) + float2(.5f, .5f)) / float2(fbSize);

  float3 rt = record.globalAABBMax + 2.f * record.rbfRadius;
  float3 lb = record.globalAABBMin - 2.f * record.rbfRadius;

  float diagonal = length(rt - lb);

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
  
  RaytracingAccelerationStructure world = gprt::getAccelHandle(record.world);
  Texture1D colormap = gprt::getTexture1DHandle(record.colormap);
  SamplerState colormapSampler = gprt::getSamplerHandle(record.colormapSampler);
  float clampMaxCumulativeValue = record.clampMaxCumulativeValue;

  ParticleTracker tracker;
  
  // We'll use either spatio-temporal blue noise or white noise to drive
  // the "free flight distance sampling" process. (figuring out how deep
  // a photon travels from the camera into the volume)
  float3 random;
  if (!record.disableBlueNoise) {
    Texture2D stbn = gprt::getTexture2DHandle(record.stbnTexture);
    // 8 by 16 grid of 256x256 blue noise textures 
    uint2 gridCoord = int2(frameID % 8, (frameID / 8) % 16);    
    uint2 texCoord = int2(pixelID.x % 256, pixelID.y % 256);
    uint2 coord = gridCoord * 256 + texCoord;
    random.x = stbn[coord].x;
    tracker.doMarching = true;
  }
  else {
    random.x = lcg_randomf(rng);
    tracker.doMarching = false;
    tracker.rng = rng;
  }

  // We'll use uniformly random numbers to handle unit-scale jitter.
  // We use this to break up the wood grain artifacts from stochastic ray marching.
  random.y = lcg_randomf(rng);
  random.z = lcg_randomf(rng);

  tracker.disableColorCorrection = record.disableColorCorrection;
  tracker.i = 0;
  tracker.LHS = 0;
  tracker.random = random.x;  
  tracker.unit = record.unit;
  tracker.visualizeAttributes = record.visualizeAttributes;
  tracker.clampMaxCumulativeValue = record.clampMaxCumulativeValue;
  tracker.albedo = float4(0.f, 0.f, 0.f, 0.f);
  tracker.dbg = false;
  tracker.cmap = record.colormap;
  tracker.dmap = record.densitymap;
  tracker.cmapSampler = record.colormapSampler;
  tracker.tree = record.world;
  tracker.lb = lb;
  tracker.rt = rt;
  tracker.shadowRay = false;
  float4 color = float4(0.f, 0.f, 0.f, 0.f);

  if (tenter < texit) {
    bool dbg = false;
    if (all(pixelID == centerID)) {
      dbg = true;
      tracker.dbg = true;
    }
    rayDesc.TMax = texit;
    tracker.t = tenter + record.unit * random.y;
    tracker.track(rayDesc);

    float4 albedo = tracker.albedo;
    float t = tracker.t;
    float unit = record.unit;
    float majorantExtinction = 1.f;

    // NEE shadow ray
    //   if we hit something and want to cast a shadow
    float visibility = 1.f;
    if (albedo.a > 0.f && record.light.ambient != 1.f) {
      float shadowTEnter, shadowTExit;
      RayDesc shadowRay;
      shadowRay.Origin = rayDesc.Origin + rayDesc.Direction * t;
      shadowRay.Direction = getLightDirection(record.light.azimuth, record.light.elevation);
      shadowRay.TMin = 0.f; 
      shadowRay.TMax = 10000.f;
      aabbIntersection(shadowRay, lb, rt, shadowTEnter, shadowTExit);
      shadowRay.TMax = shadowTExit;

      tracker.albedo = float4(0.f, 0.f, 0.f, 0.f);
      tracker.dbg = false;
      tracker.LHS = 0;
      tracker.shadowRay = true;
      if (record.disableBlueNoise) {
        tracker.t = 0.f;
      }
      else {
        tracker.t = -record.unit * random.z;
      }
      tracker.track(shadowRay);

      visibility = 1.f - tracker.albedo.w;
    }

    if (albedo.w == 1.f) {
      color.rgb = albedo.rgb * visibility * (1.f - record.light.ambient) + albedo.rgb * record.light.ambient;
      color.w = 1.f;
    }
  }

  float4 backgroundColor = float4(0.f, 0.f, 0.f, 1.f);

  color = over(color, backgroundColor);

  float4 prevColor = gprt::load<float4>(record.accumBuffer, fbOfs);
  float4 finalColor = (1.f / float(accumID)) * color + (float(accumID - 1) / float(accumID)) * prevColor;
  gprt::store<float4>(record.accumBuffer, fbOfs, finalColor);

  // exposure and gamma
  finalColor.rgb = finalColor.rgb * record.exposure;
  finalColor.rgb = pow(finalColor.rgb, record.gamma);

  // just the rendered image
  gprt::store(record.imageBuffer, fbOfs, finalColor);
}

GPRT_INTERSECTION_PROGRAM(ParticleRBFIntersection, (ParticleData, record)) {
  uint clusterID = PrimitiveIndex();
  uint32_t particlesPerLeaf = record.particlesPerLeaf;
  uint32_t numParticles = record.numParticles;
  SamplerState sampler = gprt::getSamplerHandle(record.colormapSampler);
  Texture1D radiusmap = gprt::getTexture1DHandle(record.radiusmap);
  
  for (uint32_t i = 0; i < particlesPerLeaf; ++i) {
    uint32_t primID = clusterID * particlesPerLeaf + i;
    if (primID >= numParticles) break;
    
    float4 particle = gprt::load<float4>(record.particles, primID);
    float radius = record.rbfRadius;
    radius *= radiusmap.SampleGrad(sampler, particle.w, 0.f, 0.f).r;
    float3 origin = WorldRayOrigin();
    if (distance(particle.xyz, origin) < radius) {
      RBFAttribute attr;
      attr.attribute = particle.w;
      attr.density = evaluate_rbf(particle.xyz, origin, radius, record.sigma);
      ReportHit(0.0f, 0, attr);
    }
  }
}

GPRT_ANY_HIT_PROGRAM(ParticleRBFAnyHit, (ParticleData, record), (RBFPayload, payload), (RBFAttribute, hit_particle)) {
  
  SamplerState sampler = gprt::getSamplerHandle(record.colormapSampler);
  Texture1D colormap = gprt::getTexture1DHandle(record.colormap);
  float4 color = colormap.SampleGrad(sampler, hit_particle.attribute, 0.f, 0.f);

  // transparent particle
  if (color.w == 0.f) {
    gprt::ignoreHit();
    return;
  }
  
  payload.count += 1;
  payload.density += hit_particle.density * pow(color.w, 3);

  if (record.disableColorCorrection) {
    payload.color.rgb += color.rgb * hit_particle.density * pow(color.w, 3);
  } else {
    payload.color.rgb += pow(color.rgb, 2.2f) * hit_particle.density * pow(color.w, 3);
  }
  
  if (record.clampMaxCumulativeValue > 0.f && !record.visualizeAttributes) {
    payload.density = min(payload.density, record.clampMaxCumulativeValue); 
    gprt::acceptHitAndEndSearch(); // early termination of RBF evaluation
  }
  gprt::ignoreHit(); // forces traversal to continue
}
