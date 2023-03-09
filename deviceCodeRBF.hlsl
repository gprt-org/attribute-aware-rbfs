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
#include "dda.hlsli"

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
  
  uint3 dimensions;
  int i;
  LCGRand rng;
  float unit;
  float clampMaxCumulativeValue;
  // RayDesc rayDesc;

  float4 albedo;
  float t;

  bool dbg;
  bool visualizeAttributes;

  gprt::Buffer majorants;
  gprt::Texture dmap;
  gprt::Texture cmap;
  gprt::Sampler cmapSampler; 
  gprt::Accel tree;

  float3 lb;
  float3 rt;

  bool lambda(RayDesc ray, int3 cell, float t0, float t1) {    
    float majorant = gprt::load<float>(
      majorants,
      cell.x + cell.y * dimensions.x + cell.z * dimensions.x * dimensions.y
    );

    // skip to the next cell
    if (majorant <= 0.f) return true; 

    RaytracingAccelerationStructure accel = gprt::getAccelHandle(tree);
    SamplerState sampler = gprt::getSamplerHandle(cmapSampler);
    Texture1D colormap = gprt::getTexture1DHandle(cmap);
    Texture1D densitymap = gprt::getTexture1DHandle(dmap);

    float3 org = gridPosToWorld(ray.Origin, lb, rt, dimensions);
    float3 dir = gridDirToWorld(ray.Direction, lb, rt, dimensions);

    // float 
    t = t0;
    for (; i < MAX_DEPTH; ++i) {
      // Sample a distance
      t = t - (log(1.0f - lcg_randomf(rng)) / majorant) * unit;

      // A boundary has been hit
      if (t >= t1) {
        return true; // skip to next cell
      }

      // Update current position
      float3 x = org + t * dir;

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
              gprt::getNumRayTypes(),  // number of ray types
              0,                       // miss type
              pointDesc,               // the ray to trace
              payload                  // the payload IO
      );
      if (payload.count > 0) {
        payload.color /= float(payload.count);
        payload.color.rgb = pow(payload.color.rgb, 1.f / 2.f); // note, only square rooting rbg
      }
      if (clampMaxCumulativeValue > 0.f) payload.density /= clampMaxCumulativeValue;
      float4 densityxf = densitymap.SampleGrad(sampler, payload.density, 0.f, 0.f);
      float density = densityxf.w;

      // allows colormap to hide attributes independent of RBF density
      if (visualizeAttributes) density *= payload.color.w;

      if (lcg_randomf(rng) < density / (majorant)) {
        if (visualizeAttributes)
          albedo = float4(payload.color.rgb, 1.f);
        else
          albedo = float4(densityxf.rgb, 1.f);
        return false; // terminate traversal
      }
    }

    // stop traversal if we hit our sampling limit (avoids lockup)
    return false;
  };
};

GPRT_RAYGEN_PROGRAM(ParticleRBFRayGen, (RayGenData, record)) {
  uint2 pixelID = DispatchRaysIndex().xy;
  uint2 centerID = DispatchRaysDimensions().xy / 2;
  uint2 fbSize = DispatchRaysDimensions().xy;
  const int fbOfs = pixelID.x + fbSize.x * pixelID.y;
  int frameId = record.frameID; // todo, change per frame
  LCGRand rng = get_rng(frameId, DispatchRaysIndex().xy, DispatchRaysDimensions().xy);

  float2 screen = (float2(pixelID) + float2(.5f, .5f)) / float2(fbSize);

  float3 rt = record.globalAABBMax;
  float3 lb = record.globalAABBMin;

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
  tracker.majorants = record.majorants;
  tracker.dimensions = record.ddaDimensions;
  tracker.i = 0;
  tracker.rng = rng;
  tracker.unit = record.unit;
  tracker.visualizeAttributes = record.visualizeAttributes;
  tracker.clampMaxCumulativeValue = record.clampMaxCumulativeValue;
  tracker.albedo = float4(0.f, 0.f, 0.f, 0.f);
  tracker.dbg = false;
  tracker.dmap = record.densitymap;
  tracker.cmap = record.colormap;
  tracker.cmapSampler = record.colormapSampler;
  tracker.tree = record.world;
  tracker.lb = lb;
  tracker.rt = rt;
  
  float4 color = float4(0.f, 0.f, 0.f, 0.f);

  if (tenter < texit) {
    bool dbg = false;
    if (all(pixelID == centerID)) {
      dbg = true;
      tracker.dbg = true;
    }

    #define DDA

    #ifdef DDA

    // if (dbg) {
    //   printf("OUTSIDE t0 %f t1 %f\n", tenter, texit);
    // }

    // compute origin and dir in voxel space
    RayDesc ddaRay;
    ddaRay.Origin = worldPosToGrid(rayDesc.Origin, lb, rt, tracker.dimensions);
    ddaRay.Direction = worldDirToGrid(rayDesc.Direction, lb, rt, tracker.dimensions);
    ddaRay.TMin = 0.f;
    ddaRay.TMax = texit;
    tracker.t = tenter;
    dda3(ddaRay, tracker.dimensions, false, tracker);

    // tracker.dda3(org, dir, texit, tracker.dimensions, false, colormap, colormapSampler, world);
    // tracker.dda3(org, dir, texit, tracker.dimensions, false);
    float4 albedo = tracker.albedo;
    float t = tracker.t;
    float unit = record.unit;
    float majorantExtinction = 1.f; // todo, DDA or something similar

    
    #else
    float unit = record.unit;
    float majorantExtinction = 1.f; // todo, DDA or something similar   
    float t = tenter;

    Texture1D colormap = gprt::getTexture1DHandle(record.colormap);
    SamplerState sampler = gprt::getSamplerHandle(record.colormapSampler);

    float4 albedo = float4(0.f, 0.f, 0.f, 0.f);
    for (int i = 0; i < MAX_DEPTH; ++i) {
      // Sample a distance
      t = t - (log(1.0f - lcg_randomf(rng)) / majorantExtinction) * unit;

      // A boundary has been hit
      if (t >= texit) break;

      // Update current position
      float3 x = rayDesc.Origin + t * rayDesc.Direction;

      // Sample heterogeneous media
      RayDesc pointDesc;
      pointDesc.Origin = x;
      pointDesc.Direction = float3(1.f, 1.f, 1.f); // something non-zero
      pointDesc.TMin = 0.0;
      pointDesc.TMax = 0.0;
      RBFPayload payload; 
      payload.density = 0.f;       
      payload.count = 0;
      payload.rgb = float3(0.f, 0.f, 0.f);
      TraceRay(world,         // the tree
              RAY_FLAG_NONE,   // ray flags
              0xff,                    // instance inclusion mask
              0,                       // ray type
              gprt::getNumRayTypes(),  // number of ray types
              0,                       // miss type
              pointDesc,               // the ray to trace
              payload                  // the payload IO
      );
      if (payload.count > 0) {
        payload.rgb /= float(payload.count);
        payload.rgb = pow(payload.rgb, 1.f / 2.f);
      }
      if (clampMaxCumulativeValue > 0.f) payload.density /= clampMaxCumulativeValue;
      float4 xf = densitymap.SampleGrad(sampler, payload.density, 0.f, 0.f);

      if (lcg_randomf(rng) < xf.w / (majorantExtinction)) {
        if (visualizeAttributes) 
          albedo = float4(payload.rgb, 1.f);
        else
          albedo = float4(xf.rgb, 1.f);
        break;
      }
    }
    #endif

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

      // #undef DDA
      #ifdef DDA

      RayDesc ddaRay;
      ddaRay.Origin = worldPosToGrid(shadowRay.Origin, lb, rt, tracker.dimensions);
      ddaRay.Direction = worldDirToGrid(shadowRay.Direction, lb, rt, tracker.dimensions);
      ddaRay.TMin = 0.f;
      ddaRay.TMax = shadowTExit;
      tracker.t = 0.1f;
      tracker.albedo = float4(0.f, 0.f, 0.f, 0.f);
      tracker.dbg = false;
      dda3(ddaRay, tracker.dimensions, false, tracker);

      
      // if (all(pixelID == centerID)) {
      //   dbg = true;
      //   tracker.dbg = true;
      // }

      // compute origin and dir in voxel space
      // float3 org = worldPosToGrid(shadowRay.Origin, lb, rt, tracker.dimensions);
      // float3 dir = worldDirToGrid(shadowRay.Direction, lb, rt, tracker.dimensions);
      // tracker.t = 0.1f;
      // tracker.albedo = float4(0.f, 0.f, 0.f, 0.f);
      // dda3(org, dir, 1e20f, tracker.dimensions, false, tracker);


      float ts = tracker.t;

      // if (dbg) printf("shadow albedo %f %f %f %f\n", tracker.albedo.x, tracker.albedo.y, tracker.albedo.z, tracker.albedo.w);
      if (tracker.albedo.w > 0.f) {
        visibility = 0.f;
      }


      #else      
      float ts = shadowTEnter;
      for (int i = 0; i < MAX_DEPTH; ++i) {
        ts = ts - (log(1.0f - lcg_randomf(rng)) / majorantExtinction) * unit;
        if (ts >= shadowTExit) break;
        float3 x = shadowRay.Origin + ts * shadowRay.Direction;
        RayDesc pointDesc;
        pointDesc.Origin = x;
        pointDesc.Direction = float3(1.f, 1.f, 1.f); // something non-zero
        pointDesc.TMin = 0.0;
        pointDesc.TMax = 0.0;
        RBFPayload payload; 
        payload.density = 0.f;   
        payload.count = 0;
        payload.density = 0.f;       
        payload.rgb = float3(0.f, 0.f, 0.f);    
        TraceRay(world,         // the tree
                RAY_FLAG_NONE,   // ray flags
                0xff,                    // instance inclusion mask
                0,                       // ray type
                gprt::getNumRayTypes(),  // number of ray types
                0,                       // miss type
                pointDesc,               // the ray to trace
                payload                  // the payload IO
        );
        if (clampMaxCumulativeValue > 0.f) payload.density /= clampMaxCumulativeValue;    
        float4 xf = colormap.SampleGrad(colormapSampler, payload.density, 0.f, 0.f);
        if (lcg_randomf(rng) < xf.w / (majorantExtinction)) {
          visibility = 0.f;
          break;
        }
      }
      #endif
    }

    if (albedo.w == 1.f) {
      color.rgb = albedo.rgb * visibility * (1.f - record.light.ambient) + albedo.rgb * record.light.ambient;
      color.w = 1.f;
    }
  }


  // int pattern = (pixelID.x / 32) ^ (pixelID.y / 32);
  float4 backgroundColor = float4(0.f, 0.f, 0.f, 1.f); //(pattern & 1) ? float4(.1f, .1f, .1f, 1.f) : float4(.2f, .2f, .2f, 1.f);

  color = over(color, backgroundColor);

  float4 prevColor = gprt::load<float4>(record.accumBuffer, fbOfs);
  float4 finalColor = (1.f / float(frameId)) * color + (float(frameId - 1) / float(frameId)) * prevColor;
  gprt::store<float4>(record.accumBuffer, fbOfs, finalColor);

  // exposure and gamma
  finalColor.rgb = finalColor.rgb * record.exposure;
  finalColor.rgb = pow(finalColor.rgb, record.gamma);

  // just the rendered image
  gprt::store(record.imageBuffer, fbOfs, gprt::make_bgra(finalColor));

  // if (any(pixelID == centerID)) {
  //   finalColor.rgb = float3(1.f, 1.f, 1.f) - finalColor.rgb;
  // }

  // Composite on top of everything else our user interface
  Texture2D texture = gprt::getTexture2DHandle(record.guiTexture);
  SamplerState sampler = gprt::getDefaultSampler();
  float4 guiColor = texture.SampleGrad(sampler, screen, float2(0.f, 0.f), float2(0.f, 0.f));
  finalColor = over(guiColor, float4(finalColor.r, finalColor.g, finalColor.b, finalColor.a));
  gprt::store(record.frameBuffer, fbOfs, gprt::make_bgra(finalColor));
}

GPRT_INTERSECTION_PROGRAM(ParticleRBFIntersection, (ParticleData, record)) {
  uint clusterID = PrimitiveIndex();
  uint32_t particlesPerLeaf = record.particlesPerLeaf;
  uint32_t numParticles = record.numParticles;
  
  for (uint32_t i = 0; i < particlesPerLeaf; ++i) {
    uint32_t primID = clusterID * particlesPerLeaf + i;
    if (primID >= numParticles) break;
    
    float4 particle = gprt::load<float4>(record.particles, primID);
    float radius = record.rbfRadius;
    float3 origin = WorldRayOrigin();
    if (distance(particle.xyz, origin) < radius) {
      RBFAttribute attr;
      attr.attribute = particle.w;
      attr.density = evaluate_rbf(particle.xyz, origin, radius);
      ReportHit(0.0f, 0, attr);
    }
  }
}

GPRT_ANY_HIT_PROGRAM(ParticleRBFAnyHit, (ParticleData, record), (RBFPayload, payload), (RBFAttribute, hit_particle)) {
  payload.count += 1;
  payload.density += hit_particle.density;
  
  SamplerState sampler = gprt::getSamplerHandle(record.colormapSampler);
  Texture1D colormap = gprt::getTexture1DHandle(record.colormap);
  float4 color = colormap.SampleGrad(sampler, hit_particle.attribute, 0.f, 0.f);

  payload.color.rgb += pow(color.rgb, 2.f);
  payload.color.a += color.a;
  
  if (record.clampMaxCumulativeValue > 0.f) {
    payload.density = min(payload.density, record.clampMaxCumulativeValue);
    // if we're visualizing attributes, we need to continue accumulating color
    if (!record.visualizeAttributes)
      gprt::acceptHitAndEndSearch(); // early termination of RBF evaluation
  }
  gprt::ignoreHit(); // forces traversal to continue
}