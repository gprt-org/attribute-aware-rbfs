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

struct RBFPayload {
  uint32_t test;
  float density;
};

class ParticleTracker {
  gprt::Buffer majorants;
  uint3 dimensions;
  int i;
  LCGRand rng;
  float unit;
  float clampMaxCumulativeValue;
  RayDesc rayDesc;

  float4 albedo;
  float t;

  bool dbg;


  bool lambda(int3 cell, float t0, float t1, in Texture1D cmap, in SamplerState sampler, in RaytracingAccelerationStructure tree) {    
    if (any(cell >= dimensions)) return true; // skip
    
    float majorant = gprt::load<float>(
      majorants,
      cell.x + cell.y * dimensions.x + cell.z * dimensions.x * dimensions.y
    );

    // if (dbg) {
    //   printf("DDA majorant %f\n", majorant);
    // }

    // skip to the next cell
    if (majorant <= 0.f) return true; 

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
      float3 x = rayDesc.Origin + t * rayDesc.Direction;

      // Sample heterogeneous media
      RayDesc pointDesc;
      pointDesc.Origin = x;
      pointDesc.Direction = float3(1.f, 1.f, 1.f); // something non-zero
      pointDesc.TMin = 0.0;
      pointDesc.TMax = 0.0;
      RBFPayload payload; 
      payload.density = 0.f;       
      TraceRay(tree,                  // the tree
              RAY_FLAG_NONE,           // ray flags
              0xff,                    // instance inclusion mask
              0,                       // ray type
              gprt::getNumRayTypes(),  // number of ray types
              0,                       // miss type
              pointDesc,               // the ray to trace
              payload                  // the payload IO
      );

      if (clampMaxCumulativeValue > 0.f) payload.density /= clampMaxCumulativeValue;
      float4 xf = cmap.SampleGrad(sampler, payload.density, 0.f, 0.f);

      // xf.w = majorant; // TEMPORARY
      // float4 xf = float4(majorant, majorant, majorant, majorant);
      if (lcg_randomf(rng) < xf.w / (majorant)) {
        albedo = float4(xf.rgb, 1.f);
        // if (dbg) printf("Hit at %f, albedo is \n", t, albedo.x, albedo.y, albedo.z, albedo.w);
        return false; // terminate traversal
      }
    }

    if (dbg) {
      printf("ERROR, terminating early\n");
    }

    // stop traversal if we hit our sampling limit (avoids lockup)
    return false;
  };

  void dda3(float3 org,
            float3 dir,
            float tMax,
            uint3 gridSize,
            bool dbg,
            in Texture1D cmap, in SamplerState sampler, in RaytracingAccelerationStructure tree
            )
  {    
    float3 boundsMin = float3(0.f, 0.f, 0.f);
    float3 boundsMax = float3(gridSize);
    float3 floor_org = floor(org);
    float3 floor_org_plus_one = floor_org + 1.f;
    float3 rcp_dir = rcp(dir);
    float3 abs_rcp_dir = abs(rcp_dir);
    float3 f_size = float3(gridSize);
    
    float3 t_lo = (float3(0.f, 0.f, 0.f) - org) * rcp_dir;
    float3 t_hi = (f_size     - org) * rcp_dir;
    float3 t_nr = min(t_lo,t_hi);
    float3 t_fr = max(t_lo,t_hi);
    if (dir.x == 0.f) {
      if (org.x < 0.f || org.x > f_size.x)
        // ray passes by the volume ...
        return;
      t_nr.x = -1.#INF; t_fr.x = 1.#INF;
    }
    if (dir.y == 0.f) {
      if (org.y < 0.f || org.y > f_size.y)
        // ray passes by the volume ...
        return;
      t_nr.y = -1.#INF; t_fr.y = 1.#INF;
    }
    if (dir.z == 0.f) {
      if (org.z < 0.f || org.z > f_size.z)
        // ray passes by the volume ...
        return;
      t_nr.z = -1.#INF; t_fr.z = 1.#INF;
    }
    
    if (dbg) printf("tdir %f %f %f\n",dir.x, dir.y, dir.z);

    float ray_t0 = max(0.f,  max(t_nr.x, max(t_nr.y, t_nr.z)));
    float ray_t1 = min(tMax, min(t_fr.x, min(t_fr.y, t_fr.z)));
    if (dbg) printf("t range for volume %f %f\n",ray_t0,ray_t1);
    if (ray_t0 > ray_t1) return; // no overlap with volume
    
    // compute first cell that ray is in:
    float3 org_in_volume = org + ray_t0 * dir;
    if (dbg) printf("org in vol %f %f %f size %i %i %i\n",
                    org_in_volume.x,
                    org_in_volume.y,
                    org_in_volume.z,
                    gridSize.x,
                    gridSize.y,
                    gridSize.z);
    float3 f_cell = max(float3(0.f, 0.f, 0.f),min(f_size-1.f,floor(org_in_volume)));
    float3 f_cell_end = {
                        dir.x > 0.f ? f_cell.x+1.f : f_cell.x,
                        dir.y > 0.f ? f_cell.y+1.f : f_cell.y,
                        dir.z > 0.f ? f_cell.z+1.f : f_cell.z,
    };
    if (dbg)
      printf("f_cell_end %f %f %f\n",
             f_cell_end.x,
             f_cell_end.y,
             f_cell_end.z);
    
    float3 t_step = abs(rcp_dir);
    if (dbg)
      printf("t_step %f %f %f\n",
             t_step.x,
             t_step.y,
             t_step.z);
    float3 t_next
      = {
         ((dir.x == 0.f)
          ? 1.#INF
          : (abs(f_cell_end.x - org_in_volume.x) * t_step.x)),
         ((dir.y == 0.f)
          ? 1.#INF
          : (abs(f_cell_end.y - org_in_volume.y) * t_step.y)),
         ((dir.z == 0.f)
          ? 1.#INF
          : (abs(f_cell_end.z - org_in_volume.z) * t_step.z))
    };
    if (dbg)
      printf("t_next %f %f %f\n",
             t_next.x,
             t_next.y,
             t_next.z);
    const int3 stop
      = {
         dir.x > 0.f ? (int)gridSize.x : -1,
         dir.y > 0.f ? (int)gridSize.y : -1,
         dir.z > 0.f ? (int)gridSize.z : -1
    };
    if (dbg)
      printf("stop %i %i %i\n",
             stop.x,
             stop.y,
             stop.z);
    const int3 cell_delta
      = {
         (dir.x > 0.f ? +1 : -1),
         (dir.y > 0.f ? +1 : -1),
         (dir.z > 0.f ? +1 : -1)
    };
    if (dbg)
      printf("cell_delta %i %i %i\n",
             cell_delta.x,
             cell_delta.y,
             cell_delta.z);
    int3 cell = int3(f_cell);
    float next_cell_begin = 0.f;
    while (1) {
      float t_closest = min(t_next.x, min(t_next.y, t_next.z));
      const float cell_t0 = ray_t0+next_cell_begin;
      const float cell_t1 = ray_t0+min(t_closest,tMax);
      if (dbg)
        printf("cell %i %i %i dists %f %f %f closest %f t %f %f\n",
               cell.x,cell.y,cell.z,
               t_next.x,t_next.y,t_next.z,
               t_closest,cell_t0,cell_t1);
      bool wantToGoOn = lambda(cell,cell_t0,cell_t1, cmap, sampler, tree);
      if (!wantToGoOn)
        return;
      next_cell_begin = t_closest;
      if (t_next.x == t_closest) {
        t_next.x += t_step.x;
        cell.x += cell_delta.x;
        if (cell.x == stop.x) return;
      }
      if (t_next.y == t_closest) {
        t_next.y += t_step.y;
        cell.y += cell_delta.y;
        if (cell.y == stop.y) return;
      }
      if (t_next.z == t_closest) {
        t_next.z += t_step.z;
        cell.z += cell_delta.z;
        if (cell.z == stop.z) return;
      }
    }
  }
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
  tracker.rayDesc = rayDesc;
  tracker.clampMaxCumulativeValue = record.clampMaxCumulativeValue;
  tracker.albedo = float4(0.f, 0.f, 0.f, 0.f);
  tracker.dbg = false;
  
  float4 color = float4(0.f, 0.f, 0.f, 0.f);

  if (tenter < texit) {
    bool dbg = false;
    if (all(pixelID == centerID)) {
      dbg = true;
      // tracker.dbg = true;
    }

    #define DDA

    #ifdef DDA

    // if (dbg) {
    //   printf("OUTSIDE t0 %f t1 %f\n", tenter, texit);
    // }

    // compute origin and dir in voxel space
    float3 org = worldPosToGrid(rayDesc.Origin, lb, rt, tracker.dimensions);
    float3 dir = worldDirToGrid(rayDesc.Direction, lb, rt, tracker.dimensions);
    tracker.t = tenter;
    tracker.dda3(org, dir, texit, tracker.dimensions, false, colormap, colormapSampler, world);
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
      float4 xf = colormap.SampleGrad(sampler, payload.density, 0.f, 0.f);
      if (lcg_randomf(rng) < xf.w / (majorantExtinction)) {
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

      ParticleTracker tracker;
      tracker.majorants = record.majorants;
      tracker.dimensions = record.ddaDimensions;
      tracker.i = 0;
      tracker.rng = rng;
      tracker.unit = record.unit;
      tracker.rayDesc = shadowRay;
      tracker.clampMaxCumulativeValue = record.clampMaxCumulativeValue;
      tracker.albedo = float4(0.f, 0.f, 0.f, 0.f);
      tracker.dbg = false;

      if (all(pixelID == centerID)) {
        dbg = true;
        tracker.dbg = true;
      }

      // compute origin and dir in voxel space
      float3 org = worldPosToGrid(shadowRay.Origin, lb, rt, tracker.dimensions);
      float3 dir = worldDirToGrid(shadowRay.Direction, lb, rt, tracker.dimensions);
      tracker.t = 0.1f;
      tracker.albedo = float4(0.f, 0.f, 0.f, 0.f);
      tracker.dda3(org, dir, 1e20f, tracker.dimensions, false, colormap, colormapSampler, world);
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



  int pattern = (pixelID.x / 32) ^ (pixelID.y / 32);
  float4 backgroundColor = (pattern & 1) ? float4(.1f, .1f, .1f, 1.f) : float4(.2f, .2f, .2f, 1.f);

  color = over(color, backgroundColor);

  float4 prevColor = gprt::load<float4>(record.accumBuffer, fbOfs);
  float4 finalColor = (1.f / float(frameId)) * color + (float(frameId - 1) / float(frameId)) * prevColor;
  gprt::store<float4>(record.accumBuffer, fbOfs, finalColor);

  if (any(pixelID == centerID)) {
    finalColor.rgb = float3(1.f, 1.f, 1.f) - finalColor.rgb;
  }


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
    
    float3 center = gprt::load<float4>(record.particles, primID).xyz;
    float radius = record.rbfRadius;
    float3 origin = WorldRayOrigin();
    if (distance(center, origin) < radius) {
      RBFPayload attr;
      attr.test = 42;
      attr.density = evaluate_rbf(center, origin, radius);
      ReportHit(0.0f, 0, attr);
    }
  }
}

GPRT_ANY_HIT_PROGRAM(ParticleRBFAnyHit, (ParticleData, record), (RBFPayload, payload), (RBFPayload, hit_particle)) {
  payload.density += hit_particle.density;
  if (record.clampMaxCumulativeValue > 0.f) {
    payload.density = min(payload.density, record.clampMaxCumulativeValue);
    gprt::acceptHitAndEndSearch(); // early termination of RBF evaluation
  }
  gprt::ignoreHit(); // forces traversal to continue
}
