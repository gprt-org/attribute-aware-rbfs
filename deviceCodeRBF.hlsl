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

GPRT_RAYGEN_PROGRAM(ParticleRBFRayGen, (RayGenData, record)) {
  uint2 pixelID = DispatchRaysIndex().xy;
  uint2 centerID = DispatchRaysDimensions().xy / 2;
  uint2 fbSize = DispatchRaysDimensions().xy;
  const int fbOfs = pixelID.x + fbSize.x * pixelID.y;
  int frameId = record.frameID; // todo, change per frame
  LCGRand rng = get_rng(frameId, DispatchRaysIndex().xy, DispatchRaysDimensions().xy);
  
  float2 screen = (float2(pixelID) + float2(.5f, .5f)) / float2(fbSize);

  float3 rt = gprt::load<float3>(record.globalAABB, 1);
  float3 lb = gprt::load<float3>(record.globalAABB, 0);

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

  float4 color = float4(0.f, 0.f, 0.f, 0.f);
  if (tenter < texit) {
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
        float4 xf = colormap.SampleGrad(sampler, payload.density, 0.f, 0.f);
        if (lcg_randomf(rng) < xf.w / (majorantExtinction)) {
          visibility = 0.f;
          break;
        }
      }
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
  uint primID = PrimitiveIndex();
  float3 center = gprt::load<float4>(record.particles, primID).xyz;
  float radius = record.rbfRadius;
  float3 origin = WorldRayOrigin();
  if (distance(center, origin) < radius) {
    RBFPayload attr;
    attr.test = 42;
    attr.density = evaluate_rbf(center, origin, radius);

    uint2 pixelID = DispatchRaysIndex().xy;
    uint2 centerID = DispatchRaysDimensions().xy / 2;
    // if (all(pixelID == centerID)) {
    //   printf("INTERSECTION Prim %d data %f %d\n", primID, attr.density, attr.test);
    // }
    ReportHit(0.0f, 0, attr);

    // uint2 pixelID = DispatchRaysIndex().xy;
    // uint2 centerID = DispatchRaysDimensions().xy / 2;
    // if (all(pixelID == centerID)) {
    //   printf("data value %f\n", hit_particle.density);
    // }
  }
}

GPRT_ANY_HIT_PROGRAM(ParticleRBFAnyHit, (ParticleData, record), (RBFPayload, payload), (RBFPayload, hit_particle)) {
  uint2 pixelID = DispatchRaysIndex().xy;
  uint2 centerID = DispatchRaysDimensions().xy / 2;

  // float density = asfloat(HitKind());
  // if (all(pixelID == centerID)) {
  //   printf("ANYHIT Prim %d data %f %d\n", PrimitiveIndex(), hit_particle.density, hit_particle.test);
  // }

  // payload = hit_particle;
  // payload.density = density;

  payload.density += hit_particle.density;
  // {
  // }

  
  if (record.clampMaxCumulativeValue > 0.f) {
    payload.density = min(payload.density, record.clampMaxCumulativeValue);
    gprt::acceptHitAndEndSearch(); // early termination of RBF evaluation
  }
  gprt::ignoreHit(); // forces traversal to continue
}
