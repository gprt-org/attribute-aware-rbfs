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

#pragma once

#include "gprt.h"

#define MAX_DEPTH 10000
#define EPSILON 2.2204460492503130808472633361816E-16
#define FLOAT_EPSILON 1.19209290e-7F
#define DOUBLE_EPSILON 2.2204460492503131e-16

/* variables available to all programs */

struct ParticleData {
  alignas(4) uint32_t particlesPerLeaf;
  alignas(4) uint32_t numParticles;
  alignas(4) float rbfRadius;
  alignas(4) float clampMaxCumulativeValue;
  alignas(4) float sigma;
  alignas(4) float power;
  
  // A switch to visualize either RBF density or per-particle attributes
  alignas(4) int visualizeAttributes;
  alignas(16) gprt::Texture colormap;
  alignas(16) gprt::Texture radiusmap;
  alignas(16) gprt::Sampler colormapSampler;

  alignas(16) gprt::Buffer particles;
  alignas(16) gprt::Buffer aabbs;

  alignas(4) int disableColorCorrection;
};

struct RayGenData {
  alignas(16) gprt::Texture guiTexture;
  alignas(16) gprt::Buffer frameBuffer;
  alignas(16) gprt::Buffer imageBuffer;
  alignas(16) gprt::Buffer accumBuffer;
  alignas(16) gprt::Buffer stbnBuffer;
  alignas(4) uint32_t frameID;

  alignas(16) gprt::Accel world;
  alignas(16) float3 globalAABBMin;
  alignas(16) float3 globalAABBMax;
  alignas(4) float rbfRadius;
  alignas(4) float clampMaxCumulativeValue;
  alignas(4) float sigma;
  alignas(4) float power;
  alignas(16) gprt::Buffer particles;
  alignas(4) uint32_t particlesPerLeaf;
  alignas(4) uint32_t numParticles;

  // colormap for visualization
  alignas(16) gprt::Texture colormap;
  alignas(16) gprt::Texture radiusmap;
  alignas(16) gprt::Sampler colormapSampler;

  // In the case that we end up rasterizing our particles
  // into a grid.
  alignas(16) gprt::Buffer volume;
  alignas(16) gprt::Buffer volumeCount;
  alignas(16) uint3 volumeDimensions;

  // For DDA
  alignas(16) gprt::Buffer minMaxVolume;
  alignas(16) gprt::Buffer majorants;
  alignas(16) uint3 ddaDimensions;

  // For controling the relative density of delta tracking
  alignas(4) float unit;

  // A switch to visualize either RBF density or per-particle attributes
  alignas(4) int visualizeAttributes;

  // A switch to enable or disable DDA (accelerates volume rendering)
  alignas(4) int useDDA;

  // If true, renders the time that it takes to render the given image
  alignas(4) int showHeatmap;

  alignas(4) int disableColorCorrection;
  
  alignas(4) int disableBlueNoise;

  struct {
    alignas(4) float azimuth;
    alignas(4) float elevation;
    alignas(4) float ambient;
  } light;

  struct {
    alignas(16) float3 pos;
    alignas(16) float3 dir_00;
    alignas(16) float3 dir_du;
    alignas(16) float3 dir_dv;
  } camera;

  alignas(4) int spp;
  alignas(4) float exposure;
  alignas(4) float gamma;

};

/* variables for the miss program */
struct MissProgData {
  alignas(16) float3 color0;
  alignas(16) float3 color1;
};

#ifdef GPRT_DEVICE

/* shared functions */
float3 getLightDirection(float azimuth, float elevation) {
  return float3(
      sin(elevation * 3.14f - 0.5f * 3.14f) * cos(azimuth * 2.f * 3.14f),
      sin(elevation * 3.14f - 0.5f * 3.14f) * sin(azimuth * 2.f * 3.14f),
      cos(elevation * 3.14f - 0.5f * 3.14f));
}

// P is the particle center
// X is the intersection point
// r is the radius of the particle
float evaluate_rbf(float3 X, float3 P, float r, float sigma) {
  return exp(-.5 * pow(distance(X, P), 2.f) / pow(r / sigma, 2.f));
}

float4 over(float4 a, float4 b) {
  float4 result;
  result.a = a.a + b.a * (1.f - a.a);
  result.rgb = (a.rgb * a.a + b.rgb * b.a * (1.f - a.a)) / result.a;
  return result;
}

bool aabbIntersection(in RayDesc rayDesc, float3 aabbMin, float3 aabbMax,
                      out float tenter, out float texit) {
  // typical ray AABB intersection test
  float3 dirfrac; // direction is unit direction vector of ray
  dirfrac.x = 1.0f / rayDesc.Direction.x;
  dirfrac.y = 1.0f / rayDesc.Direction.y;
  dirfrac.z = 1.0f / rayDesc.Direction.z;
  // lb is the corner of AABB with minimal coordinates - left bottom, rt is
  // maximal corner origin is origin of ray
  float3 rt = aabbMax;
  float t2 = (rt.x - rayDesc.Origin.x) * dirfrac.x;
  float t4 = (rt.y - rayDesc.Origin.y) * dirfrac.y;
  float t6 = (rt.z - rayDesc.Origin.z) * dirfrac.z;
  float3 lb = aabbMin;
  float t1 = (lb.x - rayDesc.Origin.x) * dirfrac.x;
  float t3 = (lb.y - rayDesc.Origin.y) * dirfrac.y;
  float t5 = (lb.z - rayDesc.Origin.z) * dirfrac.z;
  tenter = max(max(min(t1, t2), min(t3, t4)), min(t5, t6));
  texit = min(min(max(t1, t2), max(t3, t4)),
              max(t5, t6)); // clip hit to near position
  // truncate interval to ray description tmin/max
  tenter = max(tenter, rayDesc.TMin);
  texit = min(texit, rayDesc.TMax);
  // if texit < 0, ray (line) is intersecting AABB, but the whole AABB is behind
  // us
  bool hit = true;
  if (texit < 0) {
    hit = false;
  }
  // if tenter > texit, ray doesn't intersect AABB
  if (tenter >= texit) {
    hit = false;
  }
  return hit;
}

// minDist computes the square of the distance from a point to a rectangle.
// If the point is contained in the rectangle then the distance is zero.
//
// Implemented per Definition 2 of "Nearest Neighbor Queries" by
// N. Roussopoulos, S. Kelley and F. Vincent, ACM SIGMOD, pages 71-79, 1995.
float minDist(float3 p, float3 rmin, float3 rmax) {
	float sum = 0.0f;
  for (int i = 0; i < 3; ++i) {
    // if point is left of min
    if (p[i] < rmin[i]) {
      // take distance to left wall
      float d = p[i] - rmin[i];
      sum += d * d;
    } 
    // else if point is right of max
    else if (p[i] > rmax[i]) {
      // take distance to right wall
      float d = p[i] - rmax[i];
      sum += d * d;
    } 
    // else point is between the two
    else {
      sum += 0;
    } 
  }
  return sum;
}

// maxDist computes the square of the distance from a point to a rectangle.
float maxDist(float3 p, float3 rmin, float3 rmax) {
	float sum = 0.0f;
  for (int i = 0; i < 3; ++i) {
    // take the max distance for this dimension and sum the square
    float d1 = abs(p[i] - rmin[i]);
    float d2 = abs(p[i] - rmax[i]);
    float d = max(d1, d2);
    sum += d * d;
  }
  return sum;
}

// minMaxDist computes the minimum of the maximum distances from p to points
// on r.  If r is the bounding box of some geometric objects, then there is
// at least one object contained in r within minMaxDist(p, r) of p.
//
// Implemented per Definition 4 of "Nearest Neighbor Queries" by
// N. Roussopoulos, S. Kelley and F. Vincent, ACM SIGMOD, pages 71-79, 1995.
float minMaxDist(float3 p, float3 rmin, float3 rmax) {
	// by definition, MinMaxDist(p, r) =
	// min{1<=k<=n}(|pk - rmk|^2 + sum{1<=i<=n, i != k}(|pi - rMi|^2))
	// where rmk and rMk are defined as follows:
	
	// This formula can be computed in linear time by precomputing
	// S = sum{1<=i<=n}(|pi - rMi|^2).

	float S = 0.0f;
	for (int i = 0; i < 3; ++i) {
		float d = p[i] - ((p[i] >= (rmin[i]+rmax[i]) / 2.f) ? rmin[i] : rmax[i]);
		S += d * d;
	}

	// Compute MinMaxDist using the precomputed S.
	float minimum = 3.402823466e+38F;
	for (int i = 0; i < 3; ++i)
  {
		float d1 = p[i] - ((p[i] >= (rmin[i]+rmax[i]) / 2.f) ? rmin[i] : rmax[i]);
		float d2 = p[i] - ((p[i] <= (rmin[i]+rmax[i]) / 2.f) ? rmin[i] : rmax[i]);
		float d = S - d1*d1 + d2*d2;
		if (d < minimum) {
			minimum = d;
		}
	}

	return minimum;
}

// minDist computes the square of the distance from rectangle A to rectangle B.
// If the rectangles touch then the distance is zero.
// https://gist.github.com/dGr8LookinSparky/bd64a9f5f9deecf61e2c3c1592169c00
float minDist(float3 armin, float3 armax, float3 brmin, float3 brmax) { 
  float sum = 0.f;
  for (int i = 0; i < 3; ++i) {
    // if the right of b is less than the left of a
    if (brmax[i] < armin[i]) {
      // take the distance between these two walls
      float dist = brmax[i] - armin[i];
      // L2 dist
      sum += dist * dist;
    } 
    // if the left of b is greater than the right of a
    else if (brmin[i] > armax[i]) {
      // take the distance between these two walls
      float dist = brmin[i] - armax[i];
      // L2 dist
      sum += dist * dist;
    }
  }
  return sum;
}

// maxDist computes the square of the distance from rectangle A to rectangle B.
float maxDist(float3 armin, float3 armax, float3 brmin, float3 brmax) {
	float sum = 0.0f;
  for (int i = 0; i < 3; ++i) {
    // take the max distance for this dimension and sum the square
    float d1 = abs(armin[i] - brmin[i]);
    float d2 = abs(armin[i] - brmax[i]);
    float d3 = abs(armax[i] - brmin[i]);
    float d4 = abs(armax[i] - brmax[i]);
    float d = max(max(d1, d2), max(d3, d4));
    sum += d * d;
  }
  return sum;
}

float3 worldPosToGrid(float3 worldPt, float3 worldAABBMin, float3 worldAABBMax, uint3 gridDimensions) {
  float3 gridPt = worldPt;
  // translate so that world AABB min is origin
  gridPt = gridPt - worldAABBMin;
  // scale down by the span of the world AABB
  gridPt = gridPt / (worldAABBMax - worldAABBMin);
  // scale up by the grid
  gridPt = gridPt * float3(gridDimensions);
  // assuming grid origin is at 0,0
  return gridPt;
}

float3 gridPosToWorld(float3 gridPt, float3 worldAABBMin, float3 worldAABBMax, uint3 gridDimensions) {
  float3 worldPt = gridPt;
  // scale down by the grid
  worldPt = worldPt / float3(gridDimensions);
  // scale up by the world
  worldPt = worldPt * (worldAABBMax - worldAABBMin);
  // offset back into the world
  worldPt = worldPt + worldAABBMin;
  return worldPt;
}

float3 worldDirToGrid(float3 worldDir, float3 worldAABBMin, float3 worldAABBMax, uint3 gridDimensions) {
  float3 gridDir = worldDir;
  // scale down by the span of the world AABB
  gridDir = gridDir / (worldAABBMax - worldAABBMin);
  // scale up by the grid
  gridDir = gridDir * float3(gridDimensions);
  // assuming grid origin is at 0,0
  return gridDir;
}

float3 gridDirToWorld(float3 gridDir, float3 worldAABBMin, float3 worldAABBMax, uint3 gridDimensions) {
  float3 worldDir = gridDir;
  // scale down by the grid
  worldDir = worldDir / float3(gridDimensions);
  // scale up by the world
  worldDir = worldDir * (worldAABBMax - worldAABBMin);
  return worldDir;
}
#endif
