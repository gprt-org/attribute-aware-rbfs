/*
    pbrt source code is Copyright(c) 1998-2016
                        Matt Pharr, Greg Humphreys, and Wenzel Jakob.
    This file is part of pbrt.
    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are
    met:
    - Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    - Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
    IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
    TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
    PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
    HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
    SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
    LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
    DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
    THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
    (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
    OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*Modified by Alper Sahistan 26.09.2021
 -Uses owl types
  Modified by Alper Sahistan 12.03.2023
 -Uses gprt compatible types
*/

#pragma once

#include "accelerationUtils.h"
#include <iostream>
#include <vector>
#include <algorithm>
#include <memory>

namespace Accelerator{

// KdTreeAccel Declarations
// KdTreeAccel Local Declarations
struct KdAccelNode {
    // KdAccelNode Methods
    inline ACCEL_DECL void InitLeaf(uint32_t *primNums, uint32_t np, std::vector<uint32_t> *primitiveIndices);
    inline ACCEL_DECL void InitInterior(uint32_t axis, uint32_t ac, float s) {
        split = s;
        flags = axis;
        aboveChild |= (ac << 2);
    }
    inline ACCEL_DECL float SplitPos() const { return split; }
    inline ACCEL_DECL int nPrimitives() const { return nPrims >> 2; }
    inline ACCEL_DECL int SplitAxis() const { return flags & 3; }
    inline ACCEL_DECL bool IsLeaf() const { return (flags & 3) == 3; }
    inline ACCEL_DECL int AboveChild() const { return aboveChild >> 2; }
    union {
        float split;                 // Interior
        int onePrimitive;            // Leaf
        int primitiveIndicesOffset;  // Leaf
    };
    float2 range;            // Both

  private:
    union {
        int flags;       // Both
        int nPrims;      // Leaf
        int aboveChild;  // Interior
    };
};

struct BoundEdge;
class KdTreeAccel {
  public:
    // KdTreeAccel Public Methods
    KdTreeAccel(std::vector<float4> particles,
                int isectCost = 80, int traversalCost = 1,
                float emptyBonus = 0.5, int maxPrims = 1, int maxDepth = -1);
    float2x3 WorldBound() const { return bounds; }
    ~KdTreeAccel();
    /*bool Intersect(const Ray &ray, SurfaceInteraction *isect) const;*/
    bool Intersect(const float3& org, const float3& dir) const;

    /*! write - binary - to given file */
    void saveTo(const std::string &fileName) const;

    /*! read - binary - from given file */
    static std::shared_ptr<KdTreeAccel> loadFrom(const std::string &fileName);

    static bool same(std::shared_ptr<KdTreeAccel> kd1, std::shared_ptr<KdTreeAccel> kd2);
    
    KdTreeAccel(int isectCost = 80, int traversalCost = 1,
                float emptyBonus = 0.5, int maxPrims = 1, int maxDepth = -1);//just for loading from files
    // KdTreeAccel Private Data
    const int isectCost, traversalCost, maxPrims;
    const float emptyBonus;
    std::vector<float4> particles;
    std::vector<float2> primitiveRanges;
    float2 globalValueRange;
    std::vector<uint32_t> primitiveIndices;
    KdAccelNode *nodes;
    int nAllocatedNodes, nextFreeNode;
    float2x3 bounds;
  private:
    // KdTreeAccel Private Methods
    void writeTo(std::ostream &out) const;
    void readFrom(std::istream &in);
    void buildTree(uint32_t nodeNum, const float2x3 &bounds,
                   const std::vector<float2x3> &primBounds, uint32_t *primNums,
                   uint32_t nprims, uint32_t depth,
                   const std::unique_ptr<BoundEdge[]> edges[3], uint32_t *prims0,
                   uint32_t *prims1, uint32_t badRefines = 0);
                   
    float2 CalculateMajorants(uint32_t nodeIndex = 0);
    
};

struct KdToDo {
    const KdAccelNode *node;
    int nodeIdx;
    float tMin, tMax;
};

std::shared_ptr<KdTreeAccel> CreateKdTreeAccelerator(
    std::vector<float4> particles/*, const ParamSet &ps*/);

} // namespace Accelerator
