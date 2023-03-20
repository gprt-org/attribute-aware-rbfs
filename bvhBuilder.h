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

/*Modified by Alper Sahistan 08.10.2021
 -Uses owl types
 Modified by Alper Sahistan 12.03.2023
 -Uses gprt compatible types
*/

#pragma once

#include "accelerationUtils.h"
#include <atomic>
#include <memory>
#include "memory.h"
#include <vector>
#include  <stack>
#include <iostream>

namespace Accelerator {

struct BVHBuildNode;

// BVHAccel Forward Declarations
struct BVHPrimitiveInfo;
struct MortonPrimitive;
struct LinearBVHNode {
    float2x4 bounds = float2x4(
        {INF, -INF},
        {INF, -INF},
        {INF, -INF},
        {INF, -INF}
    );
    union {
        int primitivesOffset;   // leaf
        int secondChildOffset;  // interior
    };
    uint16_t nPrimitives;  // 0 -> interior node
    uint8_t axis;          // interior node: xyz
};

// BVHAccel Declarations
class BVHAccel /*: public Aggregate */{
  public:
    // BVHAccel Public Types
    enum class SplitMethod { MMH, HLBVH, Middle, EqualCounts };

    // BVHAccel Public Methods
    BVHAccel(std::vector<float4> particles,
             int maxPrimsInNode = 1,
             SplitMethod splitMethod = SplitMethod::MMH);
    float2x4 WorldBound() const;
    ~BVHAccel();

    /*! write - binary - to given file */
    void saveTo(const std::string &fileName) const;

    /*! read - binary - from given file */
    static std::shared_ptr<BVHAccel> loadFrom(const std::string &fileName);

    /*bool Intersect(const Ray &ray, SurfaceInteraction *isect) const;*/
    bool Intersect(const float3& org, const float3& dir) const;

    BVHAccel(int maxPrimsInNode = 1,
             SplitMethod splitMethod = SplitMethod::MMH);

  private:
    // BVHAccel Private Methods
    BVHBuildNode *recursiveBuild(
        MemoryArena &arena, std::vector<BVHPrimitiveInfo> &primitiveInfo,
        int start, int end, int *totalNodes,
        std::vector<float2> &orderedPrimRanges);
    BVHBuildNode *HLBVHBuild(
        MemoryArena &arena, const std::vector<BVHPrimitiveInfo> &primitiveInfo,
        int *totalNodes,
        std::vector<float2> &orderedPrimRanges) const;
    BVHBuildNode *emitLBVH(
        BVHBuildNode *&buildNodes,
        const std::vector<BVHPrimitiveInfo> &primitiveInfo,
        MortonPrimitive *mortonPrims, int nPrimitives, int *totalNodes,
        std::vector<float2> &orderedPrimRanges,
        std::atomic<int> *orderedPrimsOffset, int bitIndex) const;
    BVHBuildNode *buildUpperSAH(MemoryArena &arena,
                                std::vector<BVHBuildNode *> &treeletRoots,
                                int start, int end, int *totalNodes) const;

    float2 CalculateMajorants(int nodeIndex = 0);

    //bool IntersectHelper(const float3& org, const float3& dir, std::stack<int>& nodeStack) const;
    int flattenBVHTree(BVHBuildNode *node, int *offset);

    void writeTo(std::ostream &out) const;
    void readFrom(std::istream &in);
public:
    // BVHAccel Private Data
    int maxPrimsInNode;
    SplitMethod splitMethod;
    //std::vector<float4> particles;
    LinearBVHNode *nodes = nullptr;
    int nAllocatedNodes;
    std::vector<float2> primitiveRanges;
    float2 globalRange = float2(INF, -INF);
};

std::shared_ptr<BVHAccel> CreateBVHAccelerator(
    std::vector<float4> particles, int maxPrimsInNode);

} // namespace Accelerator
