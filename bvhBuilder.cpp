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

#include "bvhBuilder.h"
#include <algorithm>
#include <limits>
#include "importers/import_arborx.h"
// #include "/usr/include/trilinos/Kokkos_Core.hpp"

#include <chrono>
using namespace std::chrono;

namespace Accelerator
{

    int treeBytes = 0;
    float totalPrimitives = 0.0f, totalLeafNodes = 0.0f;
    int interiorNodes = 0;
    int leafNodes = 0;

    /*STAT_MEMORY_COUNTER("Memory/BVH tree", treeBytes);
    STAT_RATIO("BVH/Primitives per leaf node", totalPrimitives, totalLeafNodes);
    STAT_COUNTER("BVH/Interior nodes", interiorNodes);
    STAT_COUNTER("BVH/Leaf nodes", leafNodes);*/

    // BVHAccel Local Declarations
    struct BVHPrimitiveInfo
    {
        BVHPrimitiveInfo() {}
        BVHPrimitiveInfo(size_t primitiveNumber, const float3 &particle)
            : primitiveNumber(primitiveNumber),
              centroid(particle) {}
        size_t primitiveNumber;
        float3 centroid;
    };

    struct BVHBuildNode
    {
        // BVHBuildNode Public Methods
        void InitLeaf(int first, int n, const float2x3 &b)
        {
            firstPrimOffset = first;
            nPrimitives = n;
            bounds = b;
            children[0] = children[1] = nullptr;
            ++leafNodes;
            ++totalLeafNodes;
            totalPrimitives += n;
            printf("Leaf node with %d primitives created\n", n);
        }
        void InitInterior(int axis, BVHBuildNode *c0, BVHBuildNode *c1)
        {
            children[0] = c0;
            children[1] = c1;

            bounds = c0->bounds;
            bounds = extendedBounds(bounds, c1->bounds);

            splitAxis = axis;
            nPrimitives = 0;
            ++interiorNodes;
        }
        float2x3 bounds;
        BVHBuildNode *children[2];
        int splitAxis, firstPrimOffset, nPrimitives;
    };

    struct MortonPrimitive
    {
        int primitiveIndex;
        uint32_t mortonCode;
    };

    struct LBVHTreelet
    {
        int startIndex, nPrimitives;
        BVHBuildNode *buildNodes;
    };

    // BVHAccel Utility Functions
    inline uint32_t LeftShift3(uint32_t x)
    {
        // CHECK_LE(x, (1 << 10));
        if (x == (1 << 10))
            --x;
#ifdef PBRT_HAVE_BINARY_CONSTANTS
        x = (x | (x << 16)) & 0b00000011000000000000000011111111;
        // x = ---- --98 ---- ---- ---- ---- 7654 3210
        x = (x | (x << 8)) & 0b00000011000000001111000000001111;
        // x = ---- --98 ---- ---- 7654 ---- ---- 3210
        x = (x | (x << 4)) & 0b00000011000011000011000011000011;
        // x = ---- --98 ---- 76-- --54 ---- 32-- --10
        x = (x | (x << 2)) & 0b00001001001001001001001001001001;
        // x = ---- 9--8 --7- -6-- 5--4 --3- -2-- 1--0
#else
        x = (x | (x << 16)) & 0x30000ff;
        // x = ---- --98 ---- ---- ---- ---- 7654 3210
        x = (x | (x << 8)) & 0x300f00f;
        // x = ---- --98 ---- ---- 7654 ---- ---- 3210
        x = (x | (x << 4)) & 0x30c30c3;
        // x = ---- --98 ---- 76-- --54 ---- 32-- --10
        x = (x | (x << 2)) & 0x9249249;
        // x = ---- 9--8 --7- -6-- 5--4 --3- -2-- 1--0
#endif // PBRT_HAVE_BINARY_CONSTANTS
        return x;
    }

    inline uint32_t EncodeMorton3(const float3 &v)
    {
        /*CHECK_GE(v.x, 0);
        CHECK_GE(v.y, 0);
        CHECK_GE(v.z, 0);*/
        return (LeftShift3(v.z) << 2) | (LeftShift3(v.y) << 1) | LeftShift3(v.x);
    }

    static void RadixSort(std::vector<MortonPrimitive> *v)
    {
        std::vector<MortonPrimitive> tempVector(v->size());
        constexpr int bitsPerPass = 6;
        constexpr int nBits = 30;
        static_assert((nBits % bitsPerPass) == 0,
                      "Radix sort bitsPerPass must evenly divide nBits");
        constexpr int nPasses = nBits / bitsPerPass;

        for (int pass = 0; pass < nPasses; ++pass)
        {
            // Perform one pass of radix sort, sorting _bitsPerPass_ bits
            int lowBit = pass * bitsPerPass;

            // Set in and out vector pointers for radix sort pass
            std::vector<MortonPrimitive> &in = (pass & 1) ? tempVector : *v;
            std::vector<MortonPrimitive> &out = (pass & 1) ? *v : tempVector;

            // Count number of zero bits in array for current radix sort bit
            constexpr int nBuckets = 1 << bitsPerPass;
            int bucketCount[nBuckets] = {0};
            constexpr int bitMask = (1 << bitsPerPass) - 1;
            for (const MortonPrimitive &mp : in)
            {
                int bucket = (mp.mortonCode >> lowBit) & bitMask;
                /*CHECK_GE(bucket, 0);
                CHECK_LT(bucket, nBuckets);*/
                ++bucketCount[bucket];
            }

            // Compute starting index in output array for each bucket
            int outIndex[nBuckets];
            outIndex[0] = 0;
            for (int i = 1; i < nBuckets; ++i)
                outIndex[i] = outIndex[i - 1] + bucketCount[i - 1];

            // Store sorted values in output array
            for (const MortonPrimitive &mp : in)
            {
                int bucket = (mp.mortonCode >> lowBit) & bitMask;
                out[outIndex[bucket]++] = mp;
            }
        }
        // Copy final result from _tempVector_, if needed
        if (nPasses & 1)
            std::swap(*v, tempVector);
    }

    float2 BVHAccel::CalculateMajorants(int nodeIndex)
    {
        auto node = nodes[nodeIndex];
        float2 absoluteRange = float2(INF, -INF);
        if (node.bounds.row(0).w >= globalRange.x &&
            node.bounds.row(1).w <= globalRange.y) // prune tree traversal if the values are initialized
        {
            float2 rtn = float2(INF, -INF);
            rtn = extendedBounds(rtn, node.bounds.row(0).w);
            rtn = extendedBounds(rtn, node.bounds.row(1).w);
            return rtn;
        }
        else if (node.nPrimitives != 0) // leaf:base case
        {
            for (size_t i = node.primitivesOffset;
                 i < node.primitivesOffset + node.nPrimitives; i++)
            {
                absoluteRange = extendedBounds(absoluteRange, primitiveRanges[i]);
            }
        }
        else
        {
            absoluteRange = CalculateMajorants(nodeIndex + 1);
            absoluteRange = extendedBounds(absoluteRange, CalculateMajorants(node.secondChildOffset));
        }
        node.bounds = float2x4(
            {node.bounds.row(0).x, node.bounds.row(1).x},
            {node.bounds.row(0).y, node.bounds.row(1).y},
            {node.bounds.row(0).z, node.bounds.row(1).z},
            float2(absoluteRange.x, absoluteRange.y));
        // node.bounds.row(0).w = absoluteRange.x;
        // node.bounds.row(1).w = absoluteRange.y;
        return absoluteRange;
    }

    // BVHAccel Method Definitions
    BVHAccel::BVHAccel(std::vector<float4> particles,
                       int maxPrimsInNode, SplitMethod splitMethod)
        : maxPrimsInNode(/*std::min*/ (/*255,*/ maxPrimsInNode)),
          splitMethod(splitMethod)
    {
        size_t numPrim = 0;
        // ProfilePhase _(Prof::AccelConstruction);
        if ((numPrim = particles.size()) <= 0)
            return;
        // Build BVH from _primitives_

        // Initialize _primitiveInfo_ array for primitives
        std::vector<BVHPrimitiveInfo> primitiveInfo(numPrim);

        int i = 0;
        for (const auto &particle : particles)
        {
            // float2x3 b = float2x3(
            //     float2(particle.x, particle.x),
            //     float2(particle.y, particle.y),
            //     float2(particle.z, particle.z));
            float2 valueRange = float2(particle.w, particle.w);

            primitiveInfo[i] = {size_t(i), particle.xyz()};
            i = i + 1;
            primitiveRanges.push_back(valueRange);
            globalRange = extendedBounds(globalRange, valueRange);
        }
        // Build BVH tree for primitives using _primitiveInfo_
        MemoryArena arena(1024 * 1024);
        nAllocatedNodes = 0;
        std::vector<float2> orderedPrimRanges;
        orderedPrimRanges.reserve(numPrim);
        BVHBuildNode *root;
        if (splitMethod == SplitMethod::HLBVH)
            root = HLBVHBuild(arena, primitiveInfo, &nAllocatedNodes, orderedPrimRanges);
        else
            root = recursiveBuild(arena, primitiveInfo, 0, numPrim,
                                  &nAllocatedNodes, orderedPrimRanges);

        primitiveRanges.swap(orderedPrimRanges);
        primitiveInfo.resize(0);
        printf("BVH created with %d nodes for %d "
               "primitives (%.2f MB), arena allocated %.2f MB\n",
               nAllocatedNodes, (int)primitiveRanges.size(),
               float(nAllocatedNodes * sizeof(LinearBVHNode)) /
                   (1024.f * 1024.f),
               float(arena.TotalAllocated()) /
                   (1024.f * 1024.f));

        // Compute representation of depth-first traversal of BVH tree
        treeBytes += nAllocatedNodes * sizeof(LinearBVHNode) + sizeof(*this) +
                     primitiveRanges.size() * sizeof(primitiveRanges[0]);
        nodes = AllocAligned<LinearBVHNode>(nAllocatedNodes);
        int offset = 0;
        flattenBVHTree(root, &offset);

        for (size_t i = 0; i < nAllocatedNodes; i++)
        {
            nodes[i].bounds = float2x4(
                {nodes[i].bounds.row(0).x, nodes[i].bounds.row(1).x},
                {nodes[i].bounds.row(0).y, nodes[i].bounds.row(1).y},
                {nodes[i].bounds.row(0).z, nodes[i].bounds.row(1).z},
                float2(globalRange.x - 1.0f, globalRange.y + 1.0f));

            // nodes[i].bounds.row(0).w = globalRange.x -1.0f;
            // nodes[i].bounds.row(1).w = globalRange.y +1.0f;
        }

        CalculateMajorants();
        // for (size_t i = 0; i < nAllocatedNodes; i++)
        // {
        //     printf("Range: %f %f \n", nodes[i].bounds.lower.w, nodes[i].bounds.upper.w );
        // }
        // CHECK_EQ(totalNodes, offset);
    }

    float2x4 BVHAccel::WorldBound() const
    {
        return nodes ? nodes[0].bounds : float2x4(
            {INF, -INF},
            {INF, -INF},
            {INF, -INF},
            {INF, -INF}
        );
    }

    struct BucketInfo
    {
        int count = 0;
        float2x3 bounds = float2x3(
            {INF, -INF},
            {INF, -INF},
            {INF, -INF});
        float2 primitiveRange = float2(INF, -INF);
    };

    BVHBuildNode *BVHAccel::recursiveBuild(
        MemoryArena &arena, std::vector<BVHPrimitiveInfo> &primitiveInfo, int start,
        int end, int *totalNodes,
        std::vector<float2> &orderedPrimRanges)
    {
        printf("recursiveBuild start: %d, end: %d, totalNodes: %d, orderedPrimRanges size: %d, primitiveRanges size: %d\n",
         start, end, *totalNodes, orderedPrimRanges.size(), primitiveRanges.size());
        // CHECK_NE(start, end);
        BVHBuildNode *node = arena.Alloc<BVHBuildNode>();
        (*totalNodes)++;
        // Compute bounds of all primitives in BVH node
        float2x3 bounds = float2x3(
            {INF, -INF},
            {INF, -INF},
            {INF, -INF});
        for (int i = start; i < end; ++i)
            bounds = extendedBounds(bounds, primitiveInfo[i].centroid);
        int nPrimitives = end - start;
        if (nPrimitives == 1)
        {
            // Create leaf _BVHBuildNode_
            int firstPrimOffset = orderedPrimRanges.size();
            for (int i = start; i < end; ++i)
            {
                int primNum = primitiveInfo[i].primitiveNumber;
                orderedPrimRanges.push_back(primitiveRanges[primNum]);
            }
            node->InitLeaf(firstPrimOffset, nPrimitives, bounds);
            return node;
        }
        else
        {
            // Compute bound of primitive centroids, choose split dimension _dim_
            float2x3 centroidBounds = float2x3(
                {INF, -INF},
                {INF, -INF},
                {INF, -INF});
            for (int i = start; i < end; ++i)
                centroidBounds = extendedBounds(centroidBounds, primitiveInfo[i].centroid);
            int dim = MaximumExtent(centroidBounds);

            // Partition primitives into two sets and build children
            int mid = (start + end) / 2;
            if (centroidBounds.row(1)[dim] == centroidBounds.row(0)[dim])
            {
                // Create leaf _BVHBuildNode_
                int firstPrimOffset = orderedPrimRanges.size();
                for (int i = start; i < end; ++i)
                {
                    int primNum = primitiveInfo[i].primitiveNumber;
                    orderedPrimRanges.push_back(primitiveRanges[primNum]);
                }
                node->InitLeaf(firstPrimOffset, nPrimitives, bounds);
                return node;
            }
            else
            {
                // Partition primitives based on _splitMethod_
                switch (splitMethod)
                {
                case SplitMethod::Middle:
                {
                    // Partition primitives through node's midpoint
                    float pmid =
                        (centroidBounds.row(0)[dim] + centroidBounds.row(1)[dim]) / 2;
                    BVHPrimitiveInfo *midPtr = std::partition(
                        &primitiveInfo[start], &primitiveInfo[end - 1] + 1,
                        [dim, pmid](const BVHPrimitiveInfo &pi)
                        {
                            return pi.centroid[dim] < pmid;
                        });
                    mid = midPtr - &primitiveInfo[0];
                    // For lots of prims with large overlapping bounding boxes, this
                    // may fail to partition; in that case don't break and fall
                    // through
                    // to EqualCounts.
                    if (mid != start && mid != end)
                        break;
                }
                case SplitMethod::EqualCounts:
                {
                    // Partition primitives into equally-sized subsets
                    mid = (start + end) / 2;
                    std::nth_element(&primitiveInfo[start], &primitiveInfo[mid],
                                     &primitiveInfo[end - 1] + 1,
                                     [dim](const BVHPrimitiveInfo &a,
                                           const BVHPrimitiveInfo &b)
                                     {
                                         return a.centroid[dim] < b.centroid[dim];
                                     });
                    break;
                }
                case SplitMethod::MMH:
                default:
                {
                    // Partition primitives using approximate SAH
                    if (nPrimitives <= 2)
                    {
                        // Partition primitives into equally-sized subsets
                        mid = (start + end) / 2;
                        std::nth_element(&primitiveInfo[start], &primitiveInfo[mid],
                                         &primitiveInfo[end - 1] + 1,
                                         [dim](const BVHPrimitiveInfo &a,
                                               const BVHPrimitiveInfo &b)
                                         {
                                             return a.centroid[dim] <
                                                    b.centroid[dim];
                                         });
                    }
                    else
                    {
                        // Allocate _BucketInfo_ for SAH partition buckets
                        constexpr int nBuckets = 12;
                        BucketInfo buckets[nBuckets];

                        // Initialize _BucketInfo_ for SAH partition buckets
                        for (int i = start; i < end; ++i)
                        {
                            int b = nBuckets *
                                    Offset(centroidBounds,
                                           primitiveInfo[i].centroid)[dim];
                            if (b == nBuckets)
                                b = nBuckets - 1;
                            /*CHECK_GE(b, 0);
                            CHECK_LT(b, nBuckets);*/
                            buckets[b].count++;
                            buckets[b].bounds = extendedBounds(buckets[b].bounds, primitiveInfo[i].centroid);
                            buckets[b].primitiveRange = extendedBounds(buckets[b].primitiveRange, primitiveRanges[primitiveInfo[i].primitiveNumber]);
                        }

                        // Compute costs for splitting after each bucket
                        float cost[nBuckets - 1];
                        for (int i = 0; i < nBuckets - 1; ++i)
                        {
                            float2x3 b0, b1;
                            b0 = b1 = float2x3(
                                {INF, -INF},
                                {INF, -INF},
                                {INF, -INF});
                            float2 r0, r1;
                            r0 = r1 = float2(INF, -INF);
                            int count0 = 0, count1 = 0;
                            for (int j = 0; j <= i; ++j)
                            {
                                b0 = extendedBounds(b0, buckets[j].bounds);
                                r0 = extendedBounds(r0, buckets[j].primitiveRange);
                                count0 += buckets[j].count;
                            }
                            for (int j = i + 1; j < nBuckets; ++j)
                            {
                                b1 = extendedBounds(b1, buckets[j].bounds);
                                r1 = extendedBounds(r1, buckets[j].primitiveRange);
                                count1 += buckets[j].count;
                            }
                            cost[i] = 1 +
                                      (count0 * SurfaceArea(b0) +
                                       count1 * SurfaceArea(b1)) /
                                          SurfaceArea(bounds);
                            // cost[i] = 1 +
                            //           (count0 * SurfaceArea(b0) * ((r0.upper-globalRange.lower) / (globalRange.upper-globalRange.lower)) +       //Eq. 40 from https://graphics.pixar.com/library/ProductionVolumeRendering/paper.pdf
                            //            count1 * SurfaceArea(b1) * ((r1.upper-globalRange.lower) / (globalRange.upper-globalRange.lower))) /
                            //               SurfaceArea(bounds);
                        }

                        // Find bucket to split at that minimizes SAH metric
                        float minCost = cost[0];
                        int minCostSplitBucket = 0;
                        for (int i = 1; i < nBuckets - 1; ++i)
                        {
                            if (cost[i] < minCost)
                            {
                                minCost = cost[i];
                                minCostSplitBucket = i;
                            }
                        }

                        // Either create leaf or split primitives at selected SAH
                        // bucket
                        float leafCost = nPrimitives;
                        if (nPrimitives > maxPrimsInNode && minCost < leafCost)
                        {
                            BVHPrimitiveInfo *pmid = std::partition(
                                &primitiveInfo[start], &primitiveInfo[end - 1] + 1,
                                [=](const BVHPrimitiveInfo &pi)
                                {
                                    int b = nBuckets *
                                            Offset(centroidBounds, pi.centroid)[dim];
                                    if (b == nBuckets)
                                        b = nBuckets - 1;
                                    /*CHECK_GE(b, 0);
                                    CHECK_LT(b, nBuckets);*/
                                    return b <= minCostSplitBucket;
                                });
                            mid = pmid - &primitiveInfo[0];
                        }
                        else
                        {
                            // Create leaf _BVHBuildNode_
                            int firstPrimOffset = orderedPrimRanges.size();
                            for (int i = start; i < end; ++i)
                            {
                                int primNum = primitiveInfo[i].primitiveNumber;
                                orderedPrimRanges.push_back(primitiveRanges[primNum]);
                            }
                            node->InitLeaf(firstPrimOffset, nPrimitives, bounds);
                            return node;
                        }
                    }
                    break;
                }
                }
                node->InitInterior(dim,
                                   recursiveBuild(arena, primitiveInfo, start, mid,
                                                  totalNodes, orderedPrimRanges),
                                   recursiveBuild(arena, primitiveInfo, mid, end,
                                                  totalNodes, orderedPrimRanges));
            }
        }
        return node;
    }

    BVHBuildNode *BVHAccel::HLBVHBuild(
        MemoryArena &arena, const std::vector<BVHPrimitiveInfo> &primitiveInfo,
        int *totalNodes,
        std::vector<float2> &orderedPrimRanges) const
    {
        // Compute bounding box of all primitive centroids
        float2x3 bounds = float2x3(
            {INF, -INF},
            {INF, -INF},
            {INF, -INF});
        for (const BVHPrimitiveInfo &pi : primitiveInfo)
            bounds = extendedBounds(bounds, pi.centroid);

        // Compute Morton indices of primitives
        std::vector<MortonPrimitive> mortonPrims(primitiveInfo.size());

        // parallel_for(primitiveInfo.size(), [&](int i) {
        //     // Initialize _mortonPrims[i]_ for _i_th primitive
        //     constexpr int mortonBits = 10;
        //     constexpr int mortonScale = 1 << mortonBits;
        //     mortonPrims[i].primitiveIndex = primitiveInfo[i].primitiveNumber;
        //     float3 centroidOffset = Offset(bounds, primitiveInfo[i].centroid);
        //     mortonPrims[i].mortonCode = EncodeMorton3(centroidOffset * mortonScale);
        // }, 512);

        for (size_t i = 0; i < primitiveInfo.size(); i++) // This was a parallel for loop(above)
        {
            // Initialize _mortonPrims[i]_ for _i_th primitive
            constexpr int mortonBits = 10;
            constexpr int mortonScale = 1 << mortonBits;
            mortonPrims[i].primitiveIndex = primitiveInfo[i].primitiveNumber;
            float3 centroidOffset = Offset(bounds, primitiveInfo[i].centroid);
            mortonPrims[i].mortonCode = EncodeMorton3(centroidOffset * mortonScale);
        }

        // Radix sort primitive Morton indices
        RadixSort(&mortonPrims);

        // Create LBVH treelets at bottom of BVH

        // Find intervals of primitives for each treelet
        std::vector<LBVHTreelet> treeletsToBuild;
        for (int start = 0, end = 1; end <= (int)mortonPrims.size(); ++end)
        {
#ifdef PBRT_HAVE_BINARY_CONSTANTS
            uint32_t mask = 0b00111111111111000000000000000000;
#else
            uint32_t mask = 0x3ffc0000;
#endif
            if (end == (int)mortonPrims.size() ||
                ((mortonPrims[start].mortonCode & mask) !=
                 (mortonPrims[end].mortonCode & mask)))
            {
                // Add entry to _treeletsToBuild_ for this treelet
                int nPrimitives = end - start;
                int maxBVHNodes = 2 * nPrimitives;
                BVHBuildNode *nodes = arena.Alloc<BVHBuildNode>(maxBVHNodes, false);
                treeletsToBuild.push_back({start, nPrimitives, nodes});
                start = end;
            }
        }

        // Create LBVHs for treelets in parallel
        std::atomic<int> atomicTotal(0), orderedPrimsOffset(0);
        orderedPrimRanges.resize(primitiveRanges.size());
        // parallel_for(treeletsToBuild.size(), [&](int i) { //??
        //     // Generate _i_th LBVH treelet
        //     int nodesCreated = 0;
        //     const int firstBitIndex = 29 - 12;
        //     LBVHTreelet &tr = treeletsToBuild[i];
        //     tr.buildNodes =
        //         emitLBVH(tr.buildNodes, primitiveInfo, &mortonPrims[tr.startIndex],
        //                  tr.nPrimitives, &nodesCreated, orderedPrimRanges,
        //                  &orderedPrimsOffset, firstBitIndex);
        //     atomicTotal += nodesCreated;
        // });
        for (size_t i = 0; i < treeletsToBuild.size(); i++) // This was a parallel for loop(above)
        {
            // Generate _i_th LBVH treelet
            int nodesCreated = 0;
            const int firstBitIndex = 29 - 12;
            LBVHTreelet &tr = treeletsToBuild[i];
            tr.buildNodes =
                emitLBVH(tr.buildNodes, primitiveInfo, &mortonPrims[tr.startIndex],
                         tr.nPrimitives, &nodesCreated, orderedPrimRanges,
                         &orderedPrimsOffset, firstBitIndex);
            atomicTotal += nodesCreated;
        }

        *totalNodes = atomicTotal;

        // Create and return SAH BVH from LBVH treelets
        std::vector<BVHBuildNode *> finishedTreelets;
        finishedTreelets.reserve(treeletsToBuild.size());
        for (LBVHTreelet &treelet : treeletsToBuild)
            finishedTreelets.push_back(treelet.buildNodes);
        return buildUpperSAH(arena, finishedTreelets, 0, finishedTreelets.size(),
                             totalNodes);
    }

    BVHBuildNode *BVHAccel::emitLBVH(
        BVHBuildNode *&buildNodes,
        const std::vector<BVHPrimitiveInfo> &primitiveInfo,
        MortonPrimitive *mortonPrims, int nPrimitives, int *totalNodes,
        std::vector<float2> &orderedPrimRanges,
        std::atomic<int> *orderedPrimsOffset, int bitIndex) const
    {
        // CHECK_GT(nPrimitives, 0);
        if (bitIndex == -1 || nPrimitives < maxPrimsInNode)
        {
            // Create and return leaf node of LBVH treelet
            (*totalNodes)++;
            BVHBuildNode *node = buildNodes++;
            float2x3 bounds;
            int firstPrimOffset = orderedPrimsOffset->fetch_add(nPrimitives);
            for (int i = 0; i < nPrimitives; ++i)
            {
                int primitiveIndex = mortonPrims[i].primitiveIndex;
                orderedPrimRanges[firstPrimOffset + i] = primitiveRanges[primitiveIndex];
                bounds = extendedBounds(bounds, primitiveInfo[primitiveIndex].centroid);
            }
            node->InitLeaf(firstPrimOffset, nPrimitives, bounds);
            return node;
        }
        else
        {
            int mask = 1 << bitIndex;
            // Advance to next subtree level if there's no LBVH split for this bit
            if ((mortonPrims[0].mortonCode & mask) ==
                (mortonPrims[nPrimitives - 1].mortonCode & mask))
                return emitLBVH(buildNodes, primitiveInfo, mortonPrims, nPrimitives,
                                totalNodes, orderedPrimRanges, orderedPrimsOffset,
                                bitIndex - 1);

            // Find LBVH split point for this dimension
            int searchStart = 0, searchEnd = nPrimitives - 1;
            while (searchStart + 1 != searchEnd)
            {
                // CHECK_NE(searchStart, searchEnd);
                int mid = (searchStart + searchEnd) / 2;
                if ((mortonPrims[searchStart].mortonCode & mask) ==
                    (mortonPrims[mid].mortonCode & mask))
                    searchStart = mid;
                else
                {
                    /*CHECK_EQ(mortonPrims[mid].mortonCode & mask,
                             mortonPrims[searchEnd].mortonCode & mask);*/
                    searchEnd = mid;
                }
            }
            int splitOffset = searchEnd;
            // CHECK_LE(splitOffset, nPrimitives - 1);
            /*CHECK_NE(mortonPrims[splitOffset - 1].mortonCode & mask,
                     mortonPrims[splitOffset].mortonCode & mask);*/

            // Create and return interior LBVH node
            (*totalNodes)++;
            BVHBuildNode *node = buildNodes++;
            BVHBuildNode *lbvh[2] = {
                emitLBVH(buildNodes, primitiveInfo, mortonPrims, splitOffset,
                         totalNodes, orderedPrimRanges, orderedPrimsOffset,
                         bitIndex - 1),
                emitLBVH(buildNodes, primitiveInfo, &mortonPrims[splitOffset],
                         nPrimitives - splitOffset, totalNodes, orderedPrimRanges,
                         orderedPrimsOffset, bitIndex - 1)};
            int axis = bitIndex % 3;
            node->InitInterior(axis, lbvh[0], lbvh[1]);
            return node;
        }
    }

    BVHBuildNode *BVHAccel::buildUpperSAH(MemoryArena &arena,
                                          std::vector<BVHBuildNode *> &treeletRoots,
                                          int start, int end,
                                          int *totalNodes) const
    {
        // CHECK_LT(start, end);
        int nNodes = end - start;
        if (nNodes == 1)
            return treeletRoots[start];
        (*totalNodes)++;
        BVHBuildNode *node = arena.Alloc<BVHBuildNode>();

        // Compute bounds of all nodes under this HLBVH node
        float2x3 bounds = float2x3(
            {INF, -INF},
            {INF, -INF},
            {INF, -INF});

        for (int i = start; i < end; ++i)
            bounds = extendedBounds(bounds, treeletRoots[i]->bounds);

        // Compute bound of HLBVH node centroids, choose split dimension _dim_
        float2x3 centroidBounds = float2x3(
            {INF, -INF},
            {INF, -INF},
            {INF, -INF});
        for (int i = start; i < end; ++i)
        {
            float3 centroid =
                (treeletRoots[i]->bounds.row(0) + treeletRoots[i]->bounds.row(1)) *
                0.5f;
            centroidBounds = extendedBounds(centroidBounds, centroid);
        }
        int dim = MaximumExtent(centroidBounds);
        // FIXME: if this hits, what do we need to do?
        // Make sure the SAH split below does something... ?
        // CHECK_NE(centroidBounds.upper[dim], centroidBounds.pMin[dim]);

        // Allocate _BucketInfo_ for SAH partition buckets
        constexpr int nBuckets = 12;
        struct BucketInfo
        {
            int count = 0;
            float2x3 bounds = float2x3(
                {INF, -INF},
                {INF, -INF},
                {INF, -INF});
            float2 primitiveRange;
        };
        BucketInfo buckets[nBuckets];

        // Initialize _BucketInfo_ for HLBVH SAH partition buckets
        for (int i = start; i < end; ++i)
        {
            float centroid = (treeletRoots[i]->bounds.row(0)[dim] +
                              treeletRoots[i]->bounds.row(1)[dim]) *
                             0.5f;
            int b =
                nBuckets * ((centroid - centroidBounds.row(0)[dim]) /
                            (centroidBounds.row(1)[dim] - centroidBounds.row(0)[dim]));
            if (b == nBuckets)
                b = nBuckets - 1;
            // CHECK_GE(b, 0);
            // CHECK_LT(b, nBuckets);
            buckets[b].count++;
            buckets[b].bounds = extendedBounds(buckets[b].bounds, treeletRoots[i]->bounds);

            for (size_t j = treeletRoots[i]->firstPrimOffset;
                 j < treeletRoots[i]->firstPrimOffset + treeletRoots[i]->nPrimitives; j++)
            {
                buckets[b].primitiveRange = extendedBounds(buckets[b].primitiveRange, primitiveRanges[j]);
            }
        }

        // Compute costs for splitting after each bucket
        float cost[nBuckets - 1];
        for (int i = 0; i < nBuckets - 1; ++i)
        {
            float2x3 b0, b1;
            b0 = b1 = float2x3(
                {INF, -INF},
                {INF, -INF},
                {INF, -INF});
            float2 r0, r1;
            r0 = r1 = float2(INF, -INF);
            int count0 = 0, count1 = 0;
            for (int j = 0; j <= i; ++j)
            {
                b0 = extendedBounds(b0, buckets[j].bounds);
                r0 = extendedBounds(r0, buckets[j].primitiveRange);
                count0 += buckets[j].count;
            }
            for (int j = i + 1; j < nBuckets; ++j)
            {
                b1 = extendedBounds(b1, buckets[j].bounds);
                r1 = extendedBounds(r1, buckets[j].primitiveRange);
                count1 += buckets[j].count;
            }
            cost[i] = .125f +
                      (count0 * SurfaceArea(b0) + count1 * SurfaceArea(b1)) /
                          SurfaceArea(bounds);
            // cost[i] = .125f +
            //           (count0 * SurfaceArea(b0) * ((r0.upper-globalRange.lower) / (globalRange.upper-globalRange.lower)) +
            //           count1 * SurfaceArea(b1) * ((r1.upper-globalRange.lower) / (globalRange.upper-globalRange.lower))) /
            //               SurfaceArea(bounds);
        }

        // Find bucket to split at that minimizes SAH metric
        float minCost = cost[0];
        int minCostSplitBucket = 0;
        for (int i = 1; i < nBuckets - 1; ++i)
        {
            if (cost[i] < minCost)
            {
                minCost = cost[i];
                minCostSplitBucket = i;
            }
        }

        // Split nodes and create interior HLBVH SAH node
        BVHBuildNode **pmid = std::partition(
            &treeletRoots[start], &treeletRoots[end - 1] + 1,
            [=](const BVHBuildNode *node)
            {
                float centroid =
                    (node->bounds.row(0)[dim] + node->bounds.row(1)[dim]) * 0.5f;
                int b = nBuckets *
                        ((centroid - centroidBounds.row(0)[dim]) /
                         (centroidBounds.row(1)[dim] - centroidBounds.row(0)[dim]));
                if (b == nBuckets)
                    b = nBuckets - 1;
                // CHECK_GE(b, 0);
                // CHECK_LT(b, nBuckets);
                return b <= minCostSplitBucket;
            });
        int mid = pmid - &treeletRoots[0];
        // CHECK_GT(mid, start);
        // CHECK_LT(mid, end);
        node->InitInterior(
            dim, this->buildUpperSAH(arena, treeletRoots, start, mid, totalNodes),
            this->buildUpperSAH(arena, treeletRoots, mid, end, totalNodes));
        return node;
    }

    int BVHAccel::flattenBVHTree(BVHBuildNode *node, int *offset)
    {
        LinearBVHNode *linearNode = &nodes[*offset];
        linearNode->bounds = float2x4(
            {INF, -INF},
            {INF, -INF},
            {INF, -INF},
            {INF, -INF});

        int myOffset = (*offset)++;
        if (node->nPrimitives > 0)
        {
            // CHECK(!node->children[0] && !node->children[1]); //??
            // CHECK_LT(node->nPrimitives, 65536);
            linearNode->primitivesOffset = node->firstPrimOffset;
            linearNode->nPrimitives = node->nPrimitives;
        }
        else
        {
            // Create interior flattened BVH node
            linearNode->axis = node->splitAxis;
            linearNode->nPrimitives = 0;
            flattenBVHTree(node->children[0], offset);
            linearNode->secondChildOffset =
                flattenBVHTree(node->children[1], offset);
        }
        return myOffset;
    }

    BVHAccel::BVHAccel(int maxPrimsInNode,
                       SplitMethod splitMethod)
        : maxPrimsInNode(maxPrimsInNode),
          splitMethod(splitMethod)
    {
    }

    BVHAccel::~BVHAccel() { FreeAligned(nodes); }

    /*bool BVHAccel::Intersect(const Ray &ray, SurfaceInteraction *isect) const {
        if (!nodes) return false;
        ProfilePhase p(Prof::AccelIntersect);
        bool hit = false;
        Vector3f invDir(1 / ray.d.x, 1 / ray.d.y, 1 / ray.d.z);
        int dirIsNeg[3] = {invDir.x < 0, invDir.y < 0, invDir.z < 0};
        // Follow ray through BVH nodes to find primitive intersections
        int toVisitOffset = 0, currentNodeIndex = 0;
        int nodesToVisit[64];
        while (true) {
            const LinearBVHNode *node = &nodes[currentNodeIndex];
            // Check ray against BVH node
            if (node->bounds.IntersectP(ray, invDir, dirIsNeg)) {
                if (node->nPrimitives > 0) {
                    // Intersect ray with primitives in leaf BVH node
                    for (int i = 0; i < node->nPrimitives; ++i)
                        if (primitives[node->primitivesOffset + i]->Intersect(
                                ray, isect))
                            hit = true;
                    if (toVisitOffset == 0) break;
                    currentNodeIndex = nodesToVisit[--toVisitOffset];
                } else {
                    // Put far BVH node on _nodesToVisit_ stack, advance to near
                    // node
                    if (dirIsNeg[node->axis]) {
                        nodesToVisit[toVisitOffset++] = currentNodeIndex + 1;
                        currentNodeIndex = node->secondChildOffset;
                    } else {
                        nodesToVisit[toVisitOffset++] = node->secondChildOffset;
                        currentNodeIndex = currentNodeIndex + 1;
                    }
                }
            } else {
                if (toVisitOffset == 0) break;
                currentNodeIndex = nodesToVisit[--toVisitOffset];
            }
        }
        return hit;
    }*/

    // bool BVHAccel::Intersect(const owl::vec3f& org, const owl::vec3f& dir) const {
    //     if (nAllocatedNodes ==0) return false;

    //     //Find the first intersection
    //     bool hit = false;
    //     owl::vec3f invDir(1 / dir.x, 1 / dir.y, 1 / dir.z);
    //     int dirIsNeg[3] = {invDir.x < 0, invDir.y < 0, invDir.z < 0};
    //     // Follow ray through BVH nodes to find primitive intersections
    //     int toVisitOffset = 0, currentNodeIndex = 0;
    //     int nodesToVisit[64];
    //     while (true) {
    //         const LinearBVHNode *node = &nodes[currentNodeIndex];
    //         // Check ray against BVH node
    //         if (IntersectBox(node->bounds, org, dir, invDir, dirIsNeg)) {
    //             //push the visited nodes
    //             if (node->nPrimitives > 0) {
    //                 // Intersect ray with primitives in leaf BVH node
    //                 for (int i = 0; i < node->nPrimitives; ++i)
    //                 {
    //                     /*if (primitives[node->primitivesOffset + i]->Intersect(
    //                             ray, isect))*/
    //                     hit = true;
    //                     printf("tested primitive %d on node : %d\n", node->primitivesOffset + i, currentNodeIndex);
    //                 }
    //                 if (toVisitOffset == 0) break;
    //                 currentNodeIndex = nodesToVisit[--toVisitOffset];
    //             } else {
    //                 // Put far BVH node on _nodesToVisit_ stack, advance to near
    //                 // node
    //                 if (dirIsNeg[node->axis]) {
    //                     nodesToVisit[toVisitOffset++] = currentNodeIndex + 1;
    //                     currentNodeIndex = node->secondChildOffset;
    //                 } else {
    //                     nodesToVisit[toVisitOffset++] = node->secondChildOffset;
    //                     currentNodeIndex = currentNodeIndex + 1;
    //                 }
    //             }
    //         } else {
    //             if (toVisitOffset == 0) break;
    //             currentNodeIndex = nodesToVisit[--toVisitOffset];
    //         }
    //     }
    //     return hit;
    // }

    void BVHAccel::saveTo(const std::string &fileName) const
    {
        // std::ofstream out(fileName, std::ios_base::binary);
        // writeTo(out);
        // std::cout << "BVH accel saved to " << fileName << std::endl;
    }

    /*! write - binary - to given (bianry) stream */
    void BVHAccel::writeTo(std::ostream &out) const
    {
        // umesh::io::writeElement(out, bum_magic);

        // umesh::io::writeElement(out,maxPrimsInNode);
        // umesh::io::writeElement(out, splitMethod);

        // umesh::io::writeVector(out, primitiveRanges);

        // //umesh::io::writeElement(out, nAllocedNodes);
        // umesh::io::writeElement(out, nAllocatedNodes);
        // umesh::io::writeArray(out, nodes, nAllocatedNodes);
        // //printf("Alloc nodes %d\n");
    }

    std::shared_ptr<BVHAccel> BVHAccel::loadFrom(const std::string &fileName)
    {
        // auto bvhAcell = std::make_shared<BVHAccel>();
        // std::ifstream in(fileName, std::ios_base::binary);
        // bvhAcell->readFrom(in);
        // std::cout << "BVH accel loaded from " << fileName << std::endl;
        // return bvhAcell;
    }

    void BVHAccel::readFrom(std::istream &in)
    {
        // size_t magic;
        // umesh::io::readElement(in, magic);
        // if(magic == bum_magic)
        // {
        //     umesh::io::readElement(in, maxPrimsInNode);
        //     umesh::io::readElement(in, splitMethod);

        //     umesh::io::readVector(in, primitiveRanges);

        //     umesh::io::readElement(in, nAllocatedNodes);
        //     nodes = AllocAligned<LinearBVHNode>(nAllocatedNodes);
        //     umesh::io::readArray(in, nodes, nAllocatedNodes);
        //     //printf("Alloc nodes %d\n");
        //     return;
        // }
        // throw std::runtime_error("wrong magic number in bvh file ...");
    }

    std::shared_ptr<BVHAccel> CreateBVHAccelerator(
        std::vector<float4> particles, int maxPrimsInNode = 16)
    {
        // std::string splitMethodName = ps.FindOneString("splitmethod", "sah");
        BVHAccel::SplitMethod splitMethod;
        // if (splitMethodName == "sah")
        splitMethod = BVHAccel::SplitMethod::MMH;
        /*else if (splitMethodName == "hlbvh")
            splitMethod = BVHAccel::SplitMethod::HLBVH;
        else if (splitMethodName == "middle")
            splitMethod = BVHAccel::SplitMethod::Middle;
        else if (splitMethodName == "equal")
            splitMethod = BVHAccel::SplitMethod::EqualCounts;
        else {
            Warning("BVH split method \"%s\" unknown.  Using \"sah\".",
                    splitMethodName.c_str());
            splitMethod = BVHAccel::SplitMethod::SAH;
        }*/

        //int maxPrimsInNode = 16; // 65535/4;//ps.FindOneInt("maxnodeprims", 4);

        // std::cout<<"TEMPORARY FOR DEVELOPMENT"<<std::endl;
        // int maxPrimsInNode = 1;//ps.FindOneInt("maxnodeprims", 4);
        return std::make_shared<BVHAccel>(particles, maxPrimsInNode, splitMethod);
    }

} // namespace Accelerator

#include "importers/import_points.h"
#include "importers/import_mmpld.h"
#include <argparse/argparse.hpp>
#include <numeric>
#include <chrono>
// execution
int main(int argc, char *argv[])
{
    std::vector<std::vector<float4>> particles;
    size_t maxNumParticles;
    argparse::ArgumentParser program("kd tree builder");

    program.add_argument("--dbscan")
        .help("A path to a DBScan dataset (ending in .arborx)")
        .default_value("");
    program.add_argument("--mmpld")
        .help("A path to a MMPLD dataset (ending in .mmpld)")
        .default_value("");
    program.add_argument("--points")
        .help("A path to our custom points dataset (ending in .points)")
        .default_value("");

    program.add_argument("--selectTimeStep")
        .help("Select a specific time step")
        .default_value(0)
        .scan<'i', int>();
    program.add_argument("--maxPrims")
        .help("Maximum number of primitives per leaf")
        .default_value(100)
        .scan<'i', int>();

    try
    {
        program.parse_args(argc, argv);
    }
    catch (const std::runtime_error &err)
    {
        std::cerr << err.what() << std::endl;
        std::cerr << program;
        std::exit(1);
    }

    std::vector<std::vector<std::pair<uint64_t, float4>>> particleData;

    std::string dbscanPath = program.get<std::string>("--dbscan");
    std::string mmpldPath = program.get<std::string>("--mmpld");
    std::string pointsPath = program.get<std::string>("--points");
    if (dbscanPath != "")
        importArborX(dbscanPath, particleData);
    else if (mmpldPath != "")
        importMMPLD(mmpldPath, particleData);
    else if (pointsPath != "")
        importPoints(pointsPath, particleData);
    else
    {
        int ddaGridResolution = 1;
        particleData.resize(100);
        for (uint32_t frame = 0; frame < particleData.size(); ++frame)
        {
            particleData[frame].resize(3);
            for (uint32_t i = 0; i < particleData[frame].size(); ++i)
            {
                float t1 = float(i) / float(particleData[frame].size());

                float t2 = float(frame) / float(particleData.size());

                particleData[frame][i].second = float4(
                    (cos(t2 * 3.14) * .5 + .5) * sin(t1 * 2.f * 3.14f),
                    (cos(t2 * 3.14) * .5 + .5) * cos(t1 * 2.f * 3.14f),
                    0.f, t1);
            }
        }
    }

    float minScalarValue = +1e20f;
    float maxScalarValue = -1e20f;

    particles.resize(particleData.size());
    for (size_t j = 0; j < particleData.size(); ++j)
    {
        particles[j].resize(particleData[j].size());
        for (size_t i = 0; i < particles[j].size(); ++i)
        {
            particles[j][i] = particleData[j][i].second;
            minScalarValue = std::min(particles[j][i].w, minScalarValue);
            maxScalarValue = std::max(particles[j][i].w, maxScalarValue);
        }
        particleData[j].clear();
        maxNumParticles = std::max(maxNumParticles, particles[j].size());
    }

    // normalize attributes
    if (maxScalarValue > minScalarValue)
    {
        for (size_t j = 0; j < particles.size(); ++j)
        {
            for (size_t i = 0; i < particles[j].size(); ++i)
            {
                particles[j][i].w = (particles[j][i].w - minScalarValue) / (maxScalarValue - minScalarValue);
            }
        }
    }

    //time start
    auto start = std::chrono::high_resolution_clock::now();
    std::vector<float4> bvhParticles = particles[program.get<int>("--selectTimeStep")];
    auto bvh = Accelerator::CreateBVHAccelerator(bvhParticles, program.get<int>("--maxPrims"));
    //time end
    auto end = std::chrono::high_resolution_clock::now();

    printf("Created BVH with %d nodes in %fs\n", bvh->nAllocatedNodes, std::chrono::duration<double>(end - start).count());
}

