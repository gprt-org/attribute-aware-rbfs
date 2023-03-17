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


// accelerators/kdtreeaccel.cpp*
#include "kdTreeBuilder.h"

#include <algorithm>
#include "memory.h"


namespace Accelerator
{
enum class EdgeType { Start, End };
struct BoundEdge {
    // BoundEdge Public Methods
    BoundEdge() {}
    BoundEdge(float t, uint32_t primNum, bool starting) : t(t), primNum(primNum) {
        type = starting ? EdgeType::Start : EdgeType::End;
    }
    float t;
    uint32_t primNum;
    EdgeType type;
    float2 majorants; //majorants: above -> upper, below -> lower
};

void KdTreeAccel::saveTo(const std::string &fileName) const
{
    std::ofstream out(fileName, std::ios_base::binary);
    writeTo(out);
    std::cout << "KD tree accel saved to " << fileName << std::endl;
}

/*! write - binary - to given (bianry) stream */
void KdTreeAccel::writeTo(std::ostream &out) const
{
    // umesh::io::writeElement(out, bum_magic);

    // umesh::io::writeElement(out,isectCost);
    // umesh::io::writeElement(out, traversalCost);
    // umesh::io::writeElement(out, maxPrims);
    // umesh::io::writeElement(out,emptyBonus);

    // umesh::io::writeVector(out, primitiveIndices);
    // umesh::io::writeVector(out, primitiveRanges);

    // umesh::io::writeElement(out, nAllocatedNodes);
    // umesh::io::writeArray(out, nodes, nAllocatedNodes);

    // umesh::io::writeElement(out, nextFreeNode);
    // umesh::io::writeElement(out, bounds);

    //todo
}

std::shared_ptr<KdTreeAccel> KdTreeAccel::loadFrom(const std::string &fileName)
{
    auto kdAcell = std::make_shared<KdTreeAccel>();
    std::ifstream in(fileName, std::ios_base::binary);
    kdAcell->readFrom(in);
    std::cout << "KD tree accel loaded from " << fileName << std::endl;
    return kdAcell;
}

void KdTreeAccel::readFrom(std::istream &in)
{
    size_t magic;
    umesh::io::readElement(in, magic);
    if(magic == bum_magic)
    {
        // umesh::io::readElement(in,isectCost);
        // umesh::io::readElement(in, traversalCost);
        // umesh::io::readElement(in, maxPrims);
        // umesh::io::readElement(in,emptyBonus);

        // umesh::io::readVector(in, primitiveIndices);
        // umesh::io::readVector(in, primitiveRanges);

        // umesh::io::readElement(in, nAllocatedNodes);
        // nodes = AllocAligned<KdAccelNode>(nAllocatedNodes);
        // umesh::io::readArray(in, nodes, nAllocatedNodes);

        // umesh::io::readElement(in, nextFreeNode);
        // umesh::io::readElement(in, bounds);
        // return;
    }
    throw std::runtime_error("wrong magic number in kd file ...");

}

bool KdTreeAccel::same(std::shared_ptr<KdTreeAccel> kd1, std::shared_ptr<KdTreeAccel> kd2)
{
    if(kd1->emptyBonus != kd2->emptyBonus)
    {
        printf("failed at emptyBounds\n");
        return false;
    }
    if(kd1->isectCost != kd2->isectCost)
    {
        printf("failed at isectCost\n");
        return false;
    }
    if(kd1->maxPrims != kd2->maxPrims)
    {
        printf("failed at maxPrims\n");
        return false;
    }
    if(kd1->nAllocatedNodes != kd2->nAllocatedNodes)
    {
        printf("failed at nAllocatedNodes\n");
        return false;
    }
    if(kd1->nextFreeNode != kd2->nextFreeNode)
    {
        printf("failed at nextFreeNode\n");
        return false;
    }
    if(kd1->traversalCost != kd2->traversalCost)
    {
        printf("failed at traversalCost\n");
        return false;
    }
    for (size_t i = 0; i < kd1->primitiveIndices.size(); i++)
    {
        if(kd1->primitiveIndices[i] != kd2->primitiveIndices[i])
        {
            printf("failed at primitiveIndices\n");
            return false;
        }
    }
    for (int i = 0; i < kd1->nAllocatedNodes; i++)
    {
        if( (kd1->nodes[i].onePrimitive != kd2->nodes[i].onePrimitive) /*||
            (kd1->nodes[i].flags != kd2->nodes[i].flags) */)
        {
            printf("failed at node[%d] | %d %d\n ",i, kd1->nodes[i].split, kd2->nodes[i].split);
            return false;
        }
    }
    if(kd1->bounds.lower.x != kd2->bounds.lower.x)
        return false;

    return true;
}

KdTreeAccel::KdTreeAccel(int isectCost, int traversalCost, float emptyBonus,
                         int maxPrims, int maxDepth)
    : isectCost(isectCost),
      traversalCost(traversalCost),
      maxPrims(maxPrims),
      emptyBonus(emptyBonus)
{
}

// KdTreeAccel Method Definitions
KdTreeAccel::KdTreeAccel(std::vector<std::vector<float4>> particles,
                         int isectCost, int traversalCost, float emptyBonus,
                         int maxPrims, int maxDepth)
    : isectCost(isectCost),
      traversalCost(traversalCost),
      maxPrims(maxPrims),
      emptyBonus(emptyBonus),
      umesh_ptr(umesh_ptr) {
    // Build kd-tree for accelerator
    //ProfilePhase _(Prof::AccelConstruction);
    nextFreeNode = nAllocatedNodes = 0;
    uint64_t primCount = umesh_ptr->numVolumeElements();//calculateVolumeFaceCount(umesh_ptr);
    if (maxDepth <= 0)
        maxDepth = std::round(8 + 1.3f * Log2Int(int64_t(primCount)));
    printf("CALLED: KdTreeAccel, maxDept= %d\n", maxDepth);
    // Compute bounds for kd-tree construction
    std::vector<float2x3> primBounds;
    primBounds.reserve(primCount);

    auto primRefs = umesh_ptr->createVolumePrimRefs();

    for (const auto &primRef : primRefs) {
        float2x3 b;
        float2 valueRange; 
        switch (primRef.type)
        {
        case umesh::UMesh::PrimType::TET:
            b = umesh_ptr->getTetBounds(primRef.ID);
            valueRange = umesh_ptr->getTetValueRange(primRef.ID);
            break;
        case umesh::UMesh::PrimType::PYR:
            b = umesh_ptr->getPyrBounds(primRef.ID);
            valueRange = umesh_ptr->getPyrValueRange(primRef.ID);
            break;
        case umesh::UMesh::PrimType::WEDGE:
            b = umesh_ptr->getWedgeBounds(primRef.ID);
            valueRange = umesh_ptr->getWedgeValueRange(primRef.ID);
            break;
        case umesh::UMesh::PrimType::HEX:
            b = umesh_ptr->getHexBounds(primRef.ID);
            valueRange = umesh_ptr->getHexValueRange(primRef.ID);
            break;
        default:
            break;
        }
        bounds.extend(b); 
        primBounds.push_back(b);
        primitiveRanges.push_back(valueRange);
        globalRange.extend(valueRange);
    }
    std::cout<<"Done generating bounding boxes. Building tree..."<<std::endl;

    // Allocate working memory for kd-tree construction
    std::unique_ptr<BoundEdge[]> edges[3];
    for (int i = 0; i < 3; ++i)
        edges[i].reset(new BoundEdge[2ull * primCount]);
    std::unique_ptr<uint32_t[]> prims0(new uint32_t[primCount]);
    std::unique_ptr<uint32_t[]> prims1(new uint32_t[(maxDepth + 1) * primCount]);

    // Initialize _primNums_ for kd-tree construction
    std::unique_ptr<uint32_t[]> primNums(new uint32_t[primCount]);
    for (size_t i = 0; i < primCount; ++i) primNums[i] = i;

    // Start recursive construction of kd-tree
    buildTree(0, bounds, primBounds, primNums.get(), primCount,
              maxDepth, edges, prims0.get(), prims1.get());
    std::cout<<"Done building the kd tree" << std::endl;

    for (size_t i = 0; i < nAllocatedNodes; i++)
    {      
        nodes[i].range.lower = globalRange.lower - 1.0f;
        nodes[i].range.upper = globalRange.upper + 1.0f;
    }
    CalculateMajorants();
    for (size_t i = 0; i < nAllocatedNodes; i++)
    {
        printf("Range: %f %f \n", nodes[i].range.lower, nodes[i].range.upper);
    }
}

void KdAccelNode::InitLeaf(uint32_t *primNums, uint32_t np,
                           std::vector<uint32_t> *primitiveIndices) {
    flags = 3;
    nPrims |= (np << 2);
    // Store primitive ids for leaf node
    if (np == 0)
        onePrimitive = -1; //IMPORTANT:changed to -1 to avoid collisions!
    else if (np == 1)
        onePrimitive = primNums[0];
    else {
        primitiveIndicesOffset = primitiveIndices->size();
        for (uint32_t i = 0; i < np; ++i) primitiveIndices->push_back(primNums[i]);
    }
}

KdTreeAccel::~KdTreeAccel() { FreeAligned(nodes); }

void KdTreeAccel::buildTree(uint32_t nodeNum, const float2x3 &nodeBounds,
                            const std::vector<float2x3> &allPrimBounds,
                            uint32_t *primNums, uint32_t nPrimitives, uint32_t depth,
                            const std::unique_ptr<BoundEdge[]> edges[3],
                            uint32_t *prims0, uint32_t *prims1, uint32_t badRefines) {
    assert(nodeNum == nextFreeNode);
    //printf("CALLED: buildTree\n");
    // Get next free node from _nodes_ array
    if (nextFreeNode == nAllocatedNodes) {
        int nNewAllocNodes = std::max(2 * nAllocatedNodes, 1);
        KdAccelNode *n = AllocAligned<KdAccelNode>(nNewAllocNodes);
        if (nAllocatedNodes > 0) {
            memcpy(n, nodes, nAllocatedNodes * sizeof(KdAccelNode));
            FreeAligned(nodes);
        }
        nodes = n;
        nAllocatedNodes = nNewAllocNodes;
    }
    ++nextFreeNode;

    // Initialize leaf node if termination criteria met
    if (nPrimitives <= maxPrims || depth == 0) {
        std::cout<<"Emitting leaf at node num " << nodeNum << std::endl;
        nodes[nodeNum].InitLeaf(primNums, nPrimitives, &primitiveIndices);
        return;
    }

    // Initialize interior node and continue recursion

    // Choose split axis position for interior node
    int bestAxis = -1, bestOffset = -1;
    float bestCost = INF;
    float oldCost = isectCost * float(nPrimitives);
    float totalSA = SurfaceArea(nodeBounds);
    float invTotalSA = 1 / totalSA;
    umesh::vec3f d = nodeBounds.upper - nodeBounds.lower;

    // Choose which axis to split along
    int axis = MaximumExtent(nodeBounds);
    int retries = 0;
retrySplit:

    // Initialize edges for _axis_
    for (int i = 0; i < nPrimitives; ++i) {
        int pn = primNums[i];
        const float2x3 &bounds = allPrimBounds[pn];
        edges[axis][2 * i] = BoundEdge(bounds.lower[axis], pn, true);
        edges[axis][2 * i + 1] = BoundEdge(bounds.upper[axis], pn, false);
    }

    // Sort _edges_ for _axis_
    std::sort(&edges[axis][0], &edges[axis][2 * nPrimitives],
              [](const BoundEdge &e0, const BoundEdge &e1) -> bool {
                  if (e0.t == e1.t)
                      return (int)e0.type < (int)e1.type;
                  else
                      return e0.t < e1.t;
              });

    // We want to compute a split edge depending on majorant. This code computes
    // majorants below and above the split edge for all possible splits. 
    float majorantBelow = primitiveRanges[edges[axis][0].primNum].upper;
    for (int i = 0; i < 2 * nPrimitives; i++)
    {
        if( majorantBelow < primitiveRanges[edges[axis][i].primNum].upper)
            majorantBelow = primitiveRanges[edges[axis][i].primNum].upper;
        edges[axis][i].majorants.lower = majorantBelow;
    }
    float majorantAbove = primitiveRanges[edges[axis][2 * nPrimitives - 1].primNum].upper;
    for (int i = 2 * nPrimitives - 1; i >= 0; i--)
    {
        if( majorantAbove < primitiveRanges[edges[axis][i].primNum].upper)
            majorantAbove = primitiveRanges[edges[axis][i].primNum].upper;
        edges[axis][i].majorants.upper = majorantAbove;
    }

    // Compute cost of all splits for _axis_ to find best
    int nBelow = 0, nAbove = nPrimitives;
    for (int i = 0; i < 2 * nPrimitives; ++i) {
        if (edges[axis][i].type == EdgeType::End) --nAbove;
        float edgeT = edges[axis][i].t;
        if (edgeT > nodeBounds.lower[axis] && edgeT < nodeBounds.upper[axis]) {
            // Compute cost for split at _i_th edge

            // Compute child surface areas for split at _edgeT_
            int otherAxis0 = (axis + 1) % 3, otherAxis1 = (axis + 2) % 3;
            float belowSA = 2 * (d[otherAxis0] * d[otherAxis1] +
                                 (edgeT - nodeBounds.lower[axis]) *
                                     (d[otherAxis0] + d[otherAxis1]));
            float aboveSA = 2 * (d[otherAxis0] * d[otherAxis1] +
                                 (nodeBounds.upper[axis] - edgeT) *
                                     (d[otherAxis0] + d[otherAxis1]));
            // this code also doesn't make sense, 
            float pBelow = belowSA * invTotalSA;// * ((edges[axis][i].majorants.lower - globalRange.lower) / (globalRange.upper-globalRange.lower));
            float pAbove = aboveSA * invTotalSA;// * ((edges[axis][i].majorants.upper - globalRange.lower) / (globalRange.upper-globalRange.lower));
            float eb = (nAbove == 0 || nBelow == 0) ? emptyBonus : 0;
            float cost =
                traversalCost +
                isectCost * (1 - eb) * (pBelow * nBelow + pAbove * nAbove);

            // Update best split if this is lowest cost so far
            if (cost < bestCost) {
                bestCost = cost;
                bestAxis = axis;
                bestOffset = i;
            }
        }
        if (edges[axis][i].type == EdgeType::Start) ++nBelow;
    }
    assert(nBelow == nPrimitives && nAbove == 0);//??

    // Create leaf if no good splits were found
    if (bestAxis == -1 && retries < 2) {
        ++retries;
        axis = (axis + 1) % 3;
        goto retrySplit;
    }
    if (bestCost > oldCost) ++badRefines;
    if ( nPrimitives <= maxPrims &&
        ((bestCost > 4 * oldCost && nPrimitives < 16) || bestAxis == -1 ||
        badRefines == 3)) {
        nodes[nodeNum].InitLeaf(primNums, nPrimitives, &primitiveIndices);
        return;
    }

    // force valid split...
    if (retries >= 2 && bestAxis == -1) { 
        bestAxis = 0;
        bestOffset = nPrimitives / 2;
    }

    // Classify primitives with respect to split
    uint32_t n0 = 0, n1 = 0;
    for (uint32_t i = 0; i < bestOffset; ++i)
        if (edges[bestAxis][i].type == EdgeType::Start)
            prims0[n0++] = edges[bestAxis][i].primNum;
    for (uint32_t i = bestOffset + 1; i < 2 * nPrimitives; ++i)
        if (edges[bestAxis][i].type == EdgeType::End)
            prims1[n1++] = edges[bestAxis][i].primNum;

    std::cout<<"Splitting level " << depth << " Recursing down... " << std::endl;

    // Recursively initialize children nodes
    float tSplit = edges[bestAxis][bestOffset].t;
    float2x3 bounds0 = nodeBounds, bounds1 = nodeBounds;
    bounds0.upper[bestAxis] = bounds1.lower[bestAxis] = tSplit;
    buildTree(nodeNum + 1, bounds0, allPrimBounds, prims0, n0, depth - 1, edges,
              prims0, prims1 + nPrimitives, badRefines);
    uint32_t aboveChild = nextFreeNode;
    nodes[nodeNum].InitInterior(bestAxis, aboveChild, tSplit);
    buildTree(aboveChild, bounds1, allPrimBounds, prims1, n1, depth - 1, edges,
              prims0, prims1 + nPrimitives, badRefines);
}

float2 KdTreeAccel::CalculateMajorants(uint32_t nodeIndex)
{
    auto node = nodes[nodeIndex];
    float2 absoluteRange;
    if(node.range.lower >= globalRange.lower &&
        node.range.upper <= globalRange.upper) //prune tree traversal if the values are initialized
        return node.range;
    else if(node.IsLeaf() && node.nPrimitives() > 0) //leaf:base case
    {
        if(node.nPrimitives() == 1)
            absoluteRange.extend(primitiveRanges[node.onePrimitive]);
        else
            for (size_t i = node.primitiveIndicesOffset; 
                    i < node.primitiveIndicesOffset + node.nPrimitives(); i++)
            {
                absoluteRange.extend(primitiveRanges[primitiveIndices[i]]);
            }
    }
    else if(!node.IsLeaf())
    {
        absoluteRange = CalculateMajorants(nodeIndex + 1);
        absoluteRange.extend(CalculateMajorants(node.AboveChild()));
    }
    else
    {
        absoluteRange.lower = globalRange.lower;
        absoluteRange.lower = globalRange.lower;
    }
    node.range.lower = absoluteRange.lower;
    node.range.upper = absoluteRange.upper;
    return absoluteRange;
}

/*bool KdTreeAccel::Intersect(const Ray &ray, SurfaceInteraction *isect) const {
    ProfilePhase p(Prof::AccelIntersect);
    // Compute initial parametric range of ray inside kd-tree extent
    float tMin, tMax;
    if (!bounds.IntersectP(ray, &tMin, &tMax)) {
        return false;
    }

    // Prepare to traverse kd-tree for ray
    Vector3f invDir(1 / ray.d.x, 1 / ray.d.y, 1 / ray.d.z);
    PBRT_CONSTEXPR int maxTodo = 64;
    KdToDo todo[maxTodo];
    int todoPos = 0;

    // Traverse kd-tree nodes in order for ray
    bool hit = false;
    const KdAccelNode *node = &nodes[0];
    while (node != nullptr) {
        // Bail out if we found a hit closer than the current node
        if (ray.tMax < tMin) break;
        if (!node->IsLeaf()) {
            // Process kd-tree interior node

            // Compute parametric distance along ray to split plane
            int axis = node->SplitAxis();
            float tPlane = (node->SplitPos() - ray.o[axis]) * invDir[axis];

            // Get node children pointers for ray
            const KdAccelNode *firstChild, *secondChild;
            int belowFirst =
                (ray.o[axis] < node->SplitPos()) ||
                (ray.o[axis] == node->SplitPos() && ray.d[axis] <= 0);
            if (belowFirst) {
                firstChild = node + 1;
                secondChild = &nodes[node->AboveChild()];
            } else {
                firstChild = &nodes[node->AboveChild()];
                secondChild = node + 1;
            }

            // Advance to next child node, possibly enqueue other child
            if (tPlane > tMax || tPlane <= 0)
                node = firstChild;
            else if (tPlane < tMin)
                node = secondChild;
            else {
                // Enqueue _secondChild_ in todo list
                todo[todoPos].node = secondChild;
                todo[todoPos].tMin = tPlane;
                todo[todoPos].tMax = tMax;
                ++todoPos;
                node = firstChild;
                tMax = tPlane;
            }
        } else {
            // Check for intersections inside leaf node
            int nPrimitives = node->nPrimitives();
            if (nPrimitives == 1) {
                const std::shared_ptr<umesh::Triangle> &p =
                    primitives[node->onePrimitive];
                // Check one primitive inside leaf node
                if (p->Intersect(ray, isect)) hit = true;
            } else {
                for (int i = 0; i < nPrimitives; ++i) {
                    int index =
                        primitiveIndices[node->primitiveIndicesOffset + i];
                    const std::shared_ptr<umesh::Triangle> &p = primitives[index];
                    // Check one primitive inside leaf node
                    if (p->Intersect(ray, isect)) hit = true;
                }
            }

            // Grab next node to process from todo list
            if (todoPos > 0) {
                --todoPos;
                node = todo[todoPos].node;
                tMin = todo[todoPos].tMin;
                tMax = todo[todoPos].tMax;
            } else
                break;
        }
    }
    return hit;
}*/

// bool KdTreeAccel::Intersect(const owl::vec3f& org, const owl::vec3f& dir) const { //TODO
//     //ProfilePhase p(Prof::AccelIntersectP);
//     // Compute initial parametric range of ray inside kd-tree extent
//     float tMin, tMax;
//     if (!IntersectBox(bounds, org, dir, &tMin, &tMax)) {
//         return false;
//     }

//     // Prepare to traverse kd-tree for ray
//     umesh::vec3f invDir(1 / dir.x, 1 / dir.y, 1 / dir.z);
//     constexpr int maxTodo = 64;
//     KdToDo todo[maxTodo];
//     int todoPos = 0;
//     const KdAccelNode *node = &nodes[0];
//     while (node != nullptr) 
//     {
//         if (node->IsLeaf()) 
//         {
//             // Check for shadow ray intersections inside leaf node
//             int nPrimitives = node->nPrimitives();
//             if (nPrimitives == 1) 
//             {
//                 /*const std::shared_ptr<umesh::Triangle> &p =
//                     primitives[node->onePrimitive];
//                 if (p->IntersectP(ray)) {
//                     return true;
//                 }*/
//                 printf("tested primitive %d on node : %p\n", node->onePrimitive, node);
//             } 
//             else 
//             {
//                 for (int i = 0; i < nPrimitives; ++i) {
//                     int primitiveIndex =
//                         primitiveIndices[node->primitiveIndicesOffset + i];
//                     printf("tested primitive %d on node : %p\n", primitiveIndex, node);
//                     /*const std::shared_ptr<umesh::Triangle> &prim =
//                         primitives[primitiveIndex];
//                     if (prim->IntersectP(ray)) {
//                         return true;
//                     }*/
//                 }
//             }

//             // Grab next node to process from todo list
//             if (todoPos > 0) {
//                 --todoPos;
//                 node = todo[todoPos].node;
//                 tMin = todo[todoPos].tMin;
//                 tMax = todo[todoPos].tMax;
//             } else
//                 break;
//         } else {
//             // Process kd-tree interior node

//             // Compute parametric distance along ray to split plane
//             int axis = node->SplitAxis();
//             float tPlane = (node->SplitPos() - org[axis]) * invDir[axis];

//             // Get node children pointers for ray
//             const KdAccelNode *firstChild, *secondChild;
//             int belowFirst =
//                 (org[axis] < node->SplitPos()) ||
//                 (org[axis] == node->SplitPos() && dir[axis] <= 0);
//             if (belowFirst) {
//                 firstChild = node + 1;
//                 secondChild = &nodes[node->AboveChild()];
//             } else {
//                 firstChild = &nodes[node->AboveChild()];
//                 secondChild = node + 1;
//             }

//             // Advance to next child node, possibly enqueue other child
//             if (tPlane > tMax || tPlane <= 0)
//                 node = firstChild;
//             else if (tPlane < tMin)
//                 node = secondChild;
//             else {
//                 // Enqueue _secondChild_ in todo list
//                 todo[todoPos].node = secondChild;
//                 todo[todoPos].tMin = tPlane;
//                 todo[todoPos].tMax = tMax;
//                 ++todoPos;
//                 node = firstChild;
//                 tMax = tPlane;
//             }
//         }
//     }
//     return false;
// }

std::shared_ptr<KdTreeAccel> CreateKdTreeAccelerator(
    std::vector<std::vector<float4>> particles) {
    int isectCost = 80;
    int travCost = 1;
    float emptyBonus = 0.5f;
    int maxPrims = 100000;//65535/4;
    // std::cout<<"Temporarily setting max prims to 2"<<std::endl;
    // maxPrims = 1;
    int maxDepth = -1;
    return std::make_shared<KdTreeAccel>(particles, isectCost, travCost, emptyBonus,
                                         maxPrims, maxDepth);
}
} // namespace Accelerator
