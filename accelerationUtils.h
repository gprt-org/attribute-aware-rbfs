#pragma once
#include <atomic>
#include <vector>


#include <gprt.h>

#ifdef _WIN32
#define INF INFINITE
#elif __linux__
#define INF INFINITY
#else
#define INF 999999999.0f
#endif

#if defined(__CUDA_ARCH__)
#define ACCEL_DECL __device__
#else
#define ACCEL_DECL
#endif

namespace Accelerator {

const size_t bum_magic = 0x111111115ULL +1;


//Some utils owl doesn't have
inline float SurfaceArea(const float2x3 &b) {
    float3 d = b.row(1) - b.row(0);
    return 2 * (d.x * d.y + d.x * d.z + d.y * d.z);
}

// inline ACCEL_DECL bool IntersectBox(const umesh::box4f bounds, const owl::vec3f& org, const owl::vec3f& dir, 
//         const owl::vec3f& invDir, const int dirIsNeg[3]) 
// {
//     float tMin =  ((dirIsNeg[0] ? bounds.upper : bounds.lower).x - org.x) * invDir.x;
//     float tMax =  ((dirIsNeg[0] ? bounds.lower : bounds.upper).x - org.x) * invDir.x;
//     float tyMin = ((dirIsNeg[1] ? bounds.upper : bounds.lower).y - org.y) * invDir.y;
//     float tyMax = ((dirIsNeg[1] ? bounds.lower : bounds.upper).y - org.y) * invDir.y;
    
//     if (tMin > tyMax || tyMin > tMax) 
//            return false;
//     if (tyMin > tMin) tMin = tyMin; 
//     if (tyMax < tMax) tMax = tyMax;
//     float tzMin = ((dirIsNeg[2] ? bounds.upper : bounds.lower).z - org.z) * invDir.z;
//     float tzMax = ((dirIsNeg[2] ? bounds.lower : bounds.upper).z - org.z) * invDir.z; 
//     if (tMin > tzMax || tzMin > tMax) 
//         return false; 
//     if (tzMin > tMin) 
//         tMin = tzMin; 
//     if (tzMax < tMax) 
//         tMax = tzMax;

//     return true; //todo
// }

inline ACCEL_DECL bool IntersectBox(
    const float3& boundsLower, 
    const float3& boundsUpper, 
    const float3& org, 
    const float3& dir, 
    const float3& invDir, 
    const int dirIsNeg[3], 
    float *hitt0 = nullptr, float *hitt1 = nullptr) 
{
    float tMin =  ((dirIsNeg[0] ? boundsUpper : boundsLower).x - org.x) * invDir.x;
    float tMax =  ((dirIsNeg[0] ? boundsLower : boundsUpper).x - org.x) * invDir.x;
    float tyMin = ((dirIsNeg[1] ? boundsUpper : boundsLower).y - org.y) * invDir.y;
    float tyMax = ((dirIsNeg[1] ? boundsLower : boundsUpper).y - org.y) * invDir.y;
    
    if (tMin > tyMax || tyMin > tMax) 
           return false;
    if (tyMin > tMin) tMin = tyMin; 
    if (tyMax < tMax) tMax = tyMax;
    float tzMin = ((dirIsNeg[2] ? boundsUpper : boundsLower).z - org.z) * invDir.z;
    float tzMax = ((dirIsNeg[2] ? boundsLower : boundsUpper).z - org.z) * invDir.z; 
    if (tMin > tzMax || tzMin > tMax) 
        return false; 
    if (tzMin > tMin) 
        tMin = tzMin; 
    if (tzMax < tMax) 
        tMax = tzMax;

    if (hitt0) *hitt0 = tMin;
    if (hitt1) *hitt1 = tMax;
    return true; //todo
}

// inline ACCEL_DECL bool IntersectBox(const owl::box3f bounds, const owl::vec3f& org, const owl::vec3f& dir,
//         float *hitt0, float *hitt1)
// {
//     float t0 = 0, t1 = INF; //ray.tMax;
//     for (int i = 0; i < 3; ++i) 
//     {
//            float invRayDir = 1 / dir[i];
//            float tNear = (bounds.lower[i] - org[i]) * invRayDir;
//            float tFar  = (bounds.upper[i] - org[i]) * invRayDir;
//               if (tNear > tFar) {
//                   float tmp = tNear;
//                   tNear = tFar;
//                   tFar = tmp;
//                 //   std::swap(tNear, tFar);
//               }
//               tFar *= 1 + 2 * lgammaf(3.f);

//               t0 = tNear > t0 ? tNear : t0;
//               t1 = tFar  < t1 ? tFar  : t1;
//               if (t0 > t1) return false;

//     }
//     if (hitt0) *hitt0 = t0;
//     if (hitt1) *hitt1 = t1;
//     return true;
// }

// inline ACCEL_DECL bool IntersectBox(const umesh::box3f bounds, const owl::vec3f& org, const owl::vec3f& dir,
//         float *hitt0, float *hitt1)
// {
//     float t0 = 0, t1 = INF; //ray.tMax;
//     for (int i = 0; i < 3; ++i) 
//     {
//            float invRayDir = 1 / dir[i];
//            float tNear = (bounds.lower[i] - org[i]) * invRayDir;
//            float tFar  = (bounds.upper[i] - org[i]) * invRayDir;
//               if (tNear > tFar) std::swap(tNear, tFar);
//                  tFar *= 1 + 2 * lgamma(3);

//               t0 = tNear > t0 ? tNear : t0;
//               t1 = tFar  < t1 ? tFar  : t1;
//               if (t0 > t1) return false;

//     }
//     if (hitt0) *hitt0 = t0;
//     if (hitt1) *hitt1 = t1;
//     return true;
// }

inline int MaximumExtent(const float2x3 &b) {
    float3 d = b.row(1) - b.row(0);
    if (d.x > d.y && d.x > d.z)
        return 0;
    else if (d.y > d.z)
        return 1;
    else
        return 2;
}

inline float2 extendedBounds(const float2 &b, const float &p)
{
    float2 ret;
    ret.x = linalg::min(b.x, p);
    ret.y = linalg::max(b.y, p);
    return ret;
}

inline float2 extendedBounds(const float2 &b, const float2 &p)
{
    float2 ret;
    auto tmp = linalg::min(b, p);
    ret.x = linalg::min(tmp.x, tmp.y);
    tmp = linalg::max(b, p);
    ret.y = linalg::max(tmp.x, tmp.y);
    return ret;
}

inline float2x3 extendedBounds(const float2x3 &b, const float3 &p)
{
    float2x3 ret = linalg::mat<float,2,3>({linalg::min(b.row(0).x, p.x),
        linalg::min(b.row(0).y, p.y)},
        {linalg::min(b.row(0).z, p.z),
        linalg::max(b.row(1).x, p.x)},
        {linalg::max(b.row(1).y, p.y),
        linalg::max(b.row(1).z, p.z)});
    return ret;
}

inline float2x3 extendedBounds(const float2x3 &b, const float2x3 &p)
{
    float2x3 ret = linalg::mat<float,2,3>({linalg::min(b.row(0).x, p.row(0).x),
        linalg::min(b.row(0).y, p.row(0).y)},
        {linalg::min(b.row(0).z, p.row(0).z),
        linalg::max(b.row(1).x, p.row(1).x)},
        {linalg::max(b.row(1).y, p.row(1).y),
        linalg::max(b.row(1).z, p.row(1).z)});
    return ret;
}

inline float2x3 extendedBounds(const float2x3 &b, const float4 &p)
{
    float2x3 ret = linalg::mat<float,2,3>({linalg::min(b.row(0).x, p.x),
        linalg::min(b.row(0).y, p.y)},
        {linalg::min(b.row(0).z, p.z),
        linalg::max(b.row(1).x, p.x)},
        {linalg::max(b.row(1).y, p.y),
        linalg::max(b.row(1).z, p.z)});
    return ret;
}

inline float2x4 extendedBounds(const float2x4 &b, const float4 &p)
{
    float2x4 ret = linalg::mat<float,2,4>({linalg::min(b.row(0).x, p.x),
        linalg::min(b.row(0).y, p.y)},
        {linalg::min(b.row(0).z, p.z),
        linalg::min(b.row(0).w, p.w)},
        {linalg::max(b.row(1).x, p.x),
        linalg::max(b.row(1).y, p.y)},
        {linalg::max(b.row(1).z, p.z),
        linalg::max(b.row(1).w, p.w)});
    return ret;
}

// inline int calculateVolumeFaceCount(std::shared_ptr<umesh::UMesh> umesh_ptr) {
    
//     return umesh_ptr->tets.size() * 3 + umesh_ptr->pyrs.size() * 4 +
//         umesh_ptr->wedges.size() * 5 + umesh_ptr->hexes.size() * 6;
// }

inline int Log2Int(uint32_t v) {
#ifdef _WIN32
    unsigned long lz = 0;
    if (_BitScanReverse(&lz, v)) return lz;
    return 0;
#else
    return 31 - __builtin_clz(v);
#endif
}

inline int Log2Int(int32_t v) { return Log2Int((uint32_t)v); }

inline int Log2Int(uint64_t v) {
#ifdef _WIN32
    unsigned long lz = 0;
#if defined(_WIN64)
    _BitScanReverse64(&lz, v);
#else
    if  (_BitScanReverse(&lz, v >> 32))
        lz += 32;
    else
        _BitScanReverse(&lz, v & 0xffffffff);
#endif // _WIN64
    return lz;
#else  // PBRT_IS_MSVC
    return 63 - __builtin_clzll(v);
#endif
}

inline int Log2Int(int64_t v) { return Log2Int((uint64_t)v); }

template <typename T>
inline constexpr bool IsPowerOf2(T v) {
    return v && !(v & (v - 1));
}

inline float3 Offset(const float2x3 &b, const float3 &p) 
{
    float3 o = p - b.row(0);
    if (b.row(1).x > b.row(0).x) o.x /= b.row(1).x - b.row(0).x;
    if (b.row(1).y > b.row(0).y) o.y /= b.row(1).y - b.row(0).y;
    if (b.row(1).z > b.row(0).z) o.z /= b.row(1).z - b.row(0).z;
    return o;
}

} //namespace Accelerator
