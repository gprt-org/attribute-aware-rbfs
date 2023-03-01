#pragma once

// Usage: Inherit the following class, overriding the "lambda" function.
// Then, call "dda3" to traverse through the grid. 
class DDA {
  
  // override this function. Return true to continue traversal, and 
  // return false to terminate.
  bool lambda(int3 cell, float t0, float t1);

  void dda3(float3 org,
            float3 dir,
            float tMax,
            uint3 gridSize,
            bool dbg)
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
      t_nr.x = -1.#INF; t_nr.x = 1.#INF;
    }
    if (dir.y == 0.f) {
      if (org.y < 0.f || org.y > f_size.y)
        // ray passes by the volume ...
        return;
      t_nr.y = -1.#INF; t_nr.y = 1.#INF;
    }
    if (dir.z == 0.f) {
      if (org.z < 0.f || org.z > f_size.z)
        // ray passes by the volume ...
        return;
      t_nr.z = -1.#INF; t_nr.z = 1.#INF;
    }
    
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
      bool wantToGoOn = lambda(cell,cell_t0,cell_t1);
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
