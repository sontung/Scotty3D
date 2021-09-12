
#include "../lib/mathlib.h"
#include "debug.h"
#include <iostream>

bool BBox::hit(const Ray& ray, Vec2& times) const {

    // TODO (PathTracer):
    // Implement ray - bounding box intersection test
    // If the ray intersected the bounding box within the range given by
    // [times.x,times.y], update times with the new intersection times.

    return false;
}

static constexpr float MachineEpsilon =
       std::numeric_limits<float>::epsilon() * 0.5;
inline constexpr float gamma2(int n) {
    return (n * MachineEpsilon) / (1 - n * MachineEpsilon);
}

static constexpr float gamma2_3 = gamma2(3);

SimpleTrace BBox::hit_simple(const Ray& ray) const {
    SimpleTrace res;
    res.hit = false;

    float t0 = ray.dist_bounds.x, t1 = ray.dist_bounds.y;
    for (int i = 0; i < 3; ++i) {
        float invRayDir = ray.invdir[i];
        float tNear = (min[i] - ray.point[i]) * invRayDir;
        float tFar  = (max[i] - ray.point[i]) * invRayDir;
        if (tNear > tFar) {
            std::swap(tNear, tFar);
        }
        tFar *= 1 + 2 * gamma2_3;

        t0 = tNear > t0 ? tNear : t0;
        t1 = tFar  < t1 ? tFar  : t1;
        if (t0 > t1) return res;

    }
    res.hit = true;
    res.distance = t0;
    return res;





//        float tmin, tmax, tymin, tymax, tzmin, tzmax;
//        Vec3 bounds[] = {min, max};
//        tmin = (bounds[ray.sign[0]].x - ray.point.x) * ray.invdir.x;
//        tmax = (bounds[1-ray.sign[0]].x - ray.point.x) * ray.invdir.x;
//        tymin = (bounds[ray.sign[1]].y - ray.point.y) * ray.invdir.y;
//        tymax = (bounds[1-ray.sign[1]].y - ray.point.y) * ray.invdir.y;

//        if ((tmin > tymax) || (tymin > tmax)) {
//            return res;}
//        if (tymin > tmin)
//            tmin = tymin;
//        if (tymax < tmax)
//            tmax = tymax;

//        tzmin = (bounds[ray.sign[2]].z - ray.point.z) * ray.invdir.z;
//        tzmax = (bounds[1-ray.sign[2]].z - ray.point.z) * ray.invdir.z;
//        if ((tmin > tzmax) || (tzmin > tmax)){
//            return res;}
//        if (tzmin > tmin)
//            tmin = tzmin;
//        if (tzmax < tmax)
//            tmax = tzmax;
//        res.hit = true;
//        if (tmin > 0) res.distance = tmin;
//        else res.distance = tmax;
    return res;
}
