
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

    float tMin =  (bounds[  ray.sign[0]].x - ray.point.x) * ray.invdir.x;
    float tMax =  (bounds[1-ray.sign[0]].x - ray.point.x) * ray.invdir.x;
    float tyMin = (bounds[  ray.sign[1]].y - ray.point.y) * ray.invdir.y;
    float tyMax = (bounds[1-ray.sign[1]].y - ray.point.y) * ray.invdir.y;

    tMax *= 1 + 2 * gamma2_3;
    tyMax *= 1 + 2 * gamma2_3;

    if (tMin > tyMax || tyMin > tMax) {
        if (empty_or_flat()) printf("%f %f %f %f\n", tMin, tyMax, tyMin, tMax);
        return res;
    }
    if (tyMin > tMin) tMin = tyMin;
    if (tyMax < tMax) tMax = tyMax;

    float tzMin = (bounds[  ray.sign[2]].z - ray.point.z) * ray.invdir.z;
    float tzMax = (bounds[1-ray.sign[2]].z - ray.point.z) * ray.invdir.z;
    tzMax *= 1 + 2 * gamma2_3;

    if (tMin > tzMax || tzMin > tMax) return res;
    if (tzMin > tMin) tMin = tzMin;
    if (tzMax < tMax) tMax = tzMax;

    if (tMin < ray.dist_bounds[1] && tMax > 0) {
        res.hit = true;
        res.tmin = tMin;
        res.tmax = tMax;
    }
    return res;
}
