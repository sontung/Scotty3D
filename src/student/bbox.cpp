
#include "../lib/mathlib.h"
#include "debug.h"
#include <iostream>


SimpleTrace BBox::hit(const Ray& r) const {
    float t_min, t_max;
    t_min = r.dist_bounds.x;
    t_max = r.dist_bounds.y;
    SimpleTrace ret;

    for (int a = 0; a < 3; a++) {
        auto invD = 1.0f / r.dir[a];
        auto t0 = (min[a] - r.point[a]) * invD;
        auto t1 = (max[a] - r.point[a]) * invD;
        if (invD < EPS_F)
            std::swap(t0, t1);
        t_min = t0 > t_min ? t0 : t_min;
        t_max = t1 < t_max ? t1 : t_max;
        if (t_max < t_min)
            return ret;
    }
    ret.hit = true;
    ret.distance = t_min;
    return ret;
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
    if (tMin < ray.dist_bounds.y && tMax > ray.dist_bounds.x) {
        res.hit = true;
        res.distance = tMax;
    }
    return res;
}
