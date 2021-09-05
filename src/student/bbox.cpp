
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

SimpleTrace BBox::hit_simple(const Ray& ray) const {
    SimpleTrace res;
    res.hit = false;

    if (empty_or_flat()) {
        float denom = dot(normal, ray.dir);
        const float EPSILON = 0.0000001;
        if (denom > -EPSILON && denom < EPSILON) {return res;}
        Vec3 p0l0 = pointC-ray.point;
        float t = dot(p0l0, normal)/denom;

        if (t < 0.0 || t > ray.dist_bounds.y) {
            return res;
        }

        Vec3 hit_pos = ray.point+t*ray.dir;
        if (inside(hit_pos)) {
            res.hit = true;
            res.distance = t;
            return res;
        } else {
            return res;
        }
    }

    float tmin, tmax, tymin, tymax, tzmin, tzmax;
    Vec3 bounds[] = {min, max};
    tmin = (bounds[ray.sign[0]].x - ray.point.x) * ray.invdir.x;
    tmax = (bounds[1-ray.sign[0]].x - ray.point.x) * ray.invdir.x;
    tymin = (bounds[ray.sign[1]].y - ray.point.y) * ray.invdir.y;
    tymax = (bounds[1-ray.sign[1]].y - ray.point.y) * ray.invdir.y;

    if ((tmin > tymax) || (tymin > tmax))
        return res;
    if (tymin > tmin)
        tmin = tymin;
    if (tymax < tmax)
        tmax = tymax;

    tzmin = (bounds[ray.sign[2]].z - ray.point.z) * ray.invdir.z;
    tzmax = (bounds[1-ray.sign[2]].z - ray.point.z) * ray.invdir.z;
    if ((tmin > tzmax) || (tzmin > tmax))
        return res;
    if (tzmin > tmin)
        tmin = tzmin;
    if (tzmax < tmax)
        tmax = tzmax;
    res.hit = true;
    if (tmin > 0) res.distance = tmin;
    else res.distance = tmax;

    return res;
}
