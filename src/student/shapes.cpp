
#include "../rays/shapes.h"
#include "debug.h"
#include <iostream>

namespace PT {

const char* Shape_Type_Names[(int)Shape_Type::count] = {"None", "Sphere"};

BBox Sphere::bbox() const {

    BBox box;
    box.enclose(Vec3(-radius));
    box.enclose(Vec3(radius));
    return box;
}

Trace Sphere::hit(const Ray& ray) const {

    // TODO (PathTracer): Task 2
    // Intersect this ray with a sphere of radius Sphere::radius centered at the origin.

    // If the ray intersects the sphere twice, ret should
    // represent the first intersection, but remember to respect
    // ray.dist_bounds! For example, if there are two intersections,
    // but only the _later_ one is within ray.dist_bounds, you should
    // return that one!
    Trace ret;
    ret.origin = ray.point;
    ret.hit = false;

    float a = ray.dir.norm_squared();
    float b = 2*dot(ray.point, ray.dir);
    float c = ray.point.norm_squared() - radius*radius;

    float discrim = b*b-4*a*c;
    if (discrim < EPS_F) return ret;
    float root_discrim = sqrtf(discrim);
    float q;
    if (b < EPS_F) q = -.5 * (b - root_discrim);
    else q = -.5 * (b + root_discrim);
    float t0 = q/a;
    float t1 = c/q;
    if (t0 > t1) std::swap(t0, t1);
    if (t0 > ray.dist_bounds.y || t1 < ray.dist_bounds.x) {
        return ret;
    }
    ret.skip_able = false;
    float tShapeHit = t0;
    if (tShapeHit < EPS_F) {
        tShapeHit = t1;
        if (tShapeHit > ray.dist_bounds.y)
            return ret;
    }
    assert(tShapeHit>EPS_F);
    ray.dist_bounds.y = tShapeHit;
    ret.distance = tShapeHit;
    ret.position = ray.point+tShapeHit*ray.dir;
    ret.hit = true;
    ret.normal = ret.position.unit();
    return ret;
}

} // namespace PT
