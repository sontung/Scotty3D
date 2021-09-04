
#include "../rays/shapes.h"
#include "debug.h"

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

    Vec3 l = -ray.point;
    float s = dot(l, ray.dir);
    float l2 = dot(l, l);
    float r2 = radius*radius;
    if (s<0 && l2>r2) return ret;
    float m2 = l2-s*s;
    if (m2>r2) return ret;
    float q = sqrtf(r2-m2);
    float t;
    if (l2>r2) t=s-q;
    else t=s+q;
    if (t > ray.dist_bounds.y) return ret;
    ray.dist_bounds.y = t;
    ret.distance = t;
    ret.position = ray.point+t*ray.dir;
    ret.hit = true;
    ret.normal = ret.position.unit();
    return ret;

//    float od = dot(ray.point, ray.dir);
//    float od_sq = od*od;
//    float o_norm_sq = ray.point.norm_squared();
//    float num = od_sq-o_norm_sq+radius*radius;


//    if (num < 0) {
//        ret.hit = false;
//        return ret;
//    }

//    float num_sq = sqrt(num);
//    float t1 = -od-num_sq;
//    if (t1 >= ray.dist_bounds.x && t1 <= ray.dist_bounds.y) {
//        ray.dist_bounds.y = t1;
//        ret.distance = t1;
//        ret.position = ray.point+t1*ray.dir;
//        ret.hit = true;
//        ret.normal = ret.position.unit();
//        return ret;
//    }
//    float t2 = -od+num_sq;
//    if (t2 >= ray.dist_bounds.x && t2 <= ray.dist_bounds.y) {
//        ray.dist_bounds.y = t2;
//        ret.distance = t2;
//        ret.position = ray.point+t2*ray.dir;
//        ret.hit = true;
//        ret.normal = ret.position.unit();
//        return ret;
//    }

//    return ret;
}

} // namespace PT
