
#pragma once

#include <algorithm>
#include <cfloat>
#include <cmath>
#include <ostream>
#include <vector>

#include "mat4.h"
#include "ray.h"
#include "vec2.h"
#include "vec3.h"

struct SimpleTrace {
    bool hit = false;
    float distance = std::numeric_limits<float>::max();
    float tmin=-1;
    float tmax=-1;
};

struct BBox {

    /// Default min is max float value, default max is negative max float value
    BBox() : min(FLT_MAX), max(-FLT_MAX) {
    }
    /// Set minimum and maximum extent
    explicit BBox(Vec3 min, Vec3 max) : min(min), max(max) {
    }

    BBox(const BBox&) = default;
    BBox& operator=(const BBox&) = default;
    ~BBox() = default;

    /// Rest min to max float, max to negative max float
    void reset() {
        min = Vec3(FLT_MAX);
        max = Vec3(-FLT_MAX);
        bounds[0] = min;
        bounds[1] = max;
    }

    /// Expand bounding box to include point
    void enclose(Vec3 point) {
        min = hmin(min, point);
        max = hmax(max, point);
        empty_x = fabsf(min.x-max.x) <= 0.00001;
        empty_y = fabsf(min.y-max.y) <= 0.00001;
        empty_z = fabsf(min.z-max.z) <= 0.00001;
        if (empty_or_flat()) {
            if (empty_x) {
                pointC.x = max.x;
                pointC.y = max.y;
                pointC.z = min.z;
            } else if (empty_y) {
                pointC.x = max.x;
                pointC.y = max.y;
                pointC.z = min.z;
            } else if (empty_z) {
                pointC.x = max.x;
                pointC.y = min.y;
                pointC.z = min.z;
            }
            normal = cross(pointC-min, pointC-max);
        }
        bounds[0] = min;
        bounds[1] = max;

    }
    void enclose(BBox box) {
        min = hmin(min, box.min);
        max = hmax(max, box.max);
        empty_x = fabsf(min.x-max.x) <= EPS_F;
        empty_y = fabsf(min.y-max.y) <= EPS_F;
        empty_z = fabsf(min.z-max.z) <= EPS_F;
        if (empty_or_flat()) {
            if (empty_x) {
                pointC.x = max.x;
                pointC.y = max.y;
                pointC.z = min.z;
            } else if (empty_y) {
                pointC.x = max.x;
                pointC.y = max.y;
                pointC.z = min.z;
            } else if (empty_z) {
                pointC.x = max.x;
                pointC.y = min.y;
                pointC.z = min.z;
            }
            normal = cross(pointC-min, pointC-max);
        }
        bounds[0] = min;
        bounds[1] = max;
    }

    /// Get center point of box
    Vec3 center() const {
        return (min + max) * 0.5f;
    }

    // Check whether box has no volume
    bool empty() const {
        return min.x > max.x || min.y > max.y || min.z > max.z;
    }

    bool empty_or_flat() const {
        return empty_x || empty_y || empty_z;
    }

    bool empty_x = false;
    bool empty_y = false;
    bool empty_z = false;
    Vec3 pointC, normal;  // for intersection test of flat box

    bool inside(Vec3 point) const {
        bool x = (point.x-min.x >= 0.0 && point.x-max.x <= 0.0) || empty_x;
        bool y = (point.y-min.y >= 0.0 && point.y-max.y <= 0.0) || empty_y;
        bool z = (point.z-min.z >= 0.0 && point.z-max.z <= 0.0) || empty_z;
        return x&&y&&z;
    }

    BBox intersect(BBox box) {
        Vec3 min_vec, max_vec;
        min_vec.x = std::max(min.x, box.min.x);
        min_vec.y = std::max(min.y, box.min.y);
        min_vec.z = std::max(min.z, box.min.z);
        max_vec.x = std::min(max.x, box.max.x);
        max_vec.y = std::min(max.y, box.max.y);
        max_vec.z = std::min(max.z, box.max.z);
        return BBox(min_vec, max_vec);
    }

    /// Get surface area of the box
    float surface_area() const {
        if(empty()) return 0.0f;
        Vec3 extent = max - min;
        return 2.0f * (extent.x * extent.z + extent.x * extent.y + extent.y * extent.z);
    }

    /// Transform box by a matrix
    void transform(const Mat4& trans) {
        Vec3 amin = min, amax = max;
        min = max = trans[3].xyz();
        for(int i = 0; i < 3; i++) {
            for(int j = 0; j < 3; j++) {
                float a = trans[j][i] * amin[j];
                float b = trans[j][i] * amax[j];
                if(a < b) {
                    min[i] += a;
                    max[i] += b;
                } else {
                    min[i] += b;
                    max[i] += a;
                }
            }
        }
    }

    // TODO (PathTracer): see student/bbox.cpp
    bool hit(const Ray& ray, Vec2& times) const;
    SimpleTrace hit_simple(const Ray& ray) const;


    /// Get the eight corner points of the bounding box
    std::vector<Vec3> corners() const {
        std::vector<Vec3> ret(8);
        ret[0] = Vec3(min.x, min.y, min.z);
        ret[1] = Vec3(max.x, min.y, min.z);
        ret[2] = Vec3(min.x, max.y, min.z);
        ret[3] = Vec3(min.x, min.y, max.z);
        ret[4] = Vec3(max.x, max.y, min.z);
        ret[5] = Vec3(min.x, max.y, max.z);
        ret[6] = Vec3(max.x, min.y, max.z);
        ret[7] = Vec3(max.x, max.y, max.z);
        return ret;
    }

    /// Given a screen transformation (projection), calculate screen-space ([-1,1]x[-1,1])
    /// bounds that will always contain the bounding box on screen
    void screen_rect(const Mat4& transform, Vec2& min_out, Vec2& max_out) const {

        min_out = Vec2(FLT_MAX);
        max_out = Vec2(-FLT_MAX);
        auto c = corners();
        bool partially_behind = false, all_behind = true;
        for(auto& v : c) {
            Vec3 p = transform * v;
            if(p.z < 0) {
                partially_behind = true;
            } else {
                all_behind = false;
            }
            min_out = hmin(min_out, Vec2(p.x, p.y));
            max_out = hmax(max_out, Vec2(p.x, p.y));
        }

        if(partially_behind && !all_behind) {
            min_out = Vec2(-1.0f, -1.0f);
            max_out = Vec2(1.0f, 1.0f);
        } else if(all_behind) {
            min_out = Vec2(0.0f, 0.0f);
            max_out = Vec2(0.0f, 0.0f);
        }
    }

    Vec3 min, max;
    Vec3 bounds[2] = {min, max};
};

inline std::ostream& operator<<(std::ostream& out, BBox b) {
    out << "BBox{" << b.min << "," << b.max << "}";
    return out;
}
