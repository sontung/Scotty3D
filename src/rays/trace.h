
#pragma once

#include "../lib/mathlib.h"

namespace PT {

struct Trace {
    size_t prim_id=-1;
    bool skip_able = true;
    bool hit = false;
    float distance = std::numeric_limits<float>::max();
    Vec3 position, normal, origin;
    int material = 0;
    size_t origin_prim = 0;

    static Trace min(const Trace& l, const Trace& r) {
        if(l.hit && r.hit) {
            if(l.distance < r.distance) return l;
            return r;
        }
        if(l.hit) return l;
        if(r.hit) return r;
        return {};
    }

    void transform(const Mat4& transform, const Mat4& norm) {
        position = transform * position;
        origin = transform * origin;
        normal = norm.rotate(normal).unit();
        distance = (position - origin).norm();
    }
};

} // namespace PT
