
#pragma once

#include "../lib/mathlib.h"
#include "../platform/gl.h"

#include "trace.h"

namespace PT {

template<typename Primitive> class BVH {
public:
    BVH() = default;
    BVH(std::vector<Primitive>&& primitives, size_t max_leaf_size = 1);
    void build(std::vector<Primitive>&& primitives, size_t max_leaf_size = 1);

    BVH(BVH&& src) = default;
    BVH& operator=(BVH&& src) = default;

    BVH(const BVH& src) = delete;
    BVH& operator=(const BVH& src) = delete;

    BBox bbox() const;
    Trace hit(const Ray& ray) const;

    BVH copy() const;
    size_t visualize(GL::Lines& lines, GL::Lines& active, size_t level, const Mat4& trans) const;

    std::vector<Primitive> destructure();
    void clear();

private:
    class Node {
        BBox bbox;
        size_t start, size, l, r, id;
        size_t level=0;
        std::vector<size_t> prims_idx_vec;
        float split_cost=FLT_MAX;
        bool is_leaf() const;
        friend class BVH<Primitive>;
    };

    class Bucket {
        BBox bbox;
        Vec2 bounds_for_centroids;
        std::vector<size_t> prims_idx_vec;
        friend class BVH<Primitive>;
    };

    class BestBucketSplit {
        bool unset=true;
        std::vector<Bucket> bucket_array;
        size_t best_split;
        float best_cost;
        friend class BVH<Primitive>;
    };

    size_t compute_bucket_index(Bucket* buckets, size_t prim_idx, size_t nb_buckets, int dim);
    float compute_partition_cost(Bucket* buckets, size_t partition_idx, size_t nb_buckets, float bound_area);
    void node_bbox_enclosing(size_t node_idx);
    void hit_helper(const Ray& ray, Trace& closest_hit,
                    const Node& current_node, size_t& times,
                    SimpleTrace& hit_bbox) const;
    BBox enclose_box(size_t start, size_t end);
    void build_helper_sah(size_t max_leaf_size, size_t parent_index, std::vector<size_t>& ordered_prims);
    void sah_split(size_t parent_index, size_t dim, size_t nb_buckets,
                   size_t start, size_t size,
                   BBox& global_bounds,
                   BBox& centroid_bounds);

    size_t new_node(BBox box = {}, size_t start = 0, size_t size = 0, size_t l = 0, size_t r = 0);


    BestBucketSplit best_bucket_split;
    float total_split_cost = 0.0;
    std::vector<Vec3> primitive_centroids;
    std::vector<Node> nodes;
    std::vector<Primitive> primitives;
    size_t root_idx = 0;
};

} // namespace PT

#ifdef SCOTTY3D_BUILD_REF
#include "../reference/bvh.inl"
#else
#include "../student/bvh.inl"
#endif
