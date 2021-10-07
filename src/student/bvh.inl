
#include "../rays/bvh.h"
#include "debug.h"
#include <stack>
#include <iostream>

namespace PT {

template<typename Primitive>
void BVH<Primitive>::node_bbox_enclosing(size_t node_idx) {
    size_t start = nodes[node_idx].start;
    size_t size = nodes[node_idx].size;
    for (size_t i=start; i<start+size; i++) {
        nodes[node_idx].bbox.enclose(primitives[i].bbox());
    }
}

template<typename Primitive>
BBox BVH<Primitive>::enclose_box(size_t start, size_t size) {
    BBox box;
    for (size_t i=start; i<start+size; i++) {
        box.enclose(primitives[i].bbox());
    }
    return box;
}

template<typename Primitive>
size_t BVH<Primitive>::compute_bucket_index(Bucket* buckets, size_t prim_idx, size_t nb_buckets, int dim) {
    for (size_t i = 0; i<nb_buckets; i++) {
        if (primitive_centroids[prim_idx][dim] >= buckets[i].bounds_for_centroids.x) {
            if (primitive_centroids[prim_idx][dim] <= buckets[i].bounds_for_centroids.y) {
                return i;
            }
        }
    }
    return nb_buckets;
}


template<typename Primitive>
float BVH<Primitive>::compute_partition_cost(Bucket* buckets, size_t partition_idx,
                                             size_t nb_buckets, float bound_area) {
    Vec2 sa1;
    Vec2 sa2;
    sa1.x = 0.0; sa1.y = 0.0;
    sa2.x = 0.0; sa2.y = 0.0;
    BBox box1, box2;
    for (size_t i1=0; i1<partition_idx; i1++) {
        box1.enclose(buckets[i1].bbox);
        sa1.x += (float)buckets[i1].prims_idx_vec.size();
    }
    for (size_t i2=partition_idx; i2<nb_buckets; i2++) {
        box2.enclose(buckets[i2].bbox);
        sa2.x += (float)buckets[i2].prims_idx_vec.size();
    }

    if (sa1.x < 1.0 || sa2.x < 1.0) return FLT_MAX;  // if one of the split is empty, put maximum cost

    float cost1 = box1.surface_area()*sa1.x;
    float cost2 = box2.surface_area()*sa2.x;
    float total_cost = (cost1+cost2)/bound_area+1.0;

    return total_cost;
}

template<typename Primitive>
void BVH<Primitive>::sah_split(size_t parent_index,
                               size_t dim,
                               size_t nb_buckets,
                               size_t start,
                               size_t size,
                               BBox& global_bounds,
                               BBox& centroid_bounds){

    Bucket buckets[nb_buckets];
    size_t best_pid;

    // init buckets
    float max_centroid = centroid_bounds.max[dim];
    float min_centroid = centroid_bounds.min[dim];
    if (max_centroid-min_centroid < 0.00001) return;

    max_centroid += 0.000001; // To avoid
    min_centroid -= 0.000001; // numerical errors
    float step = (max_centroid-min_centroid)/(float)nb_buckets;

    for (size_t i=0; i<nb_buckets; i++) {
        buckets[i].bounds_for_centroids.x = min_centroid+step*i;
        buckets[i].bounds_for_centroids.y = min_centroid+step*(i+1);
    }

    // compute bucket for each primitive
    for (size_t i=0; i<size; i++) {
        size_t prim_id = nodes[parent_index].prims_idx_vec[i];
        size_t bucket_idx = compute_bucket_index(buckets, prim_id, nb_buckets, dim);
        assert(bucket_idx!=nb_buckets);
        buckets[bucket_idx].bbox.enclose(primitives[prim_id].bbox());
        buckets[bucket_idx].prims_idx_vec.push_back(prim_id);
    }

    float global_bound_area = global_bounds.surface_area();
    float min_cost = compute_partition_cost(buckets, 1, nb_buckets, global_bound_area);
    best_pid = 1;
    for (size_t pid=2; pid<nb_buckets; pid++) {
        float cost = compute_partition_cost(buckets, pid, nb_buckets, global_bound_area);
        if (cost < min_cost) {
            min_cost = cost;
            best_pid = pid;
        }
    }


    if (best_bucket_split.unset) {
        best_bucket_split.unset = false;
        best_bucket_split.best_split = best_pid;
        best_bucket_split.best_cost = min_cost;
        best_bucket_split.axis = dim;
        for (size_t i=0; i<nb_buckets; i++) {
            best_bucket_split.bucket_array[i] = buckets[i];
        }
    } else if (min_cost < best_bucket_split.best_cost) {
        best_bucket_split.best_split = best_pid;
        best_bucket_split.best_cost = min_cost;
        best_bucket_split.axis = dim;
        for (size_t i=0; i<nb_buckets; i++) {
            best_bucket_split.bucket_array[i] = buckets[i];
        }
    }
}


template<typename Primitive>
void BVH<Primitive>::build_helper_sah(size_t max_leaf_size, size_t parent_index, std::vector<size_t>& ordered_prims) {
    size_t start = nodes[parent_index].start;
    size_t size = nodes[parent_index].size;

    if (size < 1) {
        return;
    }
    else if (size == 1) {
        ordered_prims.push_back(nodes[parent_index].prims_idx_vec[0]);
    }
    else {
        BBox box1, box2;
        size_t left_child_idx = new_node(box1, 0, 0, 1, 1);
        size_t right_child_idx = new_node(box2, 0, 0, 2, 2);
        nodes[left_child_idx].id = left_child_idx;
        nodes[right_child_idx].id = right_child_idx;
        nodes[left_child_idx].parent_id = parent_index;
        nodes[right_child_idx].parent_id = parent_index;

        if (size >= max_leaf_size) {
            size_t nb_buckets;
            if (size > 12) nb_buckets = 12;
            else nb_buckets = size;

            // global bounds for all primitives
            BBox global_bounds, centroid_bounds;

            for (size_t i=0; i<size; i++) {
                size_t prim_id = nodes[parent_index].prims_idx_vec[i];
                global_bounds.enclose(primitives[prim_id].bbox());
                centroid_bounds.enclose(primitive_centroids[prim_id]);
            }

            // finding best split
            sah_split(parent_index, 0, nb_buckets, start, size, global_bounds, centroid_bounds);
            sah_split(parent_index, 1, nb_buckets, start, size, global_bounds, centroid_bounds);
            sah_split(parent_index, 2, nb_buckets, start, size, global_bounds, centroid_bounds);
            best_bucket_split.unset = true;

            // dont split if the cost is higher than earlier
            if (best_bucket_split.best_cost > nodes[parent_index].split_cost) {
                ordered_prims.insert(ordered_prims.end(),
                                     nodes[parent_index].prims_idx_vec.begin(), nodes[parent_index].prims_idx_vec.end());
                return;
            }

            // create nodes for the best split
            total_split_cost += best_bucket_split.best_cost;
            nodes[left_child_idx].split_cost = best_bucket_split.best_cost;
            nodes[right_child_idx].split_cost = best_bucket_split.best_cost;

            size_t bucket_size1 = 0;
            size_t bucket_size2 = 0;
            size_t best_pid = best_bucket_split.best_split;

            for (size_t i1=0; i1<best_pid; i1++) {
                for (size_t u=0; u<best_bucket_split.bucket_array[i1].prims_idx_vec.size(); u++) {
                    size_t prim_id = best_bucket_split.bucket_array[i1].prims_idx_vec[u];
                    nodes[left_child_idx].prims_idx_vec.push_back(prim_id);
                }
                bucket_size1 += best_bucket_split.bucket_array[i1].prims_idx_vec.size();
            }
            for (size_t i2=best_pid; i2<nb_buckets; i2++) {
                for (size_t u=0; u<best_bucket_split.bucket_array[i2].prims_idx_vec.size(); u++) {
                    size_t prim_id = best_bucket_split.bucket_array[i2].prims_idx_vec[u];
                    nodes[right_child_idx].prims_idx_vec.push_back(prim_id);
                }
                bucket_size2 += best_bucket_split.bucket_array[i2].prims_idx_vec.size();
            }

            nodes[left_child_idx].start = start;
            nodes[left_child_idx].size = bucket_size1;
            nodes[right_child_idx].start = start+bucket_size1;
            nodes[right_child_idx].size = bucket_size2;
            nodes[parent_index].axis = best_bucket_split.axis;
        } else {
            ordered_prims.insert(ordered_prims.end(),
                                 nodes[parent_index].prims_idx_vec.begin(), nodes[parent_index].prims_idx_vec.end());
            return;
        }

        nodes[parent_index].l = left_child_idx;
        nodes[parent_index].r = right_child_idx;
        nodes[left_child_idx].l = left_child_idx;
        nodes[left_child_idx].r = left_child_idx;
        nodes[right_child_idx].l = right_child_idx;
        nodes[right_child_idx].r = right_child_idx;

        build_helper_sah(max_leaf_size, left_child_idx, ordered_prims);
        build_helper_sah(max_leaf_size, right_child_idx, ordered_prims);
        return;
    }

}

template<typename Primitive>
void reorder(std::vector<Primitive>& vec, std::vector<size_t>& vOrder) {
    assert(vec.size() == vOrder.size());
    for( size_t vv = 0; vv < vec.size() - 1; ++vv ) {
        if (vOrder[vv] == vv) continue;
        size_t oo;
        for(oo = vv + 1; oo < vOrder.size(); ++oo) {
            if (vOrder[oo] == vv) break;
        }
        std::swap( vec[vv], vec[vOrder[vv]] );
        std::swap( vOrder[vv], vOrder[oo] );
    }
}

template<typename Primitive>
void BVH<Primitive>::build(std::vector<Primitive>&& prims, size_t max_leaf_size) {
    nodes.clear();
    primitives = std::move(prims);
    for (size_t i=0; i<primitives.size(); i++) {
        Vec3 prim_centroid = primitives[i].bbox().center();
        primitive_centroids.push_back(prim_centroid);
    }

    root_idx = 0;
    BBox root_box = enclose_box(0, primitives.size());
    new_node(root_box, 0, primitives.size(), 0, 0);

    if (!root_box.empty_or_flat()) {
        printf("Building BVH with leaf size=%zu\n", max_leaf_size);
        for (size_t i=0; i<primitives.size(); i++) nodes[root_idx].prims_idx_vec.push_back(i);
        std::vector<size_t> orderer_primitives;
        orderer_primitives.reserve(primitives.size());
        best_bucket_split.bucket_array.reserve(12);
        Bucket empty_bucket;
        for (size_t i=0; i<12; i++) best_bucket_split.bucket_array.push_back(empty_bucket);
        build_helper_sah(max_leaf_size, root_idx, orderer_primitives);
        reorder(primitives, orderer_primitives);
        for (size_t i=0; i<nodes.size(); i++) node_bbox_enclosing(i);

        // prune bad trees if the bbox of child node is not smaller than that of parent node
        for (size_t i=0; i<nodes.size(); i++) {
            size_t pid = nodes[i].parent_id;
            if (pid < i) {
                Vec3 diff1 = nodes[i].bbox.min-nodes[pid].bbox.min;
                Vec3 diff2 = nodes[i].bbox.max-nodes[pid].bbox.max;
                float diff = diff1.norm()+diff2.norm();
                if (fabsf(diff) < EPS_F) nodes[i].bad_box = true;
            }
        }

        for (size_t i=0; i<nodes.size(); i++) printf("%zu %zu %zu %zu %zu %zu %d %d\n", nodes[i].id, nodes[i].start, nodes[i].size,
                                                     nodes[i].l, nodes[i].r, nodes[i].parent_id, nodes[i].bbox.empty_or_flat(), nodes[i].bad_box);
        printf("Done building BVH with split cost = %f, %zu primitives\n", total_split_cost, primitives.size());
    }
}

template<typename Primitive>
void BVH<Primitive>::hit_helper(const Ray& ray, Trace& closest_hit,
                                const Node& current_node, SimpleTrace& hit_bbox) const {
    if (!hit_bbox.hit) {
        return;
    }
    if (!nodes[current_node.l].is_leaf() && !nodes[current_node.r].is_leaf()) {
        SimpleTrace hit_bbox_l = nodes[current_node.l].bbox.hit_simple(ray);
        SimpleTrace hit_bbox_r = nodes[current_node.r].bbox.hit_simple(ray);
        if (hit_bbox_l.tmin < hit_bbox_r.tmin && hit_bbox_l.hit && hit_bbox_r.hit) {
            hit_helper(ray, closest_hit, nodes[current_node.l], hit_bbox_l);
            hit_helper(ray, closest_hit, nodes[current_node.r], hit_bbox_r);
        } else {
            hit_helper(ray, closest_hit, nodes[current_node.r], hit_bbox_r);
            hit_helper(ray, closest_hit, nodes[current_node.l], hit_bbox_l);
        }
    } else {
        hit_helper(ray, closest_hit, nodes[current_node.l]);
        hit_helper(ray, closest_hit, nodes[current_node.r]);
    }
}

template<typename Primitive>
void BVH<Primitive>::hit_helper(const Ray& ray, Trace& closest_hit,
                                const Node& current_node) const {
    if (current_node.is_leaf()) {

        size_t start = current_node.start;
        size_t size = current_node.size;
        for (size_t i=start; i<start+size; i++) {
            if (i == ray.prev_prim_hit) {
                continue;
            }
            Trace hit = primitives[i].hit(ray);
            if (hit.hit && hit.skip_able) hit.prim_id = i;
            closest_hit = Trace::min(closest_hit, hit);
        }
    } else {
        if (current_node.bad_box) {
            hit_helper(ray, closest_hit, nodes[current_node.r]);
            hit_helper(ray, closest_hit, nodes[current_node.l]);
            return;
        }
        SimpleTrace hit_bbox = current_node.bbox.hit_simple(ray);
        if (!hit_bbox.hit) {
            return;
        }
        hit_helper(ray, closest_hit, nodes[current_node.r]);
        hit_helper(ray, closest_hit, nodes[current_node.l]);
    }
}

template<typename Primitive> Trace BVH<Primitive>::hit(const Ray& ray) const {

    // TODO (PathTracer): Task 3
    // Implement ray - BVH intersection test. A ray intersects
    // with a BVH aggregate if and only if it intersects a primitive in
    // the BVH that is not an aggregate.

    // The starter code simply iterates through all the primitives.
    // Again, remember you can use hit() on any Primitive value.

    Trace ret;
    hit_helper(ray, ret, nodes[root_idx]);

    return ret;
}

template<typename Primitive>
BVH<Primitive>::BVH(std::vector<Primitive>&& prims, size_t max_leaf_size) {
    build(std::move(prims), max_leaf_size);
}

template<typename Primitive> BVH<Primitive> BVH<Primitive>::copy() const {
    BVH<Primitive> ret;
    ret.nodes = nodes;
    ret.primitives = primitives;
    ret.root_idx = root_idx;
    return ret;
}

template<typename Primitive> bool BVH<Primitive>::Node::is_leaf() const {

    // A node is a leaf if l == r, since all interior nodes must have distinct children
    return l == r;
}

template<typename Primitive>
size_t BVH<Primitive>::new_node(BBox box, size_t start, size_t size, size_t l, size_t r) {
    Node n;
    n.bbox = box;
    n.start = start;
    n.size = size;
    n.l = l;
    n.r = r;
    nodes.push_back(n);
    return nodes.size() - 1;
}

template<typename Primitive> BBox BVH<Primitive>::bbox() const {
    return nodes[root_idx].bbox;
}

template<typename Primitive> std::vector<Primitive> BVH<Primitive>::destructure() {
    nodes.clear();
    return std::move(primitives);
}

template<typename Primitive> void BVH<Primitive>::clear() {
    nodes.clear();
    primitives.clear();
}

template<typename Primitive>
size_t BVH<Primitive>::visualize(GL::Lines& lines, GL::Lines& active, size_t level,
                                 const Mat4& trans) const {

    std::stack<std::pair<size_t, size_t>> tstack;
    tstack.push({root_idx, 0});
    size_t max_level = 0;

    if(nodes.empty()) return max_level;

    while(!tstack.empty()) {

        auto [idx, lvl] = tstack.top();
                max_level = std::max(max_level, lvl);
                const Node& node = nodes[idx];
                tstack.pop();

                Vec3 color = lvl == level ? Vec3(1.0f, 0.0f, 0.0f) : Vec3(1.0f);
                GL::Lines& add = lvl == level ? active : lines;

                BBox box = node.bbox;
                box.transform(trans);
                Vec3 min = box.min, max = box.max;

                auto edge = [&](Vec3 a, Vec3 b) { add.add(a, b, color); };

        edge(min, Vec3{max.x, min.y, min.z});
        edge(min, Vec3{min.x, max.y, min.z});
        edge(min, Vec3{min.x, min.y, max.z});
        edge(max, Vec3{min.x, max.y, max.z});
        edge(max, Vec3{max.x, min.y, max.z});
        edge(max, Vec3{max.x, max.y, min.z});
        edge(Vec3{min.x, max.y, min.z}, Vec3{max.x, max.y, min.z});
        edge(Vec3{min.x, max.y, min.z}, Vec3{min.x, max.y, max.z});
        edge(Vec3{min.x, min.y, max.z}, Vec3{max.x, min.y, max.z});
        edge(Vec3{min.x, min.y, max.z}, Vec3{min.x, max.y, max.z});
        edge(Vec3{max.x, min.y, min.z}, Vec3{max.x, max.y, min.z});
        edge(Vec3{max.x, min.y, min.z}, Vec3{max.x, min.y, max.z});

        if(!node.is_leaf()) {
            tstack.push({node.l, lvl + 1});
            tstack.push({node.r, lvl + 1});
        } else {
            for(size_t i = node.start; i < node.start + node.size; i++) {
                size_t c = primitives[i].visualize(lines, active, level - lvl, trans);
                max_level = std::max(c + lvl, max_level);
            }
        }
    }
    return max_level;
}

} // namespace PT
