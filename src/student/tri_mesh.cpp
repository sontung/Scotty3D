
#include "../rays/tri_mesh.h"
#include "../rays/samplers.h"

namespace PT {

BBox Triangle::bbox() const {

    // TODO (PathTracer): Task 2
    // Compute the bounding box of the triangle.

    // Beware of flat/zero-volume boxes! You may need to
    // account for that here, or later on in BBox::intersect.

    Tri_Mesh_Vert v_0 = vertex_list[v0];
    Tri_Mesh_Vert v_1 = vertex_list[v1];
    Tri_Mesh_Vert v_2 = vertex_list[v2];
    (void)v_0;
    (void)v_1;
    (void)v_2;

    float x_min = fminf(fminf(v_0.position.x, v_1.position.x), v_2.position.x);
    float y_min = fminf(fminf(v_0.position.y, v_1.position.y), v_2.position.y);
    float z_min = fminf(fminf(v_0.position.z, v_1.position.z), v_2.position.z);
    float x_max = fmaxf(fmaxf(v_0.position.x, v_1.position.x), v_2.position.x);
    float y_max = fmaxf(fmaxf(v_0.position.y, v_1.position.y), v_2.position.y);
    float z_max = fmaxf(fmaxf(v_0.position.z, v_1.position.z), v_2.position.z);

    Vec3 min_extent(x_min, y_min, z_min);
    Vec3 max_extent(x_max, y_max, z_max);

    BBox box(min_extent, max_extent);
    return box;
}

void Triangle::transform_hit_results(Trace &ret) const {return;}

bool Triangle::hitP(const Ray& ray) const {

    // Vertices of triangle - has postion and surface normal
    Tri_Mesh_Vert v_0 = vertex_list[v0];
    Tri_Mesh_Vert v_1 = vertex_list[v1];
    Tri_Mesh_Vert v_2 = vertex_list[v2];
    (void)v_0;
    (void)v_1;
    (void)v_2;

    Vec3 s = ray.point-v_0.position;
    Vec3 cross_e1_d = cross(e1, ray.dir);

    float det = (dot(cross_e1_d, e2));
    if (det < EPS_F && det > -EPS_F) return false;

    return true;
}

Trace Triangle::hit(const Ray& ray) const {

    // Vertices of triangle - has postion and surface normal
    Tri_Mesh_Vert v_0 = vertex_list[v0];
    (void)v_0;

    Trace ret;

    Vec3 s = ray.point-v_0.position;
    Vec3 cross_e1_d = cross(e1, ray.dir);

    float det = (dot(cross_e1_d, e2));
    if (det < EPS_F && det > -EPS_F) return ret;

    float f = 1.0/det;
    Vec3 cross_s_e2 = cross(s, e2);
    float dot_s_e2_e1 = dot(cross_s_e2, e1);

    float t = -f*dot_s_e2_e1;
    if (t>ray.dist_bounds.y || t<EPS_F) {
        return ret;
    }

    float u = -f*dot(cross_s_e2, ray.dir);
    if (u < EPS_F || u > 1.0) return ret;
    float v = f*dot(cross_e1_d, s);
    if (v < EPS_F || u + v > 1.0) return ret;

    Tri_Mesh_Vert v_1 = vertex_list[v1];
    Tri_Mesh_Vert v_2 = vertex_list[v2];
    (void)v_1;
    (void)v_2;

    ray.dist_bounds.y = t;
    ret.origin = ray.point;
    ret.distance=t;
    ret.position=ray.point+t*ray.dir;
    ret.hit=true;
    ret.normal=(1.0-u-v)*v_0.normal+u*v_1.normal+v*v_2.normal;

    return ret;
}

Trace Triangle::hit_normal_only(const Ray& ray) const {
    Tri_Mesh_Vert v_0 = vertex_list[v0];
    Tri_Mesh_Vert v_1 = vertex_list[v1];
    Tri_Mesh_Vert v_2 = vertex_list[v2];
    (void)v_0;
    (void)v_1;
    (void)v_2;
    Trace ret;
    ret.special = true;
    ret.normal=(v_0.normal+v_1.normal+v_2.normal)/3.0f;
    return ret;
}

Triangle::Triangle(Tri_Mesh_Vert* verts, unsigned int v0, unsigned int v1, unsigned int v2)
    : vertex_list(verts), v0(v0), v1(v1), v2(v2) {
    Tri_Mesh_Vert v_0 = vertex_list[v0];
    Tri_Mesh_Vert v_1 = vertex_list[v1];
    Tri_Mesh_Vert v_2 = vertex_list[v2];
    (void)v_0;
    (void)v_1;
    (void)v_2;
    e1 = v_1.position-v_0.position;
    e2 = v_2.position-v_0.position;
}

Vec3 Triangle::sample(Vec3 from) const {
    Tri_Mesh_Vert v_0 = vertex_list[v0];
    Tri_Mesh_Vert v_1 = vertex_list[v1];
    Tri_Mesh_Vert v_2 = vertex_list[v2];
    Samplers::Triangle sampler(v_0.position, v_1.position, v_2.position);
    Vec3 pos = sampler.sample();
    return (pos - from).unit();
}

float Triangle::pdf(Ray wray, const Mat4& T, const Mat4& iT) const {

    Ray tray = wray;
    tray.transform(iT);

    Trace trace = hit(tray);
    if(trace.hit) {
        trace.transform(T, iT.T());
        Vec3 v_0 = T * vertex_list[v0].position;
        Vec3 v_1 = T * vertex_list[v1].position;
        Vec3 v_2 = T * vertex_list[v2].position;
        float a = 2.0f / cross(v_1 - v_0, v_2 - v_0).norm();
        float g =
                (trace.position - wray.point).norm_squared() / std::abs(dot(trace.normal, wray.dir));
        return a * g;
    }
    return 0.0f;
}

void Tri_Mesh::build(const GL::Mesh& mesh, bool bvh) {

    use_bvh = bvh;
    verts.clear();
    triangle_bvh.clear();
    triangle_list.clear();

    for(const auto& v : mesh.verts()) {
        verts.push_back({v.pos, v.norm});
    }

    const auto& idxs = mesh.indices();

    std::vector<Triangle> tris;
    for(size_t i = 0; i < idxs.size(); i += 3) {
        tris.push_back(Triangle(verts.data(), idxs[i], idxs[i + 1], idxs[i + 2]));
    }
    if(use_bvh) {
        triangle_bvh.build(std::move(tris), 4);
        if (bbox().empty_or_flat()) {
            for(const auto& v : mesh.verts()) {
                if (!set_norm_when_flat) {
                    set_norm_when_flat = true;
                    norm_when_flat = v.norm;
                } else {
                    Vec3 diff = norm_when_flat - v.norm;
                    assert(diff.norm()<EPS_F);
                }
            }

            std::vector<Triangle> tris;
            for(size_t i = 0; i < idxs.size(); i += 3) {
                tris.push_back(Triangle(verts.data(), idxs[i], idxs[i + 1], idxs[i + 2]));
            }
            triangle_list = List<Triangle>(std::move(tris));
        }
    } else {
        triangle_list = List<Triangle>(std::move(tris));
    }

}

Tri_Mesh::Tri_Mesh(const GL::Mesh& mesh, bool use_bvh) {
    build(mesh, use_bvh);
}

Tri_Mesh Tri_Mesh::copy() const {
    Tri_Mesh ret;
    ret.verts = verts;
    ret.triangle_bvh = triangle_bvh.copy();
    ret.triangle_list = triangle_list.copy();
    ret.use_bvh = use_bvh;
    return ret;
}

BBox Tri_Mesh::bbox() const {
    if(use_bvh) {
        return triangle_bvh.bbox();
    }
    return triangle_list.bbox();
}

Trace Tri_Mesh::hit(const Ray& ray) const {
    if (bbox().empty_or_flat()) {
        return triangle_list.hit_normal_only(ray);
    }
    if(use_bvh) {
        Trace ret = triangle_bvh.hit(ray);
        return ret;
    }
    return triangle_list.hit(ray);
}

bool Tri_Mesh::hitP(const Ray& ray) const {
    return triangle_list.hitP(ray);
}

size_t Tri_Mesh::visualize(GL::Lines& lines, GL::Lines& active, size_t level,
                           const Mat4& trans) const {
    if(use_bvh) return triangle_bvh.visualize(lines, active, level, trans);
    return 0;
}

Vec3 Tri_Mesh::sample(Vec3 from) const {
    if(use_bvh) {
        die("Sampling BVH-based triangle meshes is not yet supported.");
    }
    return triangle_list.sample(from);
}

float Tri_Mesh::pdf(Ray ray, const Mat4& T, const Mat4& iT) const {
    if(use_bvh) {
        die("Sampling BVH-based triangle meshes is not yet supported.");
    }
    return triangle_list.pdf(ray, T, iT);
}

} // namespace PT
