#include "../util/rand.h"
#include "../util/camera.h"
#include "../rays/samplers.h"
#include "debug.h"

Ray Camera::generate_ray(Vec2 screen_coord) const {

    // TODO (PathTracer): Task 1
    // compute position of the input sensor sample coordinate on the
    // canonical sensor plane one unit away from the pinhole.
    // Tip: compute the ray direction in view space and use
    // the camera transform to transform it back into world space.
    float sensor_height=tan(Radians(vert_fov)/2)*focal_dist;
    float sensor_width=sensor_height*aspect_ratio;
    printf("focal %f ar %f\n", focal_dist, aspect_ratio);
    Vec2 cam_space_coord = screen_coord-0.5;
    Vec3 ray_dir(cam_space_coord.x*sensor_width/0.5, cam_space_coord.y*sensor_height/0.5, -focal_dist);

    Vec3 ray_origin(0.0f, 0.0f, 0.0f);

    if (aperture>0) {
        ray_origin.x = aperture*RNG::unit()-aperture/2;
        ray_origin.y = aperture*RNG::unit()-aperture/2;
    }
    Vec3 ray_dir_world_space = Camera::iview*ray_dir;
    Vec3 ray_origin_world_space = Camera::iview*ray_origin;
    Vec3 final_ray = ray_dir_world_space-ray_origin_world_space;
    final_ray.normalize();
    return Ray(ray_origin_world_space, final_ray);
}
