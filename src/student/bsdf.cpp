
#include "../rays/bsdf.h"
#include "../util/rand.h"
#include <iostream>


namespace PT {

static Vec3 reflect(Vec3 dir) {

    // TODO (PathTracer): Task 5
    // Return reflection of dir about the surface normal (0,1,0).
    Vec3 normal;
    normal.x=0.0;
    normal.y=1.0;
    normal.z=0.0;
    float adot = dot(dir, normal);

    Vec3 res;
    res.x=-dir.x;
    res.y=dir.y;
    res.z=-dir.z;
    return res;

    return -dir+2*adot*normal;
}

static Vec3 refract(Vec3 out_dir, float index_of_refraction, bool& was_internal) {

    // TODO (PathTracer): Task 5
    // Use Snell's Law to refract out_dir through the surface.
    // Return the refracted direction. Set was_internal to true if
    // refraction does not occur due to total internal reflection,
    // and false otherwise.

    // When dot(out_dir,normal=(0,1,0)) is positive, then out_dir corresponds to a
    // ray exiting the surface into vaccum (ior = 1). However, note that
    // you should actually treat this case as _entering_ the surface, because
    // you want to compute the 'input' direction that would cause this output,
    // and to do so you can simply find the direction that out_dir would refract
    // _to_, as refraction is symmetric.
    Vec3 normal, res;
    normal.x=0.0;
    normal.y=1.0;
    normal.z=0.0;

    float cos_theta_i = out_dir.y;

    float sin2_theta_i = fmaxf(EPS_F, 1.0-cos_theta_i*cos_theta_i);
    float sin2_theta_t = index_of_refraction*index_of_refraction*sin2_theta_i;
    if (sin2_theta_t >= 1) {
        was_internal = true;
        return res;
    }
    float cos_theta_t = sqrtf(1.0-sin2_theta_t);
    res = index_of_refraction * -out_dir + (index_of_refraction * cos_theta_i - cos_theta_t) * normal;

    return res;
}

template <typename T, typename U, typename V>
inline T Clamp(T val, U low, V high) {
    if (val < low) return low;
    else if (val > high) return high;
    else return val;
}

static float compute_fr_dielectric(float cosThetaI, float etaI, float etaT) {
    cosThetaI = Clamp(cosThetaI, -1.0, 1.0);

    bool entering = cosThetaI > 0.f;
    if (!entering) {
        std::swap(etaI, etaT);
        cosThetaI = fabsf(cosThetaI);
    }
    float sin2_theta_i = fmaxf(EPS_F, 1.0-cosThetaI*cosThetaI);
    float sin_theta_i = sqrtf(sin2_theta_i);
    float sin_theta_t = etaI / etaT * sin_theta_i;
    if (sin_theta_t >= 1) {
        return 1.0;
    }
    float cos_theta_t = sqrtf(fmaxf(EPS_F, 1.0-sin_theta_t*sin_theta_t));
    float Rparl = ((etaT * cosThetaI) - (etaI * cos_theta_t)) / ((etaT * cosThetaI) + (etaI * cos_theta_t));
    float Rperp = ((etaI * cosThetaI) - (etaT * cos_theta_t)) / ((etaI * cosThetaI) + (etaT * cos_theta_t));
    return (Rparl * Rparl + Rperp * Rperp) / 2;
}

Scatter BSDF_Lambertian::scatter(Vec3 out_dir) const {

    // TODO (PathTracer): Task 4

    // Sample the BSDF distribution using the cosine-weighted hemisphere sampler.
    // You can use BSDF_Lambertian::evaluate() to compute attenuation.



    Scatter ret;
    ret.direction = sampler.sample();
    ret.attenuation = evaluate(out_dir, ret.direction);
    return ret;
}

Spectrum BSDF_Lambertian::evaluate(Vec3 out_dir, Vec3 in_dir) const {

    // TODO (PathTracer): Task 4

    // Compute the ratio of reflected/incoming radiance when light from in_dir
    // is reflected through out_dir: albedo * cos(theta).
    return albedo;
}

float BSDF_Lambertian::pdf(Vec3 out_dir, Vec3 in_dir) const {

    // TODO (PathTracer): Task 4

    // Compute the PDF for sampling in_dir from the cosine-weighted hemisphere distribution.
    return 1.0f/PI_F;
}

Scatter BSDF_Mirror::scatter(Vec3 out_dir) const {

    // TODO (PathTracer): Task 5

    Scatter ret;
    ret.direction = reflect(out_dir);
    ret.attenuation = reflectance;
    return ret;
}

Scatter BSDF_Glass::scatter(Vec3 out_dir) const {

    // TODO (PathTracer): Task 5

    // (1) Compute Fresnel coefficient. Tip: Schlick's approximation.
    // (2) Reflect or refract probabilistically based on Fresnel coefficient. Tip: RNG::coin_flip
    // (3) Compute attenuation based on reflectance or transmittance

    // Be wary of your eta1/eta2 ratio - are you entering or leaving the surface?
    // What happens upon total internal reflection?
    float etaA = 1.0f;
    float etaB = index_of_refraction;
    float Fr = compute_fr_dielectric(out_dir.y, etaA, etaB);
    Scatter ret;

    if (RNG::coin_flip(Fr)) {
//        std::cout<<"reflect\n";
        ret.direction = reflect(out_dir);
        ret.attenuation = reflectance*Fr/fabsf(ret.direction.y);
    } else {
        bool entering = out_dir.y > EPS_F;
        float etaI = entering ? etaA : etaB;
        float etaT = entering ? etaB : etaA;
        bool internal;
        Vec3 in_dir = refract(out_dir, etaI / etaT, internal);
        if (internal) {
//            std::cout<<"reflect internal\n";
//            printf("internal reflect %f\n", Fr);
            ret.direction = reflect(out_dir);
            ret.attenuation = reflectance*Fr/fabsf(ret.direction.y);
            return ret;
        }
        else {
//            std::cout<<"refract\n";
            ret.direction = in_dir;
            ret.attenuation = transmittance*(1-Fr)/fabsf(ret.direction.y);
        }
    }

    return ret;
}

Scatter BSDF_Refract::scatter(Vec3 out_dir) const {

    // OPTIONAL (PathTracer): Task 5

    // When debugging BSDF_Glass, it may be useful to compare to a pure-refraction BSDF

    Scatter ret;
    ret.direction = Vec3();
    ret.attenuation = Spectrum{};
    return ret;
}

Spectrum BSDF_Diffuse::emissive() const {
    return radiance;
}

} // namespace PT
