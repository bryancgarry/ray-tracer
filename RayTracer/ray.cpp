#include "RayTracer/ray.hpp"
#include "geometry/geometry.hpp"

namespace cg
{

Ray::Ray(const Ray3 &initial_ray, int32_t depth, float t)
{
    o = initial_ray.o;
    d = initial_ray.d;
    recursion_level_ = depth;
    threshold_ = t;
}

Ray Ray::get_refracted_ray(const Point3       &int_pt,
                           const Vector3      &normal,
                           const MaterialNode *mat_node,
                           bool               &total_internal_reflection)
{
    // When the ray hits from outside: cos_i > 0 so entering material
    // When the ray hits from inside: cos_i < 0 so exiting to air
    float cos_i = -d.dot(normal);

    float n1, n2;
    if (cos_i < 0.0f)
    {
        // Exiting from inside the material, going to air
        n1 = mat_node->get_index_of_refraction();
        n2 = 1.0f;
    }
    else
    {
        // Entering from air, going into the material
        n1 = 1.0f;
        n2 = mat_node->get_index_of_refraction();
    }

    // Ray3::refract handles the normal flip for the inside case
    Vector3 n_copy = normal;
    RayRefractionResult result = this->refract(int_pt, n_copy, n1, n2);

    total_internal_reflection = result.total_internal_refraction;

    // Offset origin slightly along the refracted direction to prevent self-intersection
    if (!total_internal_reflection)
        result.refracted_ray.o = int_pt + result.refracted_ray.d * 0.001f;

    return Ray(result.refracted_ray, recursion_level_ - 1, threshold_);
}

bool Ray::below_threshold()
{
    return (recursion_level_ <= 0 || threshold_ < EPSILON);
}

} // namespace cg
