//
//  rt_ellipsoid_node.cpp
//  JHU_605.667
//
//  Created by Bryan Garry

#include "RayTracer/rt_ellipsoid_node.hpp"
#include "geometry/geometry.hpp"
#include <cmath>

namespace cg
{

RTEllipsoidNode::RTEllipsoidNode(const Point3 &center, float radius_x, float radius_y, float radius_z)
    : center_(center), radius_x_(radius_x), radius_y_(radius_y), radius_z_(radius_z)
{
    // Pre-compute inverse squared values for efficiency
    inv_radius_x_sq_ = 1.0f / (radius_x * radius_x);
    inv_radius_y_sq_ = 1.0f / (radius_y * radius_y);
    inv_radius_z_sq_ = 1.0f / (radius_z * radius_z);
}

RayObjectIntersectResult RTEllipsoidNode::intersect(const Ray3 &ray) const
{
    // Transform ray origin to ellipsoid-centric coordinates
    Vector3 Q = ray.o - center_;
    
    // Compute quadratic coefficients
    // A
    float A = ray.d.x * ray.d.x * inv_radius_x_sq_ +
              ray.d.y * ray.d.y * inv_radius_y_sq_ +
              ray.d.z * ray.d.z * inv_radius_z_sq_;
    
    // B
    float B = 2.0f * (Q.x * ray.d.x * inv_radius_x_sq_ +
                      Q.y * ray.d.y * inv_radius_y_sq_ +
                      Q.z * ray.d.z * inv_radius_z_sq_);
    
    // C
    float C = Q.x * Q.x * inv_radius_x_sq_ +
              Q.y * Q.y * inv_radius_y_sq_ +
              Q.z * Q.z * inv_radius_z_sq_ - 1.0f;
    
    // Solve quadratic equation using discriminant
    float discriminant = B * B - 4.0f * A * C;
    
    // No intersection if discriminant is negative
    if (discriminant < 0.0f)
    {
        return {false, 0.0f};
    }
    
    // Calculate t values using quadratic formula
    float sqrt_discriminant = std::sqrt(discriminant);
    float t1 = (-B - sqrt_discriminant) / (2.0f * A);
    float t2 = (-B + sqrt_discriminant) / (2.0f * A);
    
    // Return closest positive t value
    if (t1 > EPSILON)
    {
        return {true, t1};  // Nearest intersection
    }
    else if (t2 > EPSILON)
    {
        return {true, t2};  // Inside ellipsoid, exit point
    }
    
    // Both intersections behind the ray
    return {false, 0.0f};
}

Vector3 RTEllipsoidNode::get_normal(const Point3 &int_pt)
{
    Vector3 relative_pos = int_pt - center_;
    
    Vector3 n(relative_pos.x * inv_radius_x_sq_,
              relative_pos.y * inv_radius_y_sq_,
              relative_pos.z * inv_radius_z_sq_);
    
    n.normalize();
    return n;
}

void RTEllipsoidNode::find_closest_intersect(Ray3 ray, SceneState &current_state, SceneState &closest)
{
    // Intersect directly with the ellipsoid in world space
    RayObjectIntersectResult result = intersect(ray);

    if (result.intersects && result.distance > 0.001f)
    {
        if (result.distance < closest.t_min)
        {
            closest.t_min = result.distance;
            closest.geometry_node = this;
            closest.material_node = current_state.material_node;
            closest.texture_node = current_state.texture_node;

            // Store hit point for normal/texture calculations
            last_hit_local_point_ = ray.o + ray.d * result.distance;
        }
    }
}

bool RTEllipsoidNode::does_intersect_exist(Ray3 ray, float d, SceneState &current_state)
{
    RayObjectIntersectResult result = intersect(ray);
    return (result.intersects && result.distance > 0.001f && result.distance < d);
}

bool RTEllipsoidNode::is_convex(void) const
{
    return true;
}

Point2 RTEllipsoidNode::get_texture_coord(const Point3 &int_pt)
{
    // Transform to unit sphere space
    Vector3 normalized((int_pt.x - center_.x) / radius_x_,
                       (int_pt.y - center_.y) / radius_y_,
                       (int_pt.z - center_.z) / radius_z_);
    
    normalized.normalize();
    
    float phi = std::acos(-normalized.y);
    float theta = std::atan2(normalized.z, normalized.x);
    
    float s = (theta + M_PI) / (2.0f * M_PI);
    float t = phi / M_PI;
    
    return Point2(s, t);
}

} // namespace cg
