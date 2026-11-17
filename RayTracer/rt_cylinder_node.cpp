//
//  rt_cylinder_node.cpp
//  JHU_605.667
//
//  Created by Bryan Garry.
//

#include "RayTracer/rt_cylinder_node.hpp"
#include "geometry/geometry.hpp"

#include <cmath>
#include <limits>

namespace cg
{

RTCylinderNode::RTCylinderNode(const Point3 &center, float radius, float half_height)
    : center_(center),
      radius_(radius),
      half_height_(half_height),
      inv_radius_(1.0f / radius),
      last_hit_was_cap_(false),
      last_hit_cap_z_(0.0f)
{
}

// ---- Private helper --------------------------------------------------------

float RTCylinderNode::intersect_cap(const Ray3 &ray, float z_cap) const
{
    // Intersect the ray with the horizontal plane z = z_cap.
    // t = (z_cap - ray.o.z) / ray.d.z
    if(std::abs(ray.d.z) < EPSILON) return -1.0f; // Ray parallel to cap plane

    float t = (z_cap - ray.o.z) / ray.d.z;
    if(t < EPSILON) return -1.0f; // Behind or at origin

    // Check if the hit point is within the disc radius
    float hx = ray.o.x + t * ray.d.x - center_.x;
    float hy = ray.o.y + t * ray.d.y - center_.y;
    if(hx * hx + hy * hy > radius_ * radius_) return -1.0f;

    return t;
}

// ---- Quadric intersection --------------------------------------------------
//
// Cylinder axis is Z. Substitute P(t) = O + tD into:
//   (x - cx)^2 + (y - cy)^2 = r^2
//
// Let qx = Ox - cx,  qy = Oy - cy.
//   A = Dx^2 + Dy^2
//   B = 2(qx*Dx + qy*Dy)
//   C = qx^2 + qy^2 - r^2
//
// Each candidate t is height-tested against [z_bottom, z_top].
// End caps are flat discs at those z planes.

RayObjectIntersectResult RTCylinderNode::intersect(const Ray3 &ray) const
{
    const float z_bottom = center_.z - half_height_;
    const float z_top    = center_.z + half_height_;

    float best_t      = std::numeric_limits<float>::max();
    bool  best_is_cap = false;
    float best_cap_z  = 0.0f;

    // ---- Lateral (body) intersection ----------------------------------------
    float qx = ray.o.x - center_.x;
    float qy = ray.o.y - center_.y;

    float a = ray.d.x * ray.d.x + ray.d.y * ray.d.y;

    if(a > EPSILON) // Ray is not parallel to the Z axis
    {
        float b    = 2.0f * (qx * ray.d.x + qy * ray.d.y);
        float c    = qx * qx + qy * qy - radius_ * radius_;
        float disc = b * b - 4.0f * a * c;

        if(disc >= 0.0f)
        {
            float sq    = std::sqrt(disc);
            float inv2a = 1.0f / (2.0f * a);
            float t1    = (-b - sq) * inv2a;
            float t2    = (-b + sq) * inv2a;

            // Test t1 (nearer body hit)
            if(t1 > EPSILON)
            {
                float z1 = ray.o.z + t1 * ray.d.z;
                if(z1 >= z_bottom && z1 <= z_top && t1 < best_t)
                {
                    best_t      = t1;
                    best_is_cap = false;
                }
            }
            // Test t2 (farther body hit — only relevant if origin is inside)
            if(t2 > EPSILON && t2 < best_t)
            {
                float z2 = ray.o.z + t2 * ray.d.z;
                if(z2 >= z_bottom && z2 <= z_top)
                {
                    best_t      = t2;
                    best_is_cap = false;
                }
            }
        }
    }

    // ---- Cap intersections --------------------------------------------------
    float t_top = intersect_cap(ray, z_top);
    if(t_top > EPSILON && t_top < best_t)
    {
        best_t      = t_top;
        best_is_cap = true;
        best_cap_z  = z_top;
    }

    float t_bot = intersect_cap(ray, z_bottom);
    if(t_bot > EPSILON && t_bot < best_t)
    {
        best_t      = t_bot;
        best_is_cap = true;
        best_cap_z  = z_bottom;
    }

    if(best_t < std::numeric_limits<float>::max())
    {
        last_hit_was_cap_ = best_is_cap;
        last_hit_cap_z_   = best_cap_z;
        return {true, best_t};
    }
    return {false, 0.0f};
}

// ---- Normal ----------------------------------------------------------------

Vector3 RTCylinderNode::get_normal(const Point3 &int_pt)
{
    if(last_hit_was_cap_)
    {
        // Top cap points up (+Z), bottom cap points down (-Z)
        return (last_hit_cap_z_ > center_.z) ? Vector3(0.0f, 0.0f, 1.0f)
                                              : Vector3(0.0f, 0.0f, -1.0f);
    }

    // Body normal: radial in XY plane, Z component is zero
    Vector3 n((int_pt.x - center_.x) * inv_radius_,
              (int_pt.y - center_.y) * inv_radius_,
               0.0f);
    n.normalize();
    return n;
}

// ---- Scene graph methods ---------------------------------------------------

void RTCylinderNode::find_closest_intersect(Ray3        ray,
                                             SceneState &current_state,
                                             SceneState &closest)
{
    RayObjectIntersectResult result = intersect(ray);

    if(result.intersects && result.distance > 0.001f && result.distance < closest.t_min)
    {
        closest.t_min         = result.distance;
        closest.geometry_node = this;
        closest.material_node = current_state.material_node;
        closest.texture_node  = current_state.texture_node;
        last_hit_point_       = ray.o + ray.d * result.distance;
    }
}

bool RTCylinderNode::does_intersect_exist(Ray3        ray,
                                           float       d,
                                           SceneState &current_state)
{
    RayObjectIntersectResult result = intersect(ray);
    return (result.intersects && result.distance > 0.001f && result.distance < d);
}

bool RTCylinderNode::is_convex(void) const { return true; }

// ---- Texture coordinates ---------------------------------------------------

Point2 RTCylinderNode::get_texture_coord(const Point3 &int_pt)
{
    if(last_hit_was_cap_)
    {
        // Disc mapping centered on cap
        float s = ((int_pt.x - center_.x) * inv_radius_) * 0.5f + 0.5f;
        float t = ((int_pt.y - center_.y) * inv_radius_) * 0.5f + 0.5f;
        return Point2(s, t);
    }

    // Cylindrical mapping:
    //   s = angle around Z axis [0, 1]
    //   t = height from bottom  [0, 1]
    float dx    = int_pt.x - center_.x;
    float dy    = int_pt.y - center_.y;
    float theta = std::atan2(dy, dx);

    float s = (theta + static_cast<float>(M_PI)) / (2.0f * static_cast<float>(M_PI));
    float t = (int_pt.z - (center_.z - half_height_)) / (2.0f * half_height_);

    return Point2(s, t);
}

} // namespace cg
