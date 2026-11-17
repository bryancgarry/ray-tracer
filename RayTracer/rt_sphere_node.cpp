#include "RayTracer/rt_sphere_node.hpp"
#include "geometry/geometry.hpp"

namespace cg
{

RTSphereNode::RTSphereNode(const Point3 &center, float radius)
{
    sphere_.center = center;
    sphere_.radius = radius;
}

RayObjectIntersectResult RTSphereNode::intersect(const Ray3 &ray) const
{
    return ray.intersect(sphere_);
}

Vector3 RTSphereNode::get_normal(const Point3 &int_pt)
{
    Vector3 n(sphere_.center, int_pt);
    n.normalize();
    return n;
}

void RTSphereNode::find_closest_intersect(Ray3 ray, SceneState &current_state, SceneState &closest)
{
    // Intersect directly with the sphere in World Space
    RayObjectIntersectResult result = ray.intersect(sphere_);

    // Check for valid hit.
    if (result.intersects && result.distance > 0.001f)
    {
        // Update closest if this is the nearest object found so far
        if (result.distance < closest.t_min)
        {
            closest.t_min         = result.distance;
            closest.geometry_node = this;
            closest.material_node = current_state.material_node;
            closest.texture_node  = current_state.texture_node;

            // Compute and store normal directly in closest — thread-safe,
            // no mutable member needed
            Point3 hit_pt = ray.o + ray.d * result.distance;
            Vector3 n(sphere_.center, hit_pt);
            n.normalize();
            closest.hit_normal = n;

            // Spherical UV coordinates
            Vector3 vp = hit_pt - sphere_.center;
            vp.normalize();
            float phi   = acos(-vp.y);
            float theta = atan2(vp.z, vp.x);
            closest.hit_texcoord = Point2((theta + static_cast<float>(M_PI)) / (2.0f * static_cast<float>(M_PI)),
                                          phi / static_cast<float>(M_PI));
        }
    }
}

bool RTSphereNode::does_intersect_exist(Ray3 ray, float d, SceneState &current_state)
{
    // Check intersect in world space and return result
    RayObjectIntersectResult result = ray.intersect(sphere_);
    return (result.intersects && result.distance > 0.001f && result.distance < d);
}

bool RTSphereNode::is_convex(void) const
{
    return true;
}

Point2 RTSphereNode::get_texture_coord(const Point3 &int_pt)
{
    // Get the direction vector from center to the hit point
    Vector3 vp = (int_pt - sphere_.center);
    vp.normalize();

    // Calculate spherical coordinates
    float phi = acos(-vp.y);
    float theta = atan2(vp.z, vp.x);

    // Map to [0, 1] range for UV coordinates
    float s = (theta + M_PI) / (2.0f * M_PI);
    float t = phi / M_PI;

    return Point2(s, t);
}

} // namespace cg

//#include "RayTracer/rt_sphere_node.hpp"
//#include "geometry/geometry.hpp"
//
//namespace cg
//{
//
//RTSphereNode::RTSphereNode(const Point3 &center, float radius)
//{
//    sphere_.center = center;
//    sphere_.radius = radius;
//}
//
//RayObjectIntersectResult RTSphereNode::intersect(const Ray3 &ray) const
//{
//    return ray.intersect(sphere_);
//}
//
//Vector3 RTSphereNode::get_normal(const Point3 &int_pt)
//{
//    Vector3 n(sphere_.center, int_pt);
//    n.normalize();
//    return n;
//}
//
//void RTSphereNode::find_closest_intersect(Ray3 ray, SceneState &current_state, SceneState &closest)
//{
//    // Intersect directly with the sphere in World Space
//    RayObjectIntersectResult result = ray.intersect(sphere_);
//
//    // Check for valid hit.
//    if (result.intersects && result.distance > 0.001f)
//    {
//        // Update closest if this is the nearest object found so far
//        if (result.distance < closest.t_min)
//        {
//            closest.t_min = result.distance;
//            closest.geometry_node = this;
//            closest.material_node = current_state.material_node;
//            closest.texture_node = current_state.texture_node;
//
//            // Store the hit point in world space for lighting/normals later
//            last_hit_local_point_ = ray.o + ray.d * result.distance;
//        }
//    }
//}
//
//bool RTSphereNode::does_intersect_exist(Ray3 ray, float d, SceneState &current_state)
//{
//    // Check intersect in world space and return result
//    RayObjectIntersectResult result = ray.intersect(sphere_);
//    return (result.intersects && result.distance > 0.001f && result.distance < d);
//}
//
//bool RTSphereNode::is_convex(void) const
//{
//    return true;
//}
//
//Point2 RTSphereNode::get_texture_coord(const Point3 &int_pt)
//{
//    // Get the direction vector from center to the hit point
//    Vector3 vp = (int_pt - sphere_.center);
//    vp.normalize();
//
//    // Calculate spherical coordinates
//    float phi = acos(-vp.y);
//    float theta = atan2(vp.z, vp.x);
//
//    // Map to [0, 1] range for UV coordinates
//    float s = (theta + M_PI) / (2.0f * M_PI);
//    float t = phi / M_PI;
//
//    return Point2(s, t);
//}
//
//} // namespace cg
