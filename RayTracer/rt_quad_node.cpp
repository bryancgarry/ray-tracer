//
//  rt_quad_node.cpp
//  JHU_605.667
//
//  Created by Bryan Garry.
//

#include "RayTracer/rt_quad_node.hpp"
#include <vector>

namespace cg
{

RTQuadNode::RTQuadNode(const Point3 &center, const Vector3 &u, const Vector3 &v)
    : center_(center), u_(u), v_(v)
{
    // Calculate the normal and projection constants
    normal_ = u_.cross(v_).normalize();
    w_inv_sq_ = 1.0f / u_.dot(u_);
    h_inv_sq_ = 1.0f / v_.dot(v_);

    // Build the AABB using vertex list constructor
    std::vector<Point3> corners(4);
    corners[0] = center_ + (u_ * 0.5f) + (v_ * 0.5f);
    corners[1] = center_ + (u_ * 0.5f) - (v_ * 0.5f);
    corners[2] = center_ - (u_ * 0.5f) + (v_ * 0.5f);
    corners[3] = center_ - (u_ * 0.5f) - (v_ * 0.5f);

    bbox_ = AABB(corners);
}

void RTQuadNode::find_closest_intersect(Ray3 ray, SceneState &current_state, SceneState &closest)
{
    float denom = normal_.dot(ray.d);

    if (std::abs(denom) > 1e-6)
    {
        float t = (center_ - ray.o).dot(normal_) / denom;

        if (t > 0.001f && t < closest.t_min)
        {
            Point3  int_pt = ray.o + ray.d * t;
            Vector3 rel_pt = int_pt - center_;

            float u_proj = rel_pt.dot(u_) * w_inv_sq_;
            float v_proj = rel_pt.dot(v_) * h_inv_sq_;

            if (std::abs(u_proj) <= 0.5f && std::abs(v_proj) <= 0.5f)
            {
                closest.t_min         = t;
                closest.geometry_node = this;
                closest.material_node = current_state.material_node;
                closest.texture_node  = current_state.texture_node;

                // Store normal and UV directly in closest — thread-safe,
                // no mutable last_hit_local_point_ member needed
                closest.hit_normal   = normal_;
                closest.hit_texcoord = Point2(u_proj + 0.5f, v_proj + 0.5f);
            }
        }
    }
}

bool RTQuadNode::does_intersect_exist(Ray3 ray, float d, SceneState &current_state)
{
    float denom = normal_.dot(ray.d);
    if (std::abs(denom) > 1e-6)
    {
        float t = (center_ - ray.o).dot(normal_) / denom;

        if (t > 0.001f && t < d)
        {
            Point3  int_pt = ray.o + ray.d * t;
            Vector3 rel_pt = int_pt - center_;

            float u_proj = rel_pt.dot(u_) * w_inv_sq_;
            float v_proj = rel_pt.dot(v_) * h_inv_sq_;

            if (std::abs(u_proj) <= 0.5f && std::abs(v_proj) <= 0.5f)
                return true;
        }
    }
    return false;
}

Vector3 RTQuadNode::get_normal(const Point3 &int_pt)
{
    return normal_;
}

Point2 RTQuadNode::get_texture_coord(const Point3 &)
{
    // No longer used — UV is written to closest.hit_texcoord during
    // find_closest_intersect. Kept for interface compatibility.
    return Point2(0.0f, 0.0f);
}

} // namespace cg

////
////  rt_quad_node.cpp
////  JHU_605.667
////
////  Created by Bryan Garry.
////
//
//#include "RayTracer/rt_quad_node.hpp"
//#include <vector>
//
//namespace cg
//{
//
//RTQuadNode::RTQuadNode(const Point3 &center, const Vector3 &u, const Vector3 &v)
//    : center_(center), u_(u), v_(v)
//{
//    // Calculate the normal and projection constants
//    normal_ = u_.cross(v_).normalize();
//    w_inv_sq_ = 1.0f / u_.dot(u_);
//    h_inv_sq_ = 1.0f / v_.dot(v_);
//
//    // Build the AABB using vertex list constructor
//    std::vector<Point3> corners(4);
//    corners[0] = center_ + (u_ * 0.5f) + (v_ * 0.5f);
//    corners[1] = center_ + (u_ * 0.5f) - (v_ * 0.5f);
//    corners[2] = center_ - (u_ * 0.5f) + (v_ * 0.5f);
//    corners[3] = center_ - (u_ * 0.5f) - (v_ * 0.5f);
//
//    bbox_ = AABB(corners);
//}
//
//void RTQuadNode::find_closest_intersect(Ray3 ray, SceneState &current_state, SceneState &closest)
//{
//    // Plane Intersection: t = (center - origin) * normal / (direction * normal)
//    float denom = normal_.dot(ray.d);
//    
//    // Avoid division by zero (ray parallel to plane)
//    if (std::abs(denom) > 1e-6)
//    {
//        float t = (center_ - ray.o).dot(normal_) / denom;
//
//        // Check if this is the closest valid hit
//        if (t > 0.001f && t < closest.t_min)
//        {
//            // Check if the hit point inside the quad
//            Point3 int_pt = ray.o + ray.d * t;
//            Vector3 rel_pt = int_pt - center_;
//            
//            // Project the relative point onto the U and V axes
//            float u_proj = rel_pt.dot(u_) * w_inv_sq_;
//            float v_proj = rel_pt.dot(v_) * h_inv_sq_;
//
//            // If the projections are within [-0.5, 0.5], it is inside the quad
//            if (std::abs(u_proj) <= 0.5f && std::abs(v_proj) <= 0.5f)
//            {
//                closest.t_min = t;
//                closest.geometry_node = this;
//                closest.material_node = current_state.material_node;
//                closest.texture_node = current_state.texture_node;
//                
//                // Store for texture mapping and normals
//                last_hit_local_point_ = int_pt;
//            }
//        }
//    }
//}
//
//bool RTQuadNode::does_intersect_exist(Ray3 ray, float d, SceneState &current_state)
//{
//    float denom = normal_.dot(ray.d);
//    if (std::abs(denom) > 1e-6)
//    {
//        float t = (center_ - ray.o).dot(normal_) / denom;
//        
//        // Hit must be further than EPSILON and closer than the light
//        if (t > 0.001f && t < d)
//        {
//            Point3 int_pt = ray.o + ray.d * t;
//            Vector3 rel_pt = int_pt - center_;
//            
//            float u_proj = rel_pt.dot(u_) * w_inv_sq_;
//            float v_proj = rel_pt.dot(v_) * h_inv_sq_;
//
//            if (std::abs(u_proj) <= 0.5f && std::abs(v_proj) <= 0.5f)
//                return true;
//        }
//    }
//    return false;
//}
//
//Vector3 RTQuadNode::get_normal(const Point3 &int_pt)
//{
//    return normal_;
//}
//
//Point2 RTQuadNode::get_texture_coord(const Point3 &)
//{
//    // Map local coordinates to UV space [0, 1]
//    Vector3 rel_pt = last_hit_local_point_ - center_;
//    float s = (rel_pt.dot(u_) * w_inv_sq_) + 0.5f;
//    float t = (rel_pt.dot(v_) * h_inv_sq_) + 0.5f;
//    return Point2(s, t);
//}
//
//} // namespace cg
