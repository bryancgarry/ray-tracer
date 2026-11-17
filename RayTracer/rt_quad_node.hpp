//
//  rt_quad_node.hpp
//  JHU_605.667
//
//  Created by Bryan Garry.
//

#ifndef __RAY_TRACER_RT_QUAD_NODE_HPP__
#define __RAY_TRACER_RT_QUAD_NODE_HPP__

#include "geometry/aabb.hpp" // Added AABB include
#include "geometry/point3.hpp"
#include "geometry/vector3.hpp"
#include "geometry/ray3.hpp"
#include "scene/geometry_node.hpp"

namespace cg
{

class RTQuadNode : public GeometryNode
{
  public:
    RTQuadNode(const Point3 &center, const Vector3 &u, const Vector3 &v);

    void find_closest_intersect(Ray3 ray, SceneState &current_state, SceneState &closest) override;
    bool does_intersect_exist(Ray3 ray, float d, SceneState &current_state) override;
    
    Vector3 get_normal(const Point3 &int_pt) override;
    Point2 get_texture_coord(const Point3 &int_pt) override;
    
    // Accessor for the bounding box (unsure if need right now)
    const AABB& get_aabb() const { return bbox_; }

  private:
    Point3 center_;
    Vector3 u_;
    Vector3 v_;
    Vector3 normal_;
    float w_inv_sq_;
    float h_inv_sq_;
    
    AABB bbox_;
    
//    Point3 last_hit_local_point_;
};

} // namespace cg

#endif
