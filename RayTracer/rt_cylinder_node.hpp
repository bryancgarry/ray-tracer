//
//  rt_cylinder_node.hpp
//  JHU_605.667
//
//  Created by Bryan Garry.
//

#ifndef __RAY_TRACER_RT_CYLINDER_NODE_HPP__
#define __RAY_TRACER_RT_CYLINDER_NODE_HPP__

#include "geometry/point3.hpp"
#include "geometry/ray3.hpp"
#include "geometry/vector3.hpp"
#include "scene/geometry_node.hpp"

#include <cmath>

namespace cg
{

/**
 * Finite, capped cylinder aligned to the Z axis for ray tracing.
 * Z is the up/down axis in this scene. Defined by a center point, radius,
 * and half-height along Z.
 */
class RTCylinderNode : public GeometryNode
{
  public:
    /**
     * Constructor.
     * @param  center       Center of the cylinder (midpoint of the axis).
     * @param  radius       Cylinder radius.
     * @param  half_height  Half the total height (cylinder spans ±half_height from center.y).
     */
    RTCylinderNode(const Point3 &center, float radius, float half_height);

    /**
     * Intersects a ray with the cylinder body and end caps by solving
     * the quadric equation for the lateral surface and testing two disc planes
     * for the caps. Returns the nearest valid hit.
     * @param   ray  Ray to test.
     * @return  Intersection result with hit flag and distance t.
     */
    RayObjectIntersectResult intersect(const Ray3 &ray) const;

    /**
     * Returns the outward unit normal at a point on the cylinder surface.
     * For the body the normal is radial; for the caps it is ±Y.
     * NOTE: result is only valid after a successful find_closest_intersect call.
     */
    Vector3 get_normal(const Point3 &int_pt) override;

    /** Scene graph intersection for primary/secondary rays (updates closest hit). */
    void find_closest_intersect(Ray3 ray, SceneState &current_state,
                                SceneState &closest) override;

    /** Scene graph intersection for shadow rays (early exit). */
    bool does_intersect_exist(Ray3 ray, float d,
                              SceneState &current_state) override;

    /** Cylinders are convex. */
    bool is_convex(void) const;

    /**
     * Returns texture coordinates for the last intersection point.
     * Body: cylindrical mapping — s = θ/(2π), t = (y - bottom) / height.
     * Caps: disc mapping — s,t in [0,1] from radial position.
     * NOTE: result is only valid after a successful find_closest_intersect call.
     */
    Point2 get_texture_coord(const Point3 &int_pt) override;

  private:
    Point3 center_;      // Center of the cylinder
    float  radius_;      // Radius of the cylinder
    float  half_height_; // Half the total height
    float  inv_radius_;  // 1/radius, pre-computed for normal calculation

    // Cached state from the last successful intersection
    mutable Point3 last_hit_point_;
    mutable bool   last_hit_was_cap_;  // true = top/bottom cap, false = body
    mutable float  last_hit_cap_z_;    // z-coordinate of the hit cap (±half_height + center_.z)

    /**
     * Test a ray against a single end cap at height z_cap.
     * Returns EPSILON-guarded t, or -1 if no valid hit.
     */
    float intersect_cap(const Ray3 &ray, float z_cap) const;
};

} // namespace cg

#endif
