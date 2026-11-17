//
//  rt_ellipsoid_node.hpp
//  JHU_605.667
//
//  Created by Bryan Garry

#ifndef __RAY_TRACER_RT_ELLIPSOID_NODE_HPP__
#define __RAY_TRACER_RT_ELLIPSOID_NODE_HPP__

#include "geometry/point3.hpp"
#include "geometry/ray3.hpp"
#include "geometry/vector3.hpp"
#include "scene/geometry_node.hpp"

namespace cg
{

/**
 * Ellipsoid for ray tracing using quadric equation.
 *
 * General quadric equation: Ax² + By² + Cz² + Dxy + Exz + Fyz + Gx + Hy + Iz + J = 0
 * For an axis-aligned ellipsoid centered at origin: x²/a² + y²/b² + z²/c² = 1
 * Where a, b, c are the semi-axes along x, y, z respectively.
 */
class RTEllipsoidNode : public GeometryNode
{
  public:
    /**
     * Constructor for a ray traced ellipsoid
     * @param  center    Ellipsoid center
     * @param  radius_x  Semi-axis length along x
     * @param  radius_y  Semi-axis length along y
     * @param  radius_z  Semi-axis length along z
     */
    RTEllipsoidNode(const Point3 &center, float radius_x, float radius_y, float radius_z);

    /**
     * Calculates the intersect of a ray and an ellipsoid by solving the quadric equation.
     *
     * Method: Substitute ray equation r(t) = o + td into ellipsoid equation:
     * (x-cx)²/a² + (y-cy)²/b² + (z-cz)²/c² = 1
     *
     * This gives a quadratic in t: At² + Bt + C = 0
     * Where we solve using the quadratic formula: t = (-B ± √(B²-4AC)) / 2A
     *
     * @param   ray  Ray to intersect with this ellipsoid
     * @return  Returns whether or not there is an intersection, and the distance
     *          at which the intersection occurs (0.0 if no intersection).
     */
    RayObjectIntersectResult intersect(const Ray3 &ray) const;

    /**
     * Gets the normal to the ellipsoid at the intersect point.
     * For ellipsoid (x-cx)²/a² + (y-cy)²/b² + (z-cz)²/c² = 1,
     * the normal is: (2(x-cx)/a², 2(y-cy)/b², 2(z-cz)/c²)
     *
     * @param   int_pt  Intersection point with the ellipsoid
     * @return  Returns a unit length normal at the intersection point.
     */
    Vector3 get_normal(const Point3 &int_pt) override;

    /**
     * Ray tracing intersect method
     */
    void find_closest_intersect(Ray3 ray, SceneState &current_state, SceneState &closest) override;

    /**
     * Ray tracing intersect method for shadows
     */
    bool does_intersect_exist(Ray3 ray, float d, SceneState &current_state) override;

    /**
     * Ellipsoids are always convex.
     */
    bool is_convex(void) const;
    
    /**
     * Get texture coordinates using spherical mapping
     */
    Point2 get_texture_coord(const Point3 &int_pt) override;

  private:
    Point3 center_;
    float radius_x_;  // Semi-axis along x
    float radius_y_;  // Semi-axis along y
    float radius_z_;  // Semi-axis along z
    
    // Stored values for efficient computation
    float inv_radius_x_sq_;
    float inv_radius_y_sq_;
    float inv_radius_z_sq_;
    
    mutable Point3 last_hit_local_point_;
};

} // namespace cg

#endif
