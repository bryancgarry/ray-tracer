//============================================================================
//    Johns Hopkins University Whiting School of Engineering
//    605.767 Applied Computer Graphics
//    Instructor:    Brian Russin
//
//    Author:     David W. Nesbitt
//    File:    ray_tracer.hpp
//    Purpose: Recursive ray tracer.
//
//============================================================================

#ifndef __RAY_TRACER_RAY_TRACER_HPP__
#define __RAY_TRACER_RAY_TRACER_HPP__

#include "RayTracer/lighting.hpp"
#include "RayTracer/procedural_texture.hpp"
#include "RayTracer/ray.hpp"
#include "scene/geometry_node.hpp"

#include <memory>
#include <vector>

namespace cg
{

/**
 * Ray tracer class. Performs recursive ray tracing.
 */
class RayTracer
{
  public:
    /**
     * Constructor.
     */
    RayTracer(std::shared_ptr<SceneNode> scene_root);

    ~RayTracer();

    /**
     * Initial call to trace a ray.
     */
    Color3 trace_ray(Ray3 &initial_ray, int depth, float adaptive_threshold);

    /**
     * Recursive method to trace a ray
     */
    Color3 trace_ray(Ray &ray);

    /**
     * Set the view position (for lighting).
     */
    void set_view_position(const Point3 &pos);
    
    // Add light source to ray tracer's internal list
    void add_light(std::shared_ptr<LightNode> light);

  private:
    Lighting                   lighting_;
    std::shared_ptr<SceneNode> scene_root_;
    std::vector<std::shared_ptr<LightNode>> lights_;

    // Test to see if intersect point is in shadow relative to a specific light's position
    bool in_shadow(const Point3 &int_pt, const Point3 &light_pos, SceneNode *current_obj);
};

} // namespace cg

#endif
