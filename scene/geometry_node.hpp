//============================================================================
//	Johns Hopkins University Engineering Programs for Professionals
//	605.667 Computer Graphics and 605.767 Applied Computer Graphics
//	Instructor:	Brian Russin
//
//	Author:	 David W. Nesbitt
//	File:    geometry_node.hpp
//	Purpose: Scene graph geometry node.
//
//============================================================================

#ifndef __SCENE_GEOMETRY_NODE_HPP__
#define __SCENE_GEOMETRY_NODE_HPP__

#include "scene/scene_node.hpp"
#include "RayTracer/ray.hpp"

namespace cg
{

/**
 * Geometry node base class. Stores and draws geometry.
 */
class GeometryNode : public SceneNode
{
  public:
    /**
     * Constructor
     */
    GeometryNode();

    /**
     * Destructor
     */
    virtual ~GeometryNode();

    /**
     * Draw this geometry node. Geometry nodes are leaf nodes and have no children.
     * @param  scene_state  Current scene state
     */
    virtual void draw(SceneState &scene_state) override;
    
    /**
     * Find closest intersection - base implementation does nothing.
     * Derived classes override to implement ray tracing.
     */
    virtual void find_closest_intersect(Ray3 ray, SceneState &current_state,
                                       SceneState &closest) override;
    
    /**
     * Fast intersection test - base implementation returns false.
     * Derived classes override for shadow ray support.
     */
    virtual bool does_intersect_exist(Ray3 ray, float d,
                                     SceneState &current_state) override;
    
    /**
     * Get surface normal - base implementation returns default normal.
     * Derived classes override to return actual surface normal.
     */
    virtual Vector3 get_normal(const Point3 &point);
    
    /**
     * Get texture coordinate - base implementation returns (0,0).
     * Derived classes override for texture mapping support.
     */
    virtual Point2 get_texture_coord(const Point3 &point);
};

} // namespace cg

#endif
