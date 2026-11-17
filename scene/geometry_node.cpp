#include "scene/geometry_node.hpp"

namespace cg
{

GeometryNode::GeometryNode() { node_type_ = SceneNodeType::GEOMETRY; }

GeometryNode::~GeometryNode() {}

void GeometryNode::draw(SceneState &scene_state) {}

void GeometryNode::find_closest_intersect(Ray3 ray, SceneState &current_state,
                                         SceneState &closest)
{
    // Base class does nothing - only geometry with ray tracing support overrides
    // This allows old geometry classes to continue working without ray tracing
}

bool GeometryNode::does_intersect_exist(Ray3 ray, float d, SceneState &current_state)
{
    // Base class returns false - no intersection
    return false;
}

Vector3 GeometryNode::get_normal(const Point3 &point)
{
    // Default normal pointing up
    return Vector3(0, 1, 0);
}

Point2 GeometryNode::get_texture_coord(const Point3 &point)
{
    // Default texture coordinate
    return Point2(0, 0);
}

} // namespace cg
