#include "scene/bounding_aabb_node.hpp"

namespace cg
{

AABBNode::AABBNode() : BoundingNode() {}

AABBNode::~AABBNode() {}

void AABBNode::set(const Point3 &min_pt, const Point3 &max_pt) { box_.update(min_pt, max_pt); }

void AABBNode::draw(SceneState &scene_state)
{
    // TBD: add frustum culling
    SceneNode::draw(scene_state);
}

void AABBNode::update(SceneState &scene_state)
{
    SceneNode::update(scene_state);
}

void AABBNode::find_closest_intersect(Ray3 ray, SceneState &current_state, SceneState &closest)
{
    // The AABB is in world space, test directly with the world space ray
    if (ray.intersect(box_).intersects)
    {
        for (auto &child : children_)
            child->find_closest_intersect(ray, current_state, closest);
    }
}

bool AABBNode::does_intersect_exist(Ray3 ray, float d, SceneState &current_state)
{
    if (ray.intersect(box_).intersects)
    {
        for (auto &child : children_)
            if (child->does_intersect_exist(ray, d, current_state))
                return true;
    }
    return false;
}

} // namespace cg
