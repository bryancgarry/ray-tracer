#include "scene/bounding_sphere_node.hpp"

namespace cg
{

BoundingSphereNode::BoundingSphereNode() : BoundingNode() {}

BoundingSphereNode::~BoundingSphereNode() {}

void BoundingSphereNode::set_bounding_sphere(const BoundingSphere &sphere)
{
    bounding_sphere_ = sphere;
}

void BoundingSphereNode::merge_bounding_sphere(const BoundingSphere &sphere)
{
    bounding_sphere_.merge_with(sphere);
}

void BoundingSphereNode::draw(SceneState &scene_state)
{
    // TBD: add frustum culling
    SceneNode::draw(scene_state);
}

void BoundingSphereNode::update(SceneState &scene_state)
{
    SceneNode::update(scene_state);
}

void BoundingSphereNode::find_closest_intersect(Ray3        ray,
                                                SceneState &current_state,
                                                SceneState &closest)
{
    if (ray.intersect(bounding_sphere_).intersects)
    {
        for (auto &child : children_)
            child->find_closest_intersect(ray, current_state, closest);
    }
}

bool BoundingSphereNode::does_intersect_exist(Ray3 ray, float d, SceneState &current_state)
{
    if (ray.intersect(bounding_sphere_).intersects)
    {
        for (auto &child : children_)
            if (child->does_intersect_exist(ray, d, current_state))
                return true;
    }
    return false;
}

} // namespace cg
