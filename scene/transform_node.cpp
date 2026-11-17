#include "scene/transform_node.hpp"

namespace cg
{

TransformNode::TransformNode()
{
    node_type_ = SceneNodeType::TRANSFORM;
    load_identity();
}

TransformNode::~TransformNode() {}

void TransformNode::load_identity() { model_matrix_.set_identity(); }

void TransformNode::translate(float x, float y, float z) { model_matrix_.translate(x, y, z); }

void TransformNode::rotate(float deg, const Vector3 &v) { model_matrix_.rotate(deg, v.x, v.y, v.z); }

void TransformNode::rotate_x(float deg) { model_matrix_.rotate_x(deg); }

void TransformNode::rotate_y(float deg) { model_matrix_.rotate_y(deg); }

void TransformNode::rotate_z(float deg) { model_matrix_.rotate_z(deg); }

void TransformNode::scale(float x, float y, float z) { model_matrix_.scale(x, y, z); }

void TransformNode::draw(SceneState &scene_state)
{
    // Copy current transforms onto stack
    scene_state.push_transforms();

    // Apply this modeling transform to the current modeling matrix.
    // Note the right-multiply - this allows hierarchical transformations
    // in the scene
    scene_state.model_matrix *= model_matrix_;
    glUniformMatrix4fv(scene_state.model_matrix_loc, 1, GL_FALSE, scene_state.model_matrix.get());

    // Set the normal transform matrix (transpose of the inverse of the model matrix).
    // This transforms normals into view coordinates
    Matrix4x4 normal_matrix = scene_state.model_matrix.get_inverse().transpose();
    glUniformMatrix4fv(scene_state.normal_matrix_loc, 1, GL_FALSE, normal_matrix.get());

    // Set the composite projection, view, modeling matrix
    Matrix4x4 pvm = scene_state.pv * scene_state.model_matrix;
    glUniformMatrix4fv(scene_state.pvm_matrix_loc, 1, GL_FALSE, pvm.get());

    // Draw all children
    SceneNode::draw(scene_state);

    // Pop matrix stack to revert to prior matrices
    scene_state.pop_transforms();
}

void TransformNode::update(SceneState &scene_state) {}

void TransformNode::find_closest_intersect(Ray3 ray, SceneState &current_state,
                                            SceneState &closest)
{
    // Save the three matrices that are about to change
    Matrix4x4 saved_model = current_state.model_matrix;
    Matrix4x4 saved_inverse = current_state.inverse_matrix;
    Matrix4x4 saved_normal = current_state.normal_matrix;

    current_state.model_matrix *= model_matrix_;
    current_state.inverse_matrix = current_state.model_matrix.get_inverse();
    current_state.normal_matrix = current_state.inverse_matrix.get_transpose();

    for (auto &child : children_)
        child->find_closest_intersect(ray, current_state, closest);

    current_state.model_matrix = saved_model;
    current_state.inverse_matrix = saved_inverse;
    current_state.normal_matrix = saved_normal;
}

bool TransformNode::does_intersect_exist(Ray3 ray, float d, SceneState &current_state)
{
    Matrix4x4 saved_model = current_state.model_matrix;
    Matrix4x4 saved_inverse = current_state.inverse_matrix;
    Matrix4x4 saved_normal = current_state.normal_matrix;

    current_state.model_matrix *= model_matrix_;
    current_state.inverse_matrix = current_state.model_matrix.get_inverse();
    current_state.normal_matrix = current_state.inverse_matrix.get_transpose();

    bool hit = false;
    for (auto &child : children_)
    {
        if(child->does_intersect_exist(ray, d, current_state))
        {
            hit = true;
            break;
        }
    }

    current_state.model_matrix = saved_model;
    current_state.inverse_matrix = saved_inverse;
    current_state.normal_matrix = saved_normal;
    return hit;
}

} // namespace cg
