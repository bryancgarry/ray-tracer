#include "RayTracer/procedural_texture.hpp"

namespace cg
{

ProceduralTexture::ProceduralTexture() { node_type_ = SceneNodeType::PROCEDURAL_TEXTURE; }

void ProceduralTexture::find_closest_intersect(Ray3 ray, SceneState &current_state, SceneState &closest)
{
    // Push current texture onto stack
    current_state.push_texture();
    
    // Set this texture as current
    current_state.texture_node = this;
    
    // Traverse children
    for (auto& child : children_) {
        child->find_closest_intersect(ray, current_state, closest);
    }
    
    // Pop texture from stack - restores previous texture state
    current_state.pop_texture();
}

} // namespace cg
