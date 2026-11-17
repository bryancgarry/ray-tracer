#include "RayTracer/ray_tracer.hpp"
#include "RayTracer/image_texture.hpp"

namespace cg
{

RayTracer::RayTracer(std::shared_ptr<SceneNode> scene_root)
{
    scene_root_ = scene_root;

    // Initialize lighting support. Set the global ambient here.
    lighting_.set_ambient(Color3(0.25f, 0.25f, 0.25f));
}

RayTracer::~RayTracer() {}

Color3 RayTracer::trace_ray(Ray3 &initial_ray, int depth, float adaptive_threshold)
{
    Ray ray(initial_ray, depth, adaptive_threshold);
    return trace_ray(ray);
}

Color3 RayTracer::trace_ray(Ray &ray)
{
    // Check threshold
    if (ray.below_threshold()) {
        return Color3(0.1f, 0.1f, 0.3f);
    }

    // Initialize scene state
    SceneState current_state;
    current_state.model_matrix.set_identity();
    current_state.inverse_matrix.set_identity();
    current_state.normal_matrix.set_identity();
    current_state.t_scale = 1.0f;
    current_state.material_node = nullptr;
    current_state.texture_node = nullptr;

    SceneState closest;
    closest.t_min = std::numeric_limits<float>::max();
    closest.geometry_node = nullptr;
    closest.material_node = nullptr;
    closest.texture_node = nullptr;

    // Traverse the scene graph to find closest intersection
    scene_root_->find_closest_intersect(ray, current_state, closest);

    // If no object hit then return background value
    if (!closest.geometry_node) {
        return Color3(0.1f, 0.1f, 0.3f);
    }

    // Get the material
    MaterialNode *material = (MaterialNode *)closest.material_node;
    if (!material) {
        return Color3(1.0f, 0.0f, 1.0f); // Purple for missing material
    }

    // Get the normal and texture coord from closest — written there by the
    // geometry node during traversal, thread-safe, no mutable members needed
    Point3  int_pt = ray.intersect(closest.t_min);
    Vector3 normal = closest.hit_normal;

    // Get texture color if available
    Color3 texture_color(1.0f, 1.0f, 1.0f);
    if (closest.texture_node) {
        Point2 uv = closest.hit_texcoord;

        // ImageTexture
        auto* image_tex = dynamic_cast<cg::ImageTexture*>(closest.texture_node);
        if (image_tex) {
            TextureCoord2 tc;
            tc.s = std::max(0.0f, std::min(1.0f, uv.x));
            tc.t = std::max(0.0f, std::min(1.0f, uv.y));
            texture_color = image_tex->get_color(tc);
        } else {
            auto* proc_tex = dynamic_cast<cg::ProceduralTexture*>(closest.texture_node);
            if (proc_tex) {
                texture_color = proc_tex->get_color(int_pt);
            }
        }
    }

    // Start with ambient light contribution
    Color3 color = lighting_.get_ambient(material);

    // Add contribution from each light source
    for (auto& light : lights_) {
        if (!light) continue;

        Point3 light_pos = light->get_position();
        float shadow_factor = 1.0f; // 1.0 = full light, 0.0 = full shadow

        // Check if point is in shadow for this light
        if (in_shadow(int_pt, light_pos, closest.geometry_node)) {
            shadow_factor = 0.1f; // Still allow 10% of light through (for soft shadows)
        }

        Color3 diffuse, specular;
        lighting_.local_contribution(light.get(), material, int_pt, normal, diffuse, specular);

        // Modulate diffuse by texture color
        diffuse.r *= texture_color.r;
        diffuse.g *= texture_color.g;
        diffuse.b *= texture_color.b;

        // Apply shadow factor (shadows only affect diffuse, not specular highlights)
        color += (diffuse * shadow_factor) + (specular * shadow_factor);
    }

    // REFLECTION
    if (material->is_reflective() && ray.recursion_level_ > 0) {
        Vector3 incident = ray.d;
        incident.normalize();
        float   n_dot_i      = normal.dot(incident);
        Vector3 reflect_dir  = incident - normal * 2.0f * n_dot_i;
        reflect_dir.normalize();

        Ray3 reflected_ray;
        reflected_ray.o = int_pt + reflect_dir * 0.001f; // Epsilon
        reflected_ray.d = reflect_dir;

        Ray reflected_ray_wrapper(reflected_ray, ray.recursion_level_ - 1, ray.threshold_);
        Color3 reflected_color = trace_ray(reflected_ray_wrapper);

        Color3 global_refl = material->get_global_reflectivity();
        color.r += reflected_color.r * global_refl.r;
        color.g += reflected_color.g * global_refl.g;
        color.b += reflected_color.b * global_refl.b;
    }

    // REFRACTION/TRANSPARENCY
    if (material->is_transparent() && ray.recursion_level_ > 0) {
        bool total_internal_reflection = false;
        Ray refracted_ray = ray.get_refracted_ray(int_pt, normal, material, total_internal_reflection);

        if (!total_internal_reflection) {
            refracted_ray.o = refracted_ray.o + refracted_ray.d * 0.001f; // EPSILON
            Color3 refracted_color = trace_ray(refracted_ray);

            Color3 global_trans = material->get_global_transmission();
            color.r += refracted_color.r * global_trans.r;
            color.g += refracted_color.g * global_trans.g;
            color.b += refracted_color.b * global_trans.b;
        }
    }

    // Clamp and return color from traced ray
    color.clamp();
    return color;
}

void RayTracer::set_view_position(const Point3 &pos) { lighting_.set_view_position(pos); }

bool RayTracer::in_shadow(const Point3 &int_pt, const Point3 &light_pos, SceneNode *current_obj)
{
    Vector3 to_light = light_pos - int_pt;
    float light_dist = to_light.norm();
    to_light.normalize();

    // Create shadow ray offset slightly from the surface to avoid self-intersection
    Ray3 shadow_ray;
    shadow_ray.o = int_pt + to_light * 0.001f;
    shadow_ray.d = to_light;

    // Initialize scene state
    SceneState current_state;
    current_state.model_matrix.set_identity();
    current_state.inverse_matrix.set_identity();
    current_state.normal_matrix.set_identity();
    current_state.t_scale = 1.0f;
    current_state.material_node = nullptr;
    current_state.texture_node  = nullptr;

    return scene_root_->does_intersect_exist(shadow_ray, light_dist - 0.001f, current_state);
}

void RayTracer::add_light(std::shared_ptr<LightNode> light)
{
    lights_.push_back(light);
}

} // namespace cg

//#include "RayTracer/ray_tracer.hpp"
//#include "RayTracer/image_texture.hpp"
//
//namespace cg
//{
//
//RayTracer::RayTracer(std::shared_ptr<SceneNode> scene_root)
//{
//    scene_root_ = scene_root;
//
//    // Initialize lighting support. Set the global ambient here.
//    lighting_.set_ambient(Color3(0.25f, 0.25f, 0.25f));
//}
//
//RayTracer::~RayTracer() {}
//
//Color3 RayTracer::trace_ray(Ray3 &initial_ray, int depth, float adaptive_threshold)
//{
//    Ray ray(initial_ray, depth, adaptive_threshold);
//    return trace_ray(ray);
//}
//
//Color3 RayTracer::trace_ray(Ray &ray)
//{
//    // Check threshold
//    if (ray.below_threshold()) {
//        return Color3(0.1f, 0.1f, 0.3f);
//    }
//
//    // Initialize scene state
//    SceneState current_state;
//    current_state.model_matrix.set_identity();
//    current_state.inverse_matrix.set_identity();
//    current_state.normal_matrix.set_identity();
//    current_state.t_scale = 1.0f;
//    current_state.material_node = nullptr;
//    current_state.texture_node = nullptr;
//
//    SceneState closest;
//    closest.t_min = std::numeric_limits<float>::max();
//    closest.geometry_node = nullptr;
//    closest.material_node = nullptr;
//    closest.texture_node = nullptr;
//
//    // Traverse the scene graph to find closest intersection
//    scene_root_->find_closest_intersect(ray, current_state, closest);
//
//    // If no object hit then return background value
//    if (!closest.geometry_node) {
//        return Color3(0.1f, 0.1f, 0.3f);
//    }
//
//    // Get the material
//    MaterialNode *material = (MaterialNode *)closest.material_node;
//    if (!material) {
//        return Color3(1.0f, 0.0f, 1.0f); // Purple for missing material
//    }
//
//    // Find the intersection point and normal
//    Point3 int_pt = ray.intersect(closest.t_min);
//    Vector3 normal;
//
//    if (closest.geometry_node) {
//        auto* geo = static_cast<cg::GeometryNode*>(closest.geometry_node);
//        normal = geo->get_normal(int_pt);
//    }
//
//    // Get texture color if available
//    Color3 texture_color(1.0f, 1.0f, 1.0f);
//    if (closest.texture_node) {
//        auto* geo = static_cast<cg::GeometryNode*>(closest.geometry_node);
//        Point2 uv = geo->get_texture_coord(int_pt);
//
//        // ImageTexture
//        auto* image_tex = dynamic_cast<cg::ImageTexture*>(closest.texture_node);
//        if (image_tex) {
//            TextureCoord2 tc;
//            tc.s = std::max(0.0f, std::min(1.0f, uv.x));
//            tc.t = std::max(0.0f, std::min(1.0f, uv.y));
//            texture_color = image_tex->get_color(tc);
//        } else {
//            // ProceduralTexture
//            auto* proc_tex = dynamic_cast<cg::ProceduralTexture*>(closest.texture_node);
//            if (proc_tex) {
//                texture_color = proc_tex->get_color(int_pt);
//            }
//        }
//    }
//
//    // Start with ambient light contribution
//    Color3 color = lighting_.get_ambient(material);
//
//    // Add contribution from each light source
//    for (auto& light : lights_) {
//        if (!light) continue;
//
//        Point3 light_pos = light->get_position();
//        float shadow_factor = 1.0f; // 1.0 = full light, 0.0 = full shadow
//
//        // Check if point is in shadow for this light
//        if (in_shadow(int_pt, light_pos, closest.geometry_node)) {
//            shadow_factor = 0.1f; // Still allow 10% of light through (for soft shadows)
//        }
//
//        Color3 diffuse, specular;
//        lighting_.local_contribution(light.get(), material, int_pt, normal, diffuse, specular);
//
//        // Modulate diffuse by texture color
//        diffuse.r *= texture_color.r;
//        diffuse.g *= texture_color.g;
//        diffuse.b *= texture_color.b;
//
//        // Apply shadow factor (shadows only affect diffuse, not specular highlights)
//        color += (diffuse * shadow_factor) + (specular * shadow_factor);
//    }
//
//    // REFLECTION
//    if (material->is_reflective() && ray.recursion_level_ > 0) {
//        Vector3 incident = ray.d;
//        incident.normalize();
//        float n_dot_i = normal.dot(incident);
//        Vector3 reflect_dir = incident - normal * 2.0f * n_dot_i;
//        reflect_dir.normalize();
//
//        Ray3 reflected_ray;
//        reflected_ray.o = int_pt + reflect_dir * 0.001f; // Epsilon
//        reflected_ray.d = reflect_dir;
//
//        Ray reflected_ray_wrapper(reflected_ray, ray.recursion_level_ - 1, ray.threshold_);
//        Color3 reflected_color = trace_ray(reflected_ray_wrapper);
//
//        Color3 global_refl = material->get_global_reflectivity();
//        color.r += reflected_color.r * global_refl.r;
//        color.g += reflected_color.g * global_refl.g;
//        color.b += reflected_color.b * global_refl.b;
//    }
//
//    // REFRACTION/TRANSPARENCY
//    if (material->is_transparent() && ray.recursion_level_ > 0) {
//        bool total_internal_reflection = false;
//        Ray refracted_ray = ray.get_refracted_ray(int_pt, normal, material, total_internal_reflection);
//
//        if (!total_internal_reflection) {
//            Color3 refracted_color = trace_ray(refracted_ray);
//
//            Color3 global_trans = material->get_global_transmission();
//            color.r += refracted_color.r * global_trans.r;
//            color.g += refracted_color.g * global_trans.g;
//            color.b += refracted_color.b * global_trans.b;
//        }
//    }
//
//    // Clamp and return color from traced ray
//    color.clamp();
//    return color;
//}
//
//void RayTracer::set_view_position(const Point3 &pos) { lighting_.set_view_position(pos); }
//
//bool RayTracer::in_shadow(const Point3 &int_pt, const Point3 &light_pos, SceneNode *current_obj)
//{
//    Vector3 to_light = light_pos - int_pt;
//    float light_dist = to_light.norm();
//    to_light.normalize();
//
//    // Create shadow ray offset slightly from the surface to avoid self-intersection
//    Ray3 shadow_ray;
//    shadow_ray.o = int_pt + to_light * 0.001f;
//    shadow_ray.d = to_light;
//
//    // Initialize scene state
//    SceneState current_state;
//    current_state.model_matrix.set_identity();
//    current_state.inverse_matrix.set_identity();
//    current_state.normal_matrix.set_identity();
//    current_state.t_scale = 1.0f;
//    current_state.material_node = nullptr;
//    current_state.texture_node  = nullptr;
//
//    return scene_root_->does_intersect_exist(shadow_ray, light_dist - 0.001f, current_state);
//}
//
//void RayTracer::add_light(std::shared_ptr<LightNode> light)
//{
//    lights_.push_back(light);
//}
//
//} // namespace cg
